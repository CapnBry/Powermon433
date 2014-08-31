/*
  Powermon433
  
  Monitoring 433MHz power monitor products (both the same internally)
  -- Black and Decker EM100B 
  -- BlueLine PowerCost Monitor

  Original OOK interrupt and decoder based on jeelib "ookRelay2" project https://github.com/jcw/jeelib
  Protocol decoding by "redgreen" https://github.com/merbanan/rtl_433
    via Stewart Russell's blog http://scrus.org/
  Additional work and porting to ATmega by Bryan Mayland
*/
#include <digitalWriteFast.h>
#include <util/atomic.h>
#include "rf69_ook.h"

//#define DPIN_RF69_RESET 7
#define DPIN_OOK_RX     8
//#define DPIN_OOK_TX 3
#define DPIN_STARTTX_BUTTON 7
#define DPIN_LED        9

// The default ID of the transmitter to decode/encode from/to
#define DEFAULT_TX_ID 0xfdcc

// If defined will dump all the encoded RX data, and partial decode fails
#define DUMP_RX

static uint16_t g_TxId;
static uint8_t g_TxCnt;

// Simulated TX values
static uint16_t g_TxTemperature;
static uint8_t g_TxFlags;
static uint8_t g_TxLowBat;
static uint16_t g_TxWatts;
static uint16_t g_TxTotal;

static char g_SerialBuff[40];

static struct tagWireVal
{
  uint8_t hdr;
  union {
    uint8_t raw[2];
    uint16_t val16;
  } data;
  uint8_t crc;
} wireval;

static struct tagDecoder
{
  uint8_t state;
  uint8_t pos;
  uint8_t bit;
  uint8_t data[4];
} decoder;

#if defined(DPIN_OOK_RX)
static int8_t g_RxTemperature;
static uint8_t g_RxFlags;
static uint16_t g_RxWatts;
static uint16_t g_RxWattHours;

static bool g_RxDirty;
static uint32_t g_RxLast;
static uint8_t g_RxRssi;

#if DPIN_OOK_RX >= 14
#define VECT PCINT1_vect
#elif DPIN_OOK_RX >= 8
#define VECT PCINT0_vect
#else
#define VECT PCINT2_vect
#endif

volatile uint16_t pulse_433;

static void pinChange(void)
{
  static uint16_t last_433;

  uint16_t now = micros();
  uint16_t cnt = now - last_433;
  if (cnt > 10)
    pulse_433 = cnt;

  last_433 = now;
}

ISR(VECT) {
  pinChange();
}

static void setupPinChangeInterrupt ()
{
  pinModeFast(DPIN_OOK_RX, INPUT);
#if DPIN_OOK_RX >= 14
  bitSet(PCMSK1, DPIN_OOK_RX - 14);
  bitSet(PCICR, PCIE1);
#elif DPIN_OOK_RX >= 8
  bitSet(PCMSK0, DPIN_OOK_RX - 8);
  bitSet(PCICR, PCIE0);
#else
  PCMSK2 = bit(DPIN_OOK_RX);
  bitSet(PCICR, PCIE2);
#endif
}
#endif /* DPIN_OOK_RX */

static void printWireval(void)
{
  Serial.print("Tx ");
  for (uint8_t i=0; i<sizeof(wireval); ++i)
  {
    uint8_t v = ((uint8_t *)&wireval)[i];
    if (v < 0x10)
      Serial.print('0');
    Serial.print(v, HEX);
  }
  Serial.println();
  Serial.flush();
}

// Short burst in uSec
#define OOK_TX_SHORT 500
// Long burst in uSec
#define OOK_TX_LONG  1000
// Inter-packet delay in msec
#define OOK_TX_DELAY 65
// Inter-ID-packet delay in msec
#define OOK_ID_DELAY 225

#define OOK_PACKET_INSTANT 1
#define OOK_PACKET_TEMP    2
#define OOK_PACKET_TOTAL   3

/* crc8 from chromimum project */
 __attribute__((noinline)) uint8_t crc8(uint8_t const *data, uint8_t len)
{
  uint16_t crc = 0;
  for (uint8_t j=0; j<len; ++j)
  {
    crc ^= (data[j] << 8);
    for (uint8_t i=8; i>0; --i)
    {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return crc >> 8;
}

#if defined(DPIN_OOK_TX)
static void TxShortPulse(void)
{
  digitalWriteFast(DPIN_OOK_TX, HIGH);
  delayMicroseconds(OOK_TX_SHORT);
  digitalWriteFast(DPIN_OOK_TX, LOW);
}

static void TxRaw(uint8_t v)
{
  for (uint8_t i=0; i<8; ++i)
  {
    TxShortPulse();
    if (v & 0x80)
      delayMicroseconds(OOK_TX_SHORT);
    else
      delayMicroseconds(OOK_TX_LONG);
    v <<= 1;
  }
}

static void TxWireval(uint16_t txId)
{
  // CRC is calculated before txId is added which allows multiple transmitters
  // as the transmitter the receiver isn't tracking will just see the other
  // data is CRC errors
  wireval.crc = crc8(wireval.data.raw, sizeof(wireval.data.raw));
  wireval.data.val16 += txId;

  printWireval();

  // The header is 11111110 and a short duration pause
  // Which will come out as SS SS SS SS SS SS SS SL*1.5
  TxRaw(wireval.hdr);
  delayMicroseconds(OOK_TX_SHORT);
  TxRaw(wireval.data.raw[0]);
  TxRaw(wireval.data.raw[1]);
  TxRaw(wireval.crc);
  // Packet ends with a short pulse otherwise there's no telling what the last
  // bit value was
  TxShortPulse();
}

static void TxIdOnce(uint16_t txId)
{
  // An ID packet is just one where the CRC is good before the ID
  // is subtracted
  wireval.data.val16 = txId;
  TxWireval(0);
}

static void TxInstantOnce(uint16_t txId, uint16_t val)
{
  // last two bits are reserved for packet type
  wireval.data.val16 = (val & 0xfffc) | OOK_PACKET_INSTANT;
  TxWireval(txId);
}

static void TxTempOnce(uint16_t txId, uint8_t temp, uint8_t lowBat)
{
  Serial.print(F("Temperature "));
  wireval.data.raw[0] = (lowBat << 7) | g_TxFlags | OOK_PACKET_TEMP;
  wireval.data.raw[1] = temp;
  TxWireval(txId);
}

static void TxTotalOnce(uint16_t txId, uint16_t val)
{
  Serial.print(F("Total "));
  // last two bits are reserved for packet type
  wireval.data.val16 = (val & 0xfffc) | OOK_PACKET_TOTAL;
  TxWireval(txId);
}

static uint16_t wattsToCnt(uint16_t watts)
{
  return 3600000UL / watts;
}

static uint8_t tempFToCnt(float temp)
{
  return (temp + 28.63) / 0.823;
}
#endif // defined(DPIN_OOK_TX)

static void setRF69Freq(char *buf)
{
  static uint32_t khz;
  if (*buf == '-')
    khz -= 5;
  else if (*buf == '+')
    khz += 5;
  else
    khz = atol(buf);

  Serial.print(khz, DEC); Serial.println(F("kHz"));

  uint32_t val = khz * (524288.0 / 32000.0);
  rf69ook_writeReg(0x01, 0x04); // standby
  rf69ook_writeReg(0x07, (val >> 16) & 0xff);
  rf69ook_writeReg(0x08, (val >> 8) & 0xff);
  rf69ook_writeReg(0x09, val & 0xff);
  rf69ook_writeReg(0x01, 0x10); // RX
}

static void setRf69Thresh(uint8_t val)
{
  Serial.print(F("OokFixedThresh="));
  Serial.println(val, DEC);
  rf69ook_writeReg(0x1d, val);
}

static void resetRf69(void)
{
#if defined(DPIN_RF69_RESET)
  const uint8_t pin = 7;
  pinModeFast(pin, OUTPUT);
  digitalWriteFast(pin, HIGH);
  delayMicroseconds(100);
  digitalWriteFast(pin, LOW);
  pinModeFast(pin, INPUT);
  delay(5);
  Serial.println(F("RFM reset"));
#endif // DPIN_RF69_RESET
}

static void handleCommand(void)
{
  switch (g_SerialBuff[0])
  {
  case 'b':
    Serial.print(F("Batt"));
    if (g_SerialBuff[1] != '?')
      g_TxLowBat = atoi(&g_SerialBuff[1]);
    Serial.println(g_TxLowBat, DEC);
    g_TxCnt = 4; // force next packet is temperature
    break;
  case 'f':
    Serial.print(F("Flags"));
    if (g_SerialBuff[1] != '?')
      g_TxFlags = atoi(&g_SerialBuff[1]);
    Serial.println(g_TxFlags, DEC);
    g_TxCnt = 4; // force next packet is temperature
    break;
  case 'k':
    Serial.print(F("TotalkW"));
    if (g_SerialBuff[1] != '?')
      g_TxTotal = atoi(&g_SerialBuff[1]);
    Serial.println(g_TxTotal, DEC);
    g_TxCnt = 5; // force next packet is total
    break;
  case 'q':
    Serial.print(F("freQ"));
    setRF69Freq(&g_SerialBuff[1]);
    break;
  case 't':
    Serial.print(F("Temp"));
    if (g_SerialBuff[1] != '?')
      g_TxTemperature = atoi(&g_SerialBuff[1]);
    Serial.println(g_TxTemperature, DEC);
    g_TxCnt = 4; // force next packet is temperature
    break;
  case 'w':
    Serial.print(F("InstW"));
    if (g_SerialBuff[1] != '?')
      g_TxWatts = atoi(&g_SerialBuff[1]);
    Serial.println(g_TxWatts, DEC);
    g_TxCnt = 0; // force next packet is usage
    break;
  case 'x':
    Serial.print(F("TxId"));
    if (g_SerialBuff[1] != '?')
      g_TxId = atoi(&g_SerialBuff[1]);
    Serial.println(g_TxId, DEC);
    break;
  case 'z':
    rf69ook_dumpRegs();
    Serial.print(F("RX:")); Serial.println(digitalReadFast(DPIN_OOK_RX));
    break;
  case ']':
    setRf69Thresh(rf69ook_readReg(0x1d)+4);
    break;
  case '[':
    setRf69Thresh(rf69ook_readReg(0x1d)-4);
    break;
  case '*':
    resetRf69();
    break;
  }
}

static void serial_doWork(void)
{
  while (Serial.available())
  {
    unsigned char len = strlen(g_SerialBuff);
    char c = Serial.read();
    // support CR, LF, or CRLF line endings
    if (c == '\n' || c == '\r')  
    {
      if (len != 0)
        handleCommand();
      len = 0;
    }
    else {
      g_SerialBuff[len++] = c;
      // if the buffer fills without getting a newline, just reset
      if (len >= sizeof(g_SerialBuff))
        len = 0;
    }
    g_SerialBuff[len] = '\0';
  }  /* while Serial */
}

static void resetDecoder(void)
{
  decoder.pos = 0;
  decoder.bit = 0;
  decoder.state = 0;
}

static void decoderAddBit(uint8_t bit)
{
  decoder.data[decoder.pos] = (decoder.data[decoder.pos] << 1) | bit;
  if (++decoder.bit > 7)
  {
    decoder.bit = 0;
    if (++decoder.pos >= sizeof(decoder.data))
      resetDecoder();
  }
}

static bool decodeRxPulse(uint16_t width)
{
  // 500,1000,1500 usec pulses with 25% tolerance
  if (width > 375 && width < 1875)
  {
    // The only "extra long" long signals the end of the preamble
    if (width > 1200)
    {
      rf69ook_startRssi();
      resetDecoder();
      return false;
    }

    bool isShort = width < 750;
    if (decoder.state == 0)
    {
      // expecting a short to start a bit
      if (isShort)
      {
        decoder.state = 1;
        return false;
      }
    }
    else if (decoder.state == 1)
    {
      decoder.state = 0;
      if (isShort)
        decoderAddBit(1);
      else
        decoderAddBit(0);

      // If we have all 3 bytes, we're done
      if (decoder.pos > 2)
        return true;
      return false;
    }
  }  // if proper width

#if defined(DUMP_RX)
  // Some debug dump of where the decoder went wrong
  if ((decoder.pos * 8 + decoder.bit) > 2)
  {
    Serial.print(width, DEC);
    Serial.print('@');
    Serial.println(decoder.pos * 8 + decoder.bit, DEC);
  }
#endif

  resetDecoder();
  return false;
}

static void decodePowermon(uint16_t val16)
{
  switch (decoder.data[0] & 3)
  {
  case 0: 
  case 1:
    // val16 is the number of milliseconds between blinks
    // Each blink is one watt hour consumed
    g_RxWatts = 3600000UL / val16;
    break;

  case 2:
    //float f = decoder.data[1] * 0.823 - 28.63;
    g_RxTemperature = (int8_t)(decoder.data[1] * 210U / 256U) - 28;
    g_RxFlags = decoder.data[0];
    break;

  case 3:
    g_RxWattHours = val16;
    break;
  }
}

static void printRssi(void)
{
  Serial.print(F(" (Rssi -")); Serial.print(g_RxRssi/2, DEC);
  Serial.println(F(" dBm)"));
}

static void decodeRxPacket(void)
{
#if defined(DUMP_RX)
  for (uint8_t i=0; i<decoder.pos; ++i)
  {
    if (decoder.data[i] < 16)
      Serial.print('0');
    Serial.print(decoder.data[i], HEX);
  }
  Serial.println();
#endif

  uint16_t val16 = *(uint16_t *)decoder.data;
  if (crc8(decoder.data, 3) == 0)
  {
    g_TxId = decoder.data[1] << 8 | decoder.data[0];
    Serial.print(F("NEW DEVICE id="));
    Serial.print(val16, HEX);
    printRssi();
    return;
  }

  val16 -= g_TxId;
  decoder.data[0] = val16 & 0xff;
  decoder.data[1] = val16 >> 8;
  if (crc8(decoder.data, 3) == 0)
  {
    decodePowermon(val16 & 0xfffc);
    g_RxDirty = true;
    g_RxLast = millis();
    digitalWriteFast(DPIN_LED, HIGH);
  }
  else
  {
    Serial.print(F("CRC ERR"));
    printRssi();
  }
}

static void txSetup(void)
{
#if defined(DPIN_OOK_TX)
  pinModeFast(DPIN_OOK_TX, OUTPUT);
  pinModeFast(DPIN_STARTTX_BUTTON, INPUT);
  digitalWriteFast(DPIN_STARTTX_BUTTON, HIGH);

  wireval.hdr = 0xfe;
  g_TxTemperature = tempFToCnt(116.22);
  g_TxFlags = 0x7c;
  g_TxWatts = 5000;
  g_TxTotal = 60000;
#endif
}

static void rxSetup(void)
{
#if defined(DPIN_OOK_RX)
  setupPinChangeInterrupt();
#endif
}

static void ookTx(void)
{
#if defined(DPIN_OOK_TX)
  static uint32_t g_LastTx;

  if (digitalReadFast(DPIN_STARTTX_BUTTON) == LOW)
  {
    for (uint8_t i=0; i<4; ++i)
    {
      TxIdOnce(g_TxId);
      delay(OOK_ID_DELAY);
    }

    g_LastTx = millis();
  }

  if (g_LastTx != 0 && millis() - g_LastTx > 30500)
  {
    digitalWrite(DPIN_LED, HIGH);
    for (uint8_t i=0; i<3; ++i)
    {
      if (i == 2 || (g_TxCnt >= 0 && g_TxCnt < 4))
        TxInstantOnce(g_TxId, wattsToCnt(g_TxWatts));
      else if (g_TxCnt == 4)
        TxTempOnce(g_TxId, g_TxTemperature, g_TxLowBat);
      else if (g_TxCnt == 5)
        TxTotalOnce(g_TxId, g_TxTotal);
      delay(OOK_TX_DELAY);
    }

    ++g_TxCnt;
    if (g_TxCnt > 5)
      g_TxCnt = 0;

    digitalWriteFast(DPIN_LED, LOW);
    g_LastTx = millis();
  }
#endif //defined(DPIN_OOK_TX)
}

static void ookRx(void)
{
#if defined(DPIN_OOK_RX)
  uint16_t v;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    v = pulse_433;
    pulse_433 = 0;
  }
  if (v != 0)
  {
    if (decodeRxPulse(v) == 1)
    {
      g_RxRssi = rf69ook_Rssi();
      decodeRxPacket();
      resetDecoder();
    }
  }

  // If it has been more than 250ms since the last receive, dump the data
  else if (g_RxDirty && (millis() - g_RxLast) > 250U)
  {
 #if defined(DUMP_RX)
   Serial.print('['); Serial.print(millis(), DEC); Serial.print(F("] "));
#endif
    Serial.print(F("Energy: ")); Serial.print(g_RxWattHours, DEC);
    Serial.print(F(" Wh, Power: ")); Serial.print(g_RxWatts, DEC);
    Serial.print(F(" W, Temp: ")); Serial.print(g_RxTemperature, DEC);
    Serial.print(F(" F"));
    printRssi();

    g_RxDirty = false;
    digitalWriteFast(DPIN_LED, LOW);
  }
 #if defined(DUMP_RX)
  else if (g_RxLast != 0 && (millis() - g_RxLast) > 32000U)
  {
    Serial.print('['); Serial.print(millis(), DEC); Serial.println(F("] Missed"));
    g_RxLast = millis();
  }
#endif
#endif // DPIN_OOK_RX
}

void setup() {
  Serial.begin(38400);
  Serial.println(F("$UCID,Powermon433,"__DATE__" "__TIME__));

  pinModeFast(DPIN_LED, OUTPUT);
  if (rf69ook_init())
    Serial.println(F("RF69 initialized"));

  txSetup();
  rxSetup();

  g_TxId = DEFAULT_TX_ID;
}

void loop()
{
  ookTx();
  ookRx();
  serial_doWork();
}