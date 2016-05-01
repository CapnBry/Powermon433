#include <Arduino.h>
#include <util/atomic.h>
#include "rf69_ook.h"

#define SPI_SS      10    // PB2, pin 16
#define SPI_MOSI    11    // PB3, pin 17
#define SPI_MISO    12    // PB4, pin 18
#define SPI_SCK     13    // PB5, pin 19

static uint8_t rf69ook_byte(uint8_t v)
{
  SPDR = v;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

void rf69ook_writeReg(uint8_t reg, uint8_t val)
{
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    digitalWrite(SPI_SS, LOW);
    rf69ook_byte(reg | 0x80);
    rf69ook_byte(val);
    digitalWrite(SPI_SS, HIGH);
  }
}

uint8_t rf69ook_readReg(uint8_t reg)
{
  uint8_t retVal;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    digitalWrite(SPI_SS, LOW);
    rf69ook_byte(reg);
    retVal = rf69ook_byte(0x00);
    digitalWrite(SPI_SS, HIGH);
  }
  return retVal;
}

bool rf69ook_init(void)
{
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_SCK, OUTPUT);

  SPCR = _BV(SPE) | _BV(MSTR);
  SPSR |= _BV(SPI2X);

  delay(50);
  uint8_t deviceId = rf69ook_readReg(0x10);
  // Should be 0x24 but as long as it isn't blank
  if (deviceId == 0x00 || deviceId == 0xff)
    return false;

  rf69ook_writeReg(0x01, 0x04); // mode standby
  //rf69ook_writeReg(0x02, 0b01101000); // no shaping, OOK modulation, continuous mode with no sync
  rf69ook_writeReg(0x02, 0b01001000); // no shaping, OOK modulation, continuous mode with sync
  rf69ook_writeReg(0x03, 0x3E); // bitrate=
  rf69ook_writeReg(0x04, 0x80); // 2000 bit = 500uS per bit
  // Frequency 433.845MHz / (32M >> 19)
  rf69ook_writeReg(0x07, 0x6c);
  rf69ook_writeReg(0x08, 0x76);
  rf69ook_writeReg(0x09, 0x15);

  // bandwidth
  //rf69ook_writeReg(0x19, 0b01000000);  // Stock DdcFreq, 250khz
  //rf69ook_writeReg(0x19, 0b01010000); // Stock DdcFreq, 166khz
  //rf69ook_writeReg(0x19, 0b01001001); // Stock DdcFreq, 100Khz
  //rf69ook_writeReg(0x19, 0b01001010);  // Stock DdcFreq, 50khz
  rf69ook_writeReg(0x19, 0b01001011);  // Stock DdcFreq, 25khz
  //rf69ook_writeReg(0x19, 0b01001100);  // Stock DdcFreq, 12.5khz
  //rf69ook_writeReg(0x19, 0b01001101); // 6.3khz
  //rf69ook_writeReg(0x19, 0b01001110); // 3.1khz
  //rf69ook_writeReg(0x19, 0b01001111); // 1.3khz

  //rf69ook_writeReg(0x18, 0x80); // 200 ohm auto lna gain
  rf69ook_writeReg(0x18, 0x08); // 50 ohm auto gain (default)
  //rf69ook_writeReg(0x18, 0x81); // 200 ohm lna max gain
  //rf69ook_writeReg(0x18, 0x01); // 50ohm lna max gain
  //rf69ook_writeReg(0x18, 0x08); // -12db gain

  rf69ook_writeReg(0x1b, 0b01000000); //peak OOK threshold (default)
  //rf69ook_writeReg(0x1b, 0b00000000); //fixed OOK threshold
  rf69ook_writeReg(0x1d, 0x30); // OokFixedThresh (noise floor when used in peak mode)

  //rf69ook_writeReg(0x29, 0xe4); // RssiThreshold (default)
  //rf69ook_writeReg(0x2e, 0x18); // SYNC off, sync size 3, 0 errors
  //rf69ook_writeReg(0x2e, 0x98); // SYNC on, sync size 3, 0 errors (default)
  

  // Packet Mode Setup (not working)
  //rf69ook_writeReg(0x02, 0b00001000); // no shaping ook modulation packet mode
  //rf69ook_writeReg(0x25, 0x40); // DIO0 = PayloadReady
  //rf69ook_writeReg(0x2e, 0x00); // Sync word off
  //rf69ook_writeReg(0x37, 0x00); // Packet fixed length, no CRC, no address filter
  //rf69ook_writeReg(0x38, 0x02); // Payload length=2

  rf69ook_writeReg(0x01, 0x10); // mode Rx
  rf69ook_writeReg(0x1e, bit(1)); // AFC clear, must be set when in RX?

  return true;
}

void rf69ook_startRssi(void)
{
  rf69ook_writeReg(0x23, 0x01);
}

uint8_t rf69ook_Rssi(void)
{
  return rf69ook_readReg(0x24);
}

void rf69ook_dumpRegs(void)
{
  for (uint8_t i=0; i<0x30; ++i)
  {
    uint8_t v = rf69ook_readReg(i);
    if (v < 0x10)
      Serial.print('0');
    Serial.print(v, HEX);
    if (i % 16 == 15)
      Serial.println();
    else
      Serial.print(' ');
  }
}
