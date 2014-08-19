#ifndef __RF69_OOK_H__
#define __RF69_OOK_H__

void rf69ook_writeReg(uint8_t reg, uint8_t val);
uint8_t rf69ook_readReg(uint8_t reg);

bool rf69ook_init(void);
void rf69ook_startRssi(void);
uint8_t rf69ook_Rssi(void);
void rf69ook_dumpRegs(void);

#endif  //__RFM69_OOK_H__