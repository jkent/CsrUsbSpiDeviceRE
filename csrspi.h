#ifndef _CSRSPI_H
#define _CSRSPI_H

extern uint16_t g_nSpeed;
extern uint16_t g_nReadBits;
extern uint16_t g_nWriteBits;
extern uint16_t g_nBcA;
extern uint16_t g_nBcB;
extern uint16_t g_nUseSpecialRead;

void CsrInit(void);
bool CsrSpiRead(uint16_t nAddress, uint16_t nLength, uint16_t *pnOutput);
void CsrSpiWrite(uint16_t nAddress, uint16_t nLength, uint16_t *pnInput);
bool CsrSpiIsStopped(void);
uint16_t CsrSpiBcOperation(uint16_t nOperation);
bool CsrSpiBcCmd(uint16_t nLength, uint16_t *pnData);

#endif /* _CSRSPI_H */
