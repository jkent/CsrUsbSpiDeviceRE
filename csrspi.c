#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>

#include "csrspi.h"

uint16_t USBRead16(void);
void USBWrite16(uint16_t data);

#define PIN_CS PB6
#define PIN_MISO PB3
#define PIN_MOSI PB2
#define PIN_CLK PB1

uint16_t g_nSpeed = 393;
uint16_t g_nReadBits = 0;
uint16_t g_nWriteBits = 0;
uint16_t g_nBcA = 0;
uint16_t g_nBcB = 0;
uint16_t g_nUseSpecialRead = 0;

void CsrInit(void)
{
    DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB6);
    DDRB &= ~(1 << DDB3);
    PORTB |= (1 << PIN_MISO);
    PORTB &= ~((1 << PIN_CLK) | (1 << PIN_MOSI) | (1 << PIN_CS));
}

static void CsrSpiDelay(void)
{
    if (g_nSpeed <= 4) {
        return;
    }

    uint32_t ns = 126 * g_nSpeed + 434;
    while (ns >= 10000) {
        _delay_us(10);
        ns -= 10000;
    }
}

static void CsrSpiStart(void)
{
    PORTB = (PORTB & ~((1 << PIN_MOSI) | (1 << PIN_CLK))) | (1 << PIN_CS);
    //CsrSpiDelay();

    PORTB |= (1 << PIN_CLK);
    //CsrSpiDelay();

    PORTB &= ~(1 << PIN_CLK);
    //CsrSpiDelay();

    PORTB |= (1 << PIN_CLK);
    //CsrSpiDelay();

    PORTB &= ~(1 << PIN_CLK);
    //CsrSpiDelay();

    PORTB &= ~(1 << PIN_CS);
    //CsrSpiDelay();
}

static void CsrSpiStop(void)
{
    PORTB = (PORTB & ~((1 << PIN_MOSI) | (1 << PIN_CLK))) | (1 << PIN_CS);
}

/*static void CsrSpiSendBits(uint16_t nData, uint8_t nBits)
{
    nData <<= 16 - nBits; 
    while (nBits--) {
        PORTB &= ~((1 << PIN_MOSI) | (1 << PIN_CLK) | (1 << PIN_CS));
        //CsrSpiDelay();

        if (nData & 0x8000) {
            PORTB |= (1 << PIN_MOSI) | (1 << PIN_CLK);
        }
        else {
            PORTB |= (1 << PIN_CLK);
        }
        CsrSpiDelay();

        nData <<= 1;
    }
}*/

static void CsrSpiSendByte(uint8_t nData)
{
    uint8_t nBits = 8;

    if (g_nSpeed <= 4) {
        while (nBits--) {
            PORTB &= ~((1 << PIN_MOSI) | (1 << PIN_CLK) | (1 << PIN_CS));
            if (nData & 0x80) {
                PORTB |= (1 << PIN_MOSI) | (1 << PIN_CLK);
            }
            else {
                PORTB |= (1 << PIN_CLK);
            }
            nData <<= 1;
        }
        return;
    }

    while (nBits--) {
        PORTB &= ~((1 << PIN_MOSI) | (1 << PIN_CLK) | (1 << PIN_CS));
        //CsrSpiDelay();
        if (nData & 0x80) {
            PORTB |= (1 << PIN_MOSI) | (1 << PIN_CLK);
        }
        else {
            PORTB |= (1 << PIN_CLK);
        }
        CsrSpiDelay();
        nData <<= 1;
    }
}

static void CsrSpiSendWord(uint16_t nData)
{
    CsrSpiSendByte(nData >> 8);
    CsrSpiSendByte(nData);
}

#if 0
static uint16_t CsrSpiReadBits(uint8_t nBits)
{
    uint16_t nData = 0;

    while (nBits--) {
        PORTB &= ~((1 << PIN_MOSI) | (1 << PIN_CLK) | (1 << PIN_CS));
        //CsrSpiDelay();

        nData <<= 1;

        PORTB |= (1 << PIN_CLK);
        CsrSpiDelay();

        if (PINB & (1 << PIN_MISO)) {
            nData |= 1;
        }
    }

    return nData;
}
#endif

static uint16_t CsrSpiReadByte(void)
{
    uint8_t nData = 0;
    uint8_t nBits = 8;

    if (g_nSpeed <= 4) {
        while (nBits--) {
            PORTB &= ~((1 << PIN_MOSI) | (1 << PIN_CLK) | (1 << PIN_CS));
            nData <<= 1;
            PORTB |= (1 << PIN_CLK);
            if (PINB & (1 << PIN_MISO)) {
                nData |= 1;
            }
        }

        return nData;
    }

    while (nBits--) {
        PORTB &= ~((1 << PIN_MOSI) | (1 << PIN_CLK) | (1 << PIN_CS));
        //CsrSpiDelay();
        nData <<= 1;
        PORTB |= (1 << PIN_CLK);
        CsrSpiDelay();
        if (PINB & (1 << PIN_MISO)) {
            nData |= 1;
        }
    }

    return nData;
}

static uint16_t CsrSpiReadWord(void)
{
    uint16_t nData = CsrSpiReadByte() << 8;
    nData |= CsrSpiReadByte();
    return nData;
}

static bool CsrSpiReadBasic(uint16_t nAddress, uint16_t nLength, uint16_t *pnOutput)
{
    //nLength--;

    CsrSpiStart();
    CsrSpiSendByte(g_nReadBits | 3);
    CsrSpiSendWord(nAddress);

    uint16_t nControl = CsrSpiReadWord();
    if (nControl != (((g_nReadBits | 3) << 8) | nAddress >> 8)) {
        //CsrSpiStart();
        CsrSpiStop();
        return false;
    }

    while (nLength--) {
        *(pnOutput++) = CsrSpiReadWord();
    }

    /*uint16_t nLast = CsrSpiReadBits(15);
    PORTB &= ~((1 << PIN_MOSI) | (1 << PIN_CLK) | (1 << PIN_CS));
    CsrSpiDelay(); CsrSpiDelay(); CsrSpiDelay(); CsrSpiDelay();
    nLast <<= 1;
    if (PINB & (1 << PIN_MISO)) {
        nLast |= 1;
    }
    *(pnOutput++) = nLast;*/
    //CsrSpiStart();
    CsrSpiStop();
    return true;
}

bool CsrSpiRead(uint16_t nAddress, uint16_t nLength, uint16_t *pnOutput)
{
    int nRet = true;

    if (!g_nUseSpecialRead) {
        return CsrSpiReadBasic(nAddress, nLength, pnOutput);
    }

    nLength--;
    nRet &= CsrSpiReadBasic(nAddress, nLength, pnOutput);
    g_nReadBits |= 0x20;
    nRet &= CsrSpiReadBasic(nAddress + nLength, 1, &pnOutput[nLength]);
    g_nReadBits &= ~0x20;
    return nRet;
}

void CsrSpiWrite(uint16_t nAddress, uint16_t nLength, uint16_t *pnInput)
{
    CsrSpiStart();

    CsrSpiSendByte(g_nWriteBits | 2);
    CsrSpiSendWord(nAddress);
    while (nLength--)
        CsrSpiSendWord(*(pnInput++));
    //CsrSpiStart();
    CsrSpiStop();
}

bool CsrSpiIsStopped(void)
{
    uint16_t nOldSpeed = g_nSpeed;
    g_nSpeed += 32;
    CsrSpiStart();
    g_nSpeed = nOldSpeed;
    uint8_t nRead = PINB;
    CsrSpiStop();
    if (nRead & (1 << PIN_MISO)) {
        return true;
    }
    else {
        return false;
    }
}

uint16_t CsrSpiBcOperation(uint16_t nOperation)
{
    uint16_t var0, var1;

    CsrSpiRead(g_nBcA, 1, &var0);
    CsrSpiWrite(g_nBcB, 1, &nOperation);
    CsrSpiWrite(g_nBcA, 1, &var0);
    for (uint8_t i = 0; i < 30; i++) {
        CsrSpiRead(g_nBcB, 1, &var1);
        if (nOperation != var1) {
            return var1;
        }
    }
    return var1;
}
    
bool CsrSpiBcCmd(uint16_t nLength, uint16_t *pnData)
{
    uint16_t var0, var1;

    if (CsrSpiBcOperation(0x7) != 0) {
        return false;
    }

    CsrSpiWrite(g_nBcB + 1, 1, &nLength);
    if (CsrSpiBcOperation(0x1) != 2) {
        return false;
    }

    if (!CsrSpiRead(g_nBcB + 2, 1, &var0)) {
        return false;
    }

    CsrSpiWrite(var0, nLength, pnData);
    CsrSpiBcOperation(0x4);
    for (uint8_t i = 0; i < 30; i++) {
        if (CsrSpiRead(g_nBcB, 1, &var1) && var1 == 0x6) {
            CsrSpiRead(var0, nLength, pnData);
            CsrSpiBcOperation(0x7);
            return true;
        }
    }

    return false;
}
