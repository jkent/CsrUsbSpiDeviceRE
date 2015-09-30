#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Board/LEDs.h>
//#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Platform/Platform.h>

#include "Descriptors.h"
#include "csrspi.h"

#define MODE_SPI 0
#define MODE_JTAG 0xFFFF

uint16_t g_nMode = MODE_SPI;

enum {
    CMD_READ       = 0x0100,
    CMD_WRITE      = 0x0200,
    CMD_SETSPEED   = 0x0300,
    CMD_GETSTOPPED = 0x0400,
    CMD_GETSPEED   = 0x0500,
    CMD_UPDATE     = 0x0600,
    CMD_GETSERIAL  = 0x0700,
    CMD_GETVERSION = 0x0800,
    CMD_SETMODE    = 0x0900,
    CMD_SETBITS    = 0x0F00,
    CMD_BCCMDINIT  = 0x4000,
    CMD_BCCMD      = 0x4100,
};

static void SetupHardware(void);

static bool nReadLastPacket = false;
static uint8_t nReadLen = 0;
static bool nWritten = 0;
static uint8_t nWriteCount = 0;
static uint8_t USBRead8(void);
uint16_t USBRead16(void);
void USBWrite16(uint16_t data);

static void CmdRead(uint16_t nAddress, uint16_t nLength);
static void CmdWrite(uint16_t nAddress, uint16_t nLength);
static void CmdSetSpeed(uint16_t nSpeed);
static void CmdGetStopped(void);
static void CmdGetSpeed(void);
static void CmdUpdate(void);
static void CmdGetSerial(void);
static void CmdGetVersion(void);
static void CmdSetMode(uint16_t nMode);
static void CmdSetBits(uint16_t nWhich, uint8_t nValue);
static void CmdBcCmdInit(uint16_t nA, uint8_t nB);
static void CmdBcCmd(uint16_t nLength);


int main(void)
{
    uint16_t nCommand;
    uint16_t nArgs[2];

	SetupHardware();
	GlobalInterruptEnable();

	Endpoint_SelectEndpoint(VENDOR_OUT_EPADDR);
    for (;;) {
        while (!Endpoint_IsOUTReceived()) {
            USB_USBTask();
            Endpoint_SelectEndpoint(VENDOR_OUT_EPADDR);
        }

        USBRead8(); // discard byte

        nCommand = USBRead16();

        switch (nCommand) {
        case CMD_READ:
            nArgs[0] = USBRead16();
            nArgs[1] = USBRead16();
            CmdRead(nArgs[0], nArgs[1]);
            break;

        case CMD_WRITE:
            nArgs[0] = USBRead16();
            nArgs[1] = USBRead16();
            CmdWrite(nArgs[0], nArgs[1]);
            break;

        case CMD_SETSPEED:
            nArgs[0] = USBRead16();
            CmdSetSpeed(nArgs[0]);
            break;

        case CMD_GETSTOPPED:
            CmdGetStopped();
            break;

        case CMD_GETSPEED:
            CmdGetSpeed();
            break;

        case CMD_UPDATE:
            CmdUpdate();
            break;

        case CMD_GETSERIAL:
            CmdGetSerial();
            break;

        case CMD_GETVERSION:
            CmdGetVersion();
            break;

        case CMD_SETMODE:
            nArgs[0] = USBRead16();
            CmdSetMode(nArgs[0]);
            break;

        case CMD_SETBITS:
            nArgs[0] = USBRead16();
            nArgs[1] = USBRead16();
            CmdSetBits(nArgs[0], nArgs[1]);
            break;

        case CMD_BCCMDINIT:
            nArgs[0] = USBRead16();
            nArgs[1] = USBRead16();
            CmdBcCmdInit(nArgs[0], nArgs[1]);
            break;

        case CMD_BCCMD:
            nArgs[0] = USBRead16();
            CmdBcCmd(nArgs[0]);
            break;

        default:
            break;
        }

        if (nWritten) {
            Endpoint_SelectEndpoint(VENDOR_IN_EPADDR);
            Endpoint_ClearIN();
            nWriteCount = 0;
            nWritten = false;
        }

        if (nReadLen) {
            Endpoint_SelectEndpoint(VENDOR_OUT_EPADDR);
            USBRead8(); // discard byte
        }
    }
}

static void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
    //Serial_Init(115200, false);
    CsrInit();
}

void EVENT_USB_Device_ConfigurationChanged(void)
{
	/* Setup Endpoints */
	Endpoint_ConfigureEndpoint(VENDOR_IN_EPADDR,  EP_TYPE_BULK, VENDOR_IO_EPSIZE, 2);
	Endpoint_ConfigureEndpoint(VENDOR_OUT_EPADDR, EP_TYPE_BULK, VENDOR_IO_EPSIZE, 2);
}

static uint8_t USBRead8(void)
{
    uint8_t data;

    while (nReadLen == 0) {
        Endpoint_WaitUntilReady();
        nReadLen = Endpoint_BytesInEndpoint();
        if (nReadLen == 0) {
            Endpoint_ClearOUT();
        }
        nReadLastPacket = (nReadLen < VENDOR_IO_EPSIZE);
    }

    data = Endpoint_Read_8();
    nReadLen--;

    if (nReadLen == 0) {
        Endpoint_ClearOUT();
        if (!nReadLastPacket) {
            Endpoint_WaitUntilReady();
            nReadLen = Endpoint_BytesInEndpoint();
            if (nReadLen == 0) {
                Endpoint_ClearOUT();
            }
            nReadLastPacket = (nReadLen < VENDOR_IO_EPSIZE);
        }
    }

    return data;
}

uint16_t USBRead16(void)
{
    uint16_t data;

    if (nReadLen >= 3) {
        data = Endpoint_Read_16_BE();
        nReadLen -= 2;
    }
    else {
        data = (USBRead8() << 8) | USBRead8();
    }

    return data;
}


void USBWrite16(uint16_t data)
{
    if (nWriteCount == 0) {
        Endpoint_WaitUntilReady();
        nWritten = true;
    }

    Endpoint_Write_16_BE(data);
    nWriteCount += 2;

    if (nWriteCount == VENDOR_IO_EPSIZE) {
        Endpoint_ClearIN();
        nWriteCount = 0;
    }
}

static void USBWrite32(uint32_t data)
{
    USBWrite16(data >> 16);
    USBWrite16(data & 0xffff);
}

/* Command handlers */

uint16_t pCsrBuffer[1024];

static void CmdRead(uint16_t nAddress, uint16_t nLength)
{
    uint16_t *pCurrent;

    LEDs_TurnOnLEDs(LEDS_LED1);
    Endpoint_SelectEndpoint(VENDOR_IN_EPADDR);
    if (g_nMode == MODE_SPI && nLength <= 1024 &&
        CsrSpiRead(nAddress, nLength, pCsrBuffer)) {
        LEDs_TurnOffLEDs(LEDS_LED1);
        USBWrite16(CMD_READ);
        USBWrite16(nAddress);
        USBWrite16(nLength);
        pCurrent = pCsrBuffer;
        while (nLength--) {
            USBWrite16(*(pCurrent++));
        }
    }
    else {
        LEDs_TurnOffLEDs(LEDS_LED1);
        USBWrite16(CMD_READ + 1);
        USBWrite16(nAddress);
        USBWrite16(nLength);
        while (nLength--) {
            USBWrite16(0);
        }
    }
}

static void CmdWrite(uint16_t nAddress, uint16_t nLength)
{
    if (nLength > 1024 || g_nMode != MODE_SPI) {
        while (nLength--) {
            USBRead16();
        }
        return;
    }

    for (uint16_t i = 0; i < nLength; i++) {
        pCsrBuffer[i] = USBRead16();
    }

    LEDs_TurnOnLEDs(LEDS_LED2);
    CsrSpiWrite(nAddress, nLength, pCsrBuffer);
    LEDs_TurnOffLEDs(LEDS_LED2);
}

static void CmdSetSpeed(uint16_t nSpeed)
{
    g_nSpeed = nSpeed;
}

static void CmdGetStopped()
{
    Endpoint_SelectEndpoint(VENDOR_IN_EPADDR);
    USBWrite16(CMD_GETSTOPPED);
    USBWrite16(g_nMode != MODE_SPI || CsrSpiIsStopped());
}

static void CmdGetSpeed()
{
    Endpoint_SelectEndpoint(VENDOR_IN_EPADDR);
    USBWrite16(CMD_GETSPEED);
    USBWrite16(g_nSpeed);
}

static void CmdUpdate()
{
}

static void CmdGetSerial()
{
    Endpoint_SelectEndpoint(VENDOR_IN_EPADDR);
    USBWrite16(CMD_GETSERIAL);
    USBWrite32(31337);
}

static void CmdGetVersion()
{
    Endpoint_SelectEndpoint(VENDOR_IN_EPADDR);
    USBWrite16(CMD_GETVERSION);
    USBWrite16(0x119);
}

static void CmdSetMode(uint16_t nMode)
{
    g_nMode = nMode;
}

static void CmdSetBits(uint16_t nWhich, uint8_t nValue)
{
	if (nWhich) {
        g_nWriteBits = nValue;
    }
	else {
        g_nReadBits = nValue;
    }
}

static void CmdBcCmdInit(uint16_t nA, uint8_t nB)
{
    g_nBcA = nA;
    g_nBcB = nB;
}

static void CmdBcCmd(uint16_t nLength)
{
    for (uint16_t i = 0; i < nLength; i++) {
        pCsrBuffer[i] = USBRead16();
    }

    Endpoint_SelectEndpoint(VENDOR_IN_EPADDR);
    LEDs_TurnOnLEDs(LEDS_LED1 | LEDS_LED2);
    if (CsrSpiBcCmd(nLength, pCsrBuffer)) {
        USBWrite16(CMD_BCCMD);
    }
    else {
        USBWrite16(CMD_BCCMD + 1);
    }
    LEDs_TurnOffLEDs(LEDS_LED1 | LEDS_LED2);
    USBWrite16(nLength);
    for (uint16_t i = 0; i < nLength; i++) {
        USBWrite16(pCsrBuffer[i]);
    }
}
