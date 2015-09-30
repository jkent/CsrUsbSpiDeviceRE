CsrUsbSpiDevice-ProMicro
===
Description
---
This is a reverse engineered re-implementation of CSR's USB<->SPI Converter on the Sparkfun Pro Micro. It is compatible with CSR's own drivers and BlueSuite tools, and should work on any BlueCore chip that supports programming through SPI.

Based on origional work by Frans-Willem for the Stellaris Launchpad - https://github.com/Frans-Willem/CsrUsbSpiDeviceRE

Disclaimer
---
I make no guarantees about this code. For me it worked, but it might not for you. If you break your BlueCore module or anything else by using this software this is your own responsibility.

How to use
---
* Get a Sparkfun Pro Micro or Chinese clone.

* Connect your BlueCore module. 3.3v -> 3.3v, GND -> GND, 10 -> SPI_CSB, 16 -> SPI_MOSI, 14 -> SPI_MISO, 15 -> SPI_CLK

* Install avr-gcc and avr-libc if you have not already

* Clone this repository, and run `git submodule init` and `git submodule update`

* Make sure your Pro Micro is in programming mode

* run `make avrdude`

FINALLY 
* Plug in "device" microUSB port to a host computer with CSR BlueSuite (or Bluelab) installed.
* Device should be recognized, Drivers can be found at csrsupport.com (needs registration), underBrowse category tree -> Bluetooth PC Software/Tools -> USB-SPI Converter.
* Use any of the BlueSuite or BlueLabs tools to play with your bluecore module! (BlueSuite can be found on CSRSupport.com under Browser category tree -> Bluetooth PC Software/Tools -> Current BlueSuite Development Tools)

Notes
---
* The SPI clock is not accurate
* If something does or does not work, let me know (jkent)

Original code - and most of the hard work - by Frans-Willem Dec 2012
