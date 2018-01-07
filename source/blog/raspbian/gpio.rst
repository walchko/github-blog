raspi-gpio
============

:date: 2017-02-05
:summary: Setting GPIO Pins and serial

First install the software

::

	sudo apt-get update
	sudo apt-get install raspi-gpio

Then you can set or get the RPi's pin info::

	pi@zoidberg ~ $ sudo raspi-gpio get
	BANK0 (GPIO 0 to 27):
	  GPIO 00: level=1 fsel=0 alt=  func=INPUT
	  GPIO 01: level=1 fsel=0 alt=  func=INPUT
	  GPIO 02: level=1 fsel=4 alt=0 func=SDA1
	  GPIO 03: level=1 fsel=4 alt=0 func=SCL1
	  GPIO 04: level=1 fsel=0 alt=  func=INPUT
	  GPIO 05: level=1 fsel=0 alt=  func=INPUT
	  GPIO 06: level=1 fsel=0 alt=  func=INPUT
	  GPIO 07: level=1 fsel=0 alt=  func=INPUT
	  GPIO 08: level=1 fsel=0 alt=  func=INPUT
	  GPIO 09: level=0 fsel=0 alt=  func=INPUT
	  GPIO 10: level=0 fsel=0 alt=  func=INPUT
	  GPIO 11: level=0 fsel=0 alt=  func=INPUT
	  GPIO 12: level=0 fsel=0 alt=  func=INPUT
	  GPIO 13: level=0 fsel=0 alt=  func=INPUT
	  GPIO 14: level=1 fsel=2 alt=5 func=TXD1
	  GPIO 15: level=1 fsel=2 alt=5 func=RXD1
	  GPIO 16: level=0 fsel=0 alt=  func=INPUT
	  GPIO 17: level=0 fsel=0 alt=  func=INPUT
	  GPIO 18: level=1 fsel=4 alt=0 func=PCM_CLK
	  GPIO 19: level=1 fsel=4 alt=0 func=PCM_FS
	  GPIO 20: level=0 fsel=4 alt=0 func=PCM_DIN
	  GPIO 21: level=0 fsel=4 alt=0 func=PCM_DOUT
	  GPIO 22: level=0 fsel=0 alt=  func=INPUT
	  GPIO 23: level=0 fsel=0 alt=  func=INPUT
	  GPIO 24: level=0 fsel=0 alt=  func=INPUT
	  GPIO 25: level=0 fsel=0 alt=  func=INPUT
	  GPIO 26: level=0 fsel=0 alt=  func=INPUT
	  GPIO 27: level=0 fsel=0 alt=  func=INPUT
	BANK1 (GPIO 28 to 45):
	  GPIO 28: level=0 fsel=0 alt=  func=INPUT
	  GPIO 29: level=1 fsel=0 alt=  func=INPUT
	  GPIO 30: level=0 fsel=0 alt=  func=INPUT
	  GPIO 31: level=0 fsel=0 alt=  func=INPUT
	  GPIO 32: level=1 fsel=7 alt=3 func=TXD0
	  GPIO 33: level=1 fsel=7 alt=3 func=RXD0
	  GPIO 34: level=1 fsel=7 alt=3 func=SD1_CLK
	  GPIO 35: level=1 fsel=7 alt=3 func=SD1_CMD
	  GPIO 36: level=1 fsel=7 alt=3 func=SD1_DAT0
	  GPIO 37: level=1 fsel=7 alt=3 func=SD1_DAT1
	  GPIO 38: level=1 fsel=7 alt=3 func=SD1_DAT2
	  GPIO 39: level=1 fsel=7 alt=3 func=SD1_DAT3
	  GPIO 40: level=0 fsel=4 alt=0 func=PWM0
	  GPIO 41: level=0 fsel=4 alt=0 func=PWM1
	  GPIO 42: level=0 fsel=4 alt=0 func=GPCLK1
	  GPIO 43: level=1 fsel=4 alt=0 func=GPCLK2
	  GPIO 44: level=1 fsel=0 alt=  func=INPUT
	  GPIO 45: level=1 fsel=0 alt=  func=INPUT
	BANK2 (GPIO 46 to 53):
	  GPIO 46: level=1 fsel=4 alt=0 func=SDA0
	  GPIO 47: level=1 fsel=4 alt=0 func=SCL0
	  GPIO 48: level=0 fsel=4 alt=0 func=SD0_CLK
	  GPIO 49: level=1 fsel=4 alt=0 func=SD0_CMD
	  GPIO 50: level=1 fsel=4 alt=0 func=SD0_DAT0
	  GPIO 51: level=1 fsel=4 alt=0 func=SD0_DAT1
	  GPIO 52: level=1 fsel=4 alt=0 func=SD0_DAT2
	  GPIO 53: level=1 fsel=4 alt=0 func=SD0_DAT3

This shows us that BCM pins 14/15 are tied to ``serial1`` while BCM pins 32/33
are tied to ``serial0``.

The full help is::

	pi@zoidberg ~ $ sudo raspi-gpio help

	WARNING! raspi-gpio set writes directly to the GPIO control registers
	ignoring whatever else may be using them (such as Linux drivers) -
	it is designed as a debug tool, only use it if you know what you
	are doing and at your own risk!

	The raspi-gpio tool is designed to help hack / debug BCM283x GPIO.
	Running raspi-gpio with the help argument prints this help.
	raspi-gpio can get and print the state of a GPIO (or all GPIOs)
	and can be used to set the function, pulls and value of a GPIO.
	raspi-gpio must be run as root.
	Use:
	  raspi-gpio get [GPIO]
	OR
	  raspi-gpio set <GPIO> [options]
	OR
	  raspi-gpio funcs [GPIO]
	OR
	  raspi-gpio raw
	Note that omitting [GPIO] from raspi-gpio get prints all GPIOs.
	raspi-gpio funcs will dump all the possible GPIO alt funcions in CSV format
	or if [GPIO] is specified the alternate funcs just for that specific GPIO.
	Valid [options] for raspi-gpio set are:
	  ip      set GPIO as input
	  op      set GPIO as output
	  a0-a5   set GPIO to alternate function alt0-alt5
	  pu      set GPIO in-pad pull up
	  pd      set GPIO pin-pad pull down
	  pn      set GPIO pull none (no pull)
	  dh      set GPIO to drive to high (1) level (only valid if set to be an output)
	  dl      set GPIO to drive low (0) level (only valid if set to be an output)
	Examples:
	  raspi-gpio get              Prints state of all GPIOs one per line
	  raspi-gpio get 20           Prints state of GPIO20
	  raspi-gpio set 20 a5        Set GPIO20 to ALT5 function (GPCLK0)
	  raspi-gpio set 20 pu        Enable GPIO20 ~50k in-pad pull up
	  raspi-gpio set 20 pd        Enable GPIO20 ~50k in-pad pull down
	  raspi-gpio set 20 op        Set GPIO20 to be an output
	  raspi-gpio set 20 dl        Set GPIO20 to output low/zero (must already be set as an output)
	  raspi-gpio set 20 ip pd     Set GPIO20 to input with pull down
	  raspi-gpio set 35 a0 pu     Set GPIO35 to ALT0 function (SPI_CE1_N) with pull up
	  raspi-gpio set 20 op pn dh  Set GPIO20 to ouput with no pull and driving high

Serial Setup
---------------

For Raspberry Pi 3’s the command is similar but referencing ``/dev/ttyS0``::

	$ sudo systemctl stop serial-getty@ttyS0.service
	$ sudo systemctl disable serial-getty@ttyS0.service

You also need to remove the console from the ``cmdline.txt``. If you edit this with::

	$ sudo nano /boot/cmdline.txt

you will see something like::

	dwc_otg.lpm_enable=0 console=serial0,115200 console=tty1 root=/dev/mmcblk0p2
	rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait

remove the line: console=serial0,115200 and save and reboot for changes to take
effect.

When you enable uart_1 in ``/boot/config.txt``::

	pi@zoidberg ~ $ ls -al /dev/serial1
	lrwxrwxrwx 1 root root 7 Feb  5 19:18 /dev/serial1 -> ttyAMA0
	pi@zoidberg ~ $ ls -al /dev/serial0
	lrwxrwxrwx 1 root root 5 Feb  5 19:18 /dev/serial0 -> ttyS0

Thus if I want to set an alternate pin configuration I could do::

	sudo raspi-gpio set 17 a5

This should set BCM 17 to alternate 5 configuration, which make it RTS for
``serial0``. Now you can use flow control from python:

.. code-block:: python

	import serial
	ser = serial.Serial(port='/dev/serial0', baudrate=9600, rtscts=True)
	ser.setRTS(False)
	ser.setRTS(True)

Overlay
---------

::

	dtoverlay=pi3-miniuart-bt

References
-------------

- `RPi pins <http://pinout.xyz/pinout/pin11_gpio17>`_
