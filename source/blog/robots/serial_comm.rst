Serial Communication
========================

:date: 2016-08-22
:modified: 2017-08-13
:summary: Simple serial info

If you need to quickly use a serial port, use ``pyserial``::

	python -m serial.tools.miniterm -e /dev/tty.usbserial-A5004Flb 9600

This opens up a simple little terminal. The ``-e`` echos what you type. The
above serial port is on macOS, but RPi typically has ``/dev/ttyACM0`` or
``/dev/ttyS0`` for the serial port.

Python
~~~~~~~~~

A simple python script is:

.. code-block:: python

	import serial

	# s = serial.Serial('/dev/ttyS0', 19200, timeout=0.1)
	s = serial.Serial()
	s.baudrate = 19200
	s.port = '/dev/serial0'
	s.timeout = 0.1
	s.open()
	s.write('hello')

	msg = s.read(100)
	print(msg)

	if s.is_open:
		s.close()

- `pyserial docs <http://pyserial.readthedocs.io>`_

Setup (old)
--------------

Depending, if you don't have permission to open the serial port and are forced
to use ``sudo``, add your user to the ``tty`` group::

	sudo usermod -a -G tty pi

RPi3
------

Basically the serial situation is all fucked up and no one really knows what is
going on because of the terrible documentation. The setup is::

	pi@zoidberg ~ $ ls -al /dev/serial*
	lrwxrwxrwx 1 root root 5 Feb 13 20:57 /dev/serial0 -> ttyS0
	lrwxrwxrwx 1 root root 7 Feb 13 20:57 /dev/serial1 -> ttyAMA0

where:

================ ================ ========= =============================
Port             Alias            uart      Use
================ ================ ========= =============================
``/dev/ttyAMA0`` ``/dev/serial1`` ``uart0`` Bluetooth, independent of core clock rate
``/dev/ttyS0``   ``/dev/serial0`` ``uart1`` GPIO, baud rate is dependent on core clock rate
================ ================ ========= =============================

Fix Things
~~~~~~~~~~~~

** this doesn't completely work **

1. Edit ``/boot/config.txt`` and add:
	- ``enable_uart=1``
	- ``dtoverlay=pi3-disable-bt``
	- ``dtoverlay=pi3-miniuart-bt-overlay``
	- ``force_turbo=1``
2. Edit ``/boot/cmdline.txt`` and remove console commands tied to serial port::

		dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait

3. Turn off any serial services::
		$ sudo systemctl stop serial-getty@ttyS0.service
		$ sudo systemctl disable serial-getty@ttyS0.service

Now I have::

	pi@zoidberg bin $ ls -al /dev/serial*
	lrwxrwxrwx 1 root root 7 Feb 17 16:47 /dev/serial0 -> ttyAMA0
	lrwxrwxrwx 1 root root 5 Feb 17 16:47 /dev/serial1 -> ttyS0

See how the serial terminals have changed.


- `RASPBERRY PI 3 UART BAUD RATE WORKAROUND <https://frillip.com/raspberry-pi-3-uart-baud-rate-workaround/>`_
- `Stack Exchange <http://raspberrypi.stackexchange.com/questions/45570/how-do-i-make-serial-work-on-the-raspberry-pi3>`_
- `spellfoundry.com <http://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3/>`_
