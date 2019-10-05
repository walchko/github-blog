

Bluetooth
=========

:date: 2015-06-30
:modified: 2017-01-06
:summary: Working with bluetooth on linux.

.. image:: pics/bluetooth.png
	:width: 100px
	:align: center

**Note:** Things changed with Raspbian Jessie, some of this might be out of
date.

**Note:** Also, RPi 3 comes with an integrated bluetooth module

RPI 3
------

Services::

	pi@zoidberg ~ $ service bluetooth status
	● bluetooth.service - Bluetooth service
	   Loaded: loaded (/lib/systemd/system/bluetooth.service; enabled)
	   Active: active (running) since Fri 2016-12-30 20:35:36 MST; 1 weeks 0 days ago
	     Docs: man:bluetoothd(8)
	 Main PID: 586 (bluetoothd)
	   Status: "Running"
	   CGroup: /system.slice/bluetooth.service
	           └─586 /usr/lib/bluetooth/bluetoothd

Packages installed::

	pi@zoidberg ~ $ dpkg-query -l '*blue*'
	Desired=Unknown/Install/Remove/Purge/Hold
	| Status=Not/Inst/Conf-files/Unpacked/halF-conf/Half-inst/trig-aWait/Trig-pend
	|/ Err?=(none)/Reinst-required (Status,Err: uppercase=bad)
	||/ Name           Version      Architecture Description
	+++-==============-============-============-=================================
	ii  bluez          5.23-2+rpi2  armhf        Bluetooth tools and daemons
	un  bluez-alsa     <none>       <none>       (no description available)
	un  bluez-audio    <none>       <none>       (no description available)
	ii  bluez-firmware 1.2-3+rpi1   all          Firmware for Bluetooth devices
	un  bluez-input    <none>       <none>       (no description available)
	un  bluez-network  <none>       <none>       (no description available)
	un  bluez-serial   <none>       <none>       (no description available)
	un  bluez-utils    <none>       <none>       (no description available)
	ii  pi-bluetooth   0.1.1        armhf        Raspberry Pi 3 bluetooth

Dongle (RPi 2)
-----------------

Make sure your bluetooth dongle is working::

    pi@calculon ~ $ lsusb
    Bus 001 Device 002: ID 0424:9512 Standard Microsystems Corp.
    Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    Bus 001 Device 003: ID 0424:ec00 Standard Microsystems Corp.
    Bus 001 Device 004: ID 0a12:0001 Cambridge Silicon Radio, Ltd Bluetooth Dongle (HCI mode)
    Bus 001 Device 005: ID 0bc2:3312 Seagate RSS LLC

Device 004 is my bluetooth dongle.

Audio
-----

References
`1 <http://blog.whatgeek.com.pt/2014/04/20/raspberry-pi-bluetooth-wireless-speaker/>`__
`2 <http://www.correderajorge.es/bluetooth-on-raspberry-audio-streaming/>`__

Get the bluetooth software::

    sudo apt-get install bluetooth bluez-utils bluez-alsa

Add ``pi`` to the user group:

::

    sudo gpasswd -a pi bluetooth

Start up bluetooth dongle:

::

    sudo hciconfig hci0 up

Scan for bluetooth devices

::

    pi@calculon ~ $ hcitool scan
    Scanning ...
        44:2A:60:D6:49:34   Dalek

If you need to figure out what your bluetooth hardware address is for ``hci0``::

    hcitool dev

Pair with device using ``bluetooth-agent --adapter hci0 <pin> <hardware_id>``::

    bluetooth-agent --adapter hci0 0000 00:11:67:8C:17:80

See if device is trusted or not::

    bluez-test-device trusted <hardware_id>

If it returns a ``0`` then it is not trusted and a ``1`` if it is
trusted. To make it trusted::

    bluez-test-device trusted <hardware_id> yes

Edit/create ``~/.asoundrc`` with::

    pcm.bluetooth {
                    type bluetooth
                    device <hardware_id>
    }

**Note:** You may have to copy this file and rename it to: ``/etc/asound.conf``

Edit ``/etc/bluetooth/audio.conf`` and add the following to the
``[General]`` section::

    Disable=Media
    Enable=Socket,Sink,Source

Restart bluetooth service with::

    sudo /etc/init.d/bluetooth restart

Audio Programs
--------------

::

    mplayer -ao alsa:device=bluetooth sound.mp3
    mpg321 -a bluetooth -g 15 sound.mp3
