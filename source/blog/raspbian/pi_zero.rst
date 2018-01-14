Install Raspberry Pi (RPi) Zero
======================================

:date: 2017-04-30
:summary: Installing Rasbian on Pi Zero W

.. figure:: pics/rpi-org.png
	:width: 200px
	:align: center

You need to make a couple changes before you put the SD card into the Pi.

In ``config.txt``, add the following to the bottom::

	dtoverlay=dwc2

In ``cmdline.txt``, add this right after ``rootwait``::

	modules-load=dwc2,g_ether

Now put the SD card in the Pi and plug the Pi into your computer with a USB cable.

To give it access to the internet:

* System Preferences
	* Sharing
		* Share your connection from: WiFi (or Ethernet if you have a wired connection)
		* To computers using: RNDIS/Ethernet Gadget
		* Then check/select ``Internet Sharing`` in the service box

**Note:** If you already plugged in your Pi to your computer, you will need to
reboot the Pi using::

	sudo reboot

This process sets up a dhcp server for the ``RNDIS/Ethernet Gadget`` and assigns
it an IP address, then allows it to talk to the internet using WiFi.

A good resource is `here <http://www.circuitbasics.com/raspberry-pi-zero-ethernet-gadget/>`__

SSH
-----

To enable ``ssh``, go to the ``/boot`` directory and create a file called *ssh*:

    touch ssh

This will tell raspbian to enable it on boot.

Bypass known_hosts
---------------------

Since all RPi's hostname are raspberrypi.local, it **sucks** when you try to connect
to a new one and you get the man-in-the-middle attack warning.

You can disable the check with::

	ssh -o UserKnownHostsFile=/dev/null pi@raspberrypi.local
