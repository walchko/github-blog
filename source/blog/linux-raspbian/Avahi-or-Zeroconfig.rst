Avahi
============================

:date: 2016-06-03
:summary: Getting linux to talk zeroconfig and play nice with macOS.

Making available to OSX
-----------------------

**Note:** Apple is finally starting to get serious about getting rid of AFP
and recommends going forward to use SMB (e.g., Samba) for file sharing. So you
can use avahi to zeroconfig, but don't use netatalk to do file sharing.

Avahi
~~~~~~~

.. figure:: pics/avahi.png
	:width: 200px
	:align: center

::

	sudo apt-get install netatalk
	sudo apt-get install avahi-utils

Make the following changes to the config file, adding an external drive to
netatalk, if you have one.

::

    pi@calculon ~ $ sudo pico /etc/netatalk/AppleVolumes.default
    # The line below sets some DEFAULT, starting with Netatalk 2.1.
    :DEFAULT: options:upriv,usedots

    # By default all users have access to their home directories.
    ~/              "Calculon"
    /mnt/usbdrive   "Calculon USB HD"

    # End of File

There are also options to enable Time Machine support (see tm).

Then restart ``netatalk``::

    pi@calculon ~ $ sudo /etc/init.d/netatalk stop
    Stopping Netatalk Daemons: afpd cnid_metad papd timelord atalkd.
    pi@calculon ~ $ sudo /etc/init.d/netatalk start
    Starting Netatalk services (this will take a while):  cnid_metad afpd.

::

	$ avahi-browse -arp
