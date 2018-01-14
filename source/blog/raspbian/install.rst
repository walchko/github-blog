Install Raspberry Pi (RPi)
============================

:date: 2015-01-02
:summary: Installing Rasbian
:modified: 2016-11-27

.. figure:: pics/rpi-org.png
	:width: 200px
	:align: center


Where to buy?
-------------

I always buy mine from `Adafruit <https://www.adafruit.com>`__, they
have tons of other great stuff at great prices. They also make make lots
of example code and drivers available for their products.

Install
--------

`Raspbian <http://www.raspbian.org>`__ is a Raspberry optimized version
of Debian. The version installed here is based on Debian Jessie.

::

    [kevin@raspberrypi ~]$ lscpu
    Architecture:          armv6l
    Byte Order:            Little Endian
    CPU(s):                1
    On-line CPU(s) list:   0
    Thread(s) per core:    1
    Core(s) per socket:    1
    Socket(s):             1

Note this output doesn't really tell you much other than it is ARMv6.

Copying an image to the SD Card in Mac OS X
-------------------------------------------

.. figure:: pics/sd.jpg
   :width: 200px
   :alt: sd logo

These commands and actions need to be performed from an account that has
administrator privileges.

1. Download the image from a `mirror or
   torrent <http://www.raspberrypi.org/downloads>`__.

2. Verify if the the hash key is the same (optional), in the terminal
   run::

       shasum ~/Downloads/debian6-19-04-2012.zip

3. Extract the image::

       unzip ~/Downloads/debian6-19-04-2012.zip

4. Attach the SD Card to the computer and identify the mount point::

       df -h

   Record the device name of the filesystem's partition, e.g.
   ``/dev/disk3s1``

5. Unmount the partition so that you will be allowed to overwrite the
   disk, note that unmount is **NOT** the same as eject:

   ::

       sudo diskutil unmount /dev/disk3s1

6. Write the image to the card with this command:

   ::

       sudo dd bs=1m if=rasbian.img of=/dev/rdisk3

7. After the dd command finishes, eject the card:

   ::

       sudo diskutil eject /dev/disk3

8. Insert it in the raspberry pi, and have fun

Configuration
--------------

Once you download and install Raspbian you have to configure it for it to be useful.

#. ``sudo raspi-config`` and change
    #. update ``raspi-config`` via the advanced option, update
    #. hostname
    #. memory split between GPU and RAM
	#. set local to en_US.UTF-8 UTF-8 (the default is en-GB)
    #. resize the file system to the size of your disk
    #. set correct timezone via the internationalization option
    #. turn on I2C interface
#. ``sudo apt-get update`` and then ``sudo apt-get upgrade``
#. ``sudo apt-get install apt-show-versions``
#. ``wget https://bootstrap.pypa.io/get-pip.py`` and then ``python get-pip.py``
#. ``sudo apt-get install rpi-update`` and then ``sudo rpi-update`` to update the kernel
#. Fix the pip paths so you don't have to use sudo (that is a security risk)
    #. ``sudo chown -R pi /usr/local``
    #. ``sudo chown -R pi /usr/lib/python2.7/dist-packages``
#. Fix the ``pip`` compile issues ``sudo apt-get install python-dev``
#. Find outdated python libraries with ``pip list --outdated`` then update them with ``pip install -U package_name``

Useful Software
-----------------

Add the following software with::

	sudo apt-get install <package> <package> ...

Some useful packages are:

* cmake
* build-essential
* python-dev
* nmap
* arp-scan
* htop
* git


Add the following software with::

	pip install <package> <package> ...

* pyarchey
* numpy

Headless
----------

Raspbian is now posting images for a *Lite* version of Raspbian, I suggest you
use that if you are doing headless.

SSH Login
---------

To increase security, you can disable password logins and rely on ssh
public keys. To do this, take a look
`here <https://wiki.archlinux.org/index.php/SSH_Keys>`__ for details.
Basic steps are:

1. Generate an ssh key::

       ssh-keygen

2. Copy the public key (.pub) to the server you will connect to::

       ssh-copy-id username@remote-server.org

3. Edit /etc/ssh/sshd\_config to disable password logins::

       PasswordAuthentication no
       ChallengeResponseAuthentication no

Bypass known_hosts
---------------------

Since all RPi's hostname are raspberrypi.local, it **sucks** when you try to connect
to a new one and you get the man-in-the-middle attack warning.

You can disable the check with::

	ssh -o UserKnownHostsFile=/dev/null pi@raspberrypi.local

OSX
~~~~

On OSX install ``ssh-copy-id`` via ``brew`` and in a terminal window on OSX::

    ssh-copy-id pi@raspberry.local
