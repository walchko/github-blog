---
title: Arch Linux
date: 2015-12-13
abstract: How to install Arch Linux and setup
...

.. figure:: pics/arch_linux.png
   :width: 200px
   :align: center

I like arch linux better than raspbian is a lot of ways, except:

-  default is python 3
-  not as fully supported (tutorials, software packages, etc) on RPi as
   raspbian is

Basic Install Process
---------------------

Don't follow the install instructions, they suck. Instead, look at the
`Beginner's Guide <https://wiki.archlinux.org/index.php/Beginners%27_Guide>`__ on the wiki.

Virtualbox Guest
----------------

Follow these
`instructions <https://wiki.archlinux.org/index.php/Arch_Linux_VirtualBox_Guest#Arch_Linux_guests>`__
for the install.

1. Install this package:

   pacman -S virtualbox-guest-utils

2. Create the module file /etc/modules-load/virtualbox.conf

   vboxguest vboxsf vboxvideo

Post-Install Packages
---------------------

::

    pacman -S package

Useful packages:

-  distcc
-  sudo - edit /etc/sudoers, uncomment wheel group
-  avahi and nss-mdns
-  openssh
-  virtualbox-guest-utils

Update System
-------------

::

    pacman -Syu

Fonts
-----

Fontconfig configuration is done via ``/etc/fonts/conf.avail`` and ``conf.d``.
Read ``/etc/fonts/conf.d/README`` for more information.

Configuration via ``/etc/fonts/local.conf`` is still possible, but is no
longer recommended for options available in conf.avail.

Main system wide configuration should be done by symlinks (especially for
autohinting, sub-pixel and lcdfilter):

::

    cd /etc/fonts/conf.d
    ln -s ../conf.avail/XX-foo.conf

Check also https://wiki.archlinux.org/index.php/Font\_Configuration and
https://wiki.archlinux.org/index.php/Fonts.

Arch Networking
-----------------

Avahi - Multicast
--------------------------------------------

You can enable Avahi Daemon at startup with the following command:

::

    systemctl enable avahi-daemon.service

SSH
---

For the client edit /etc/ssh/ssh\_config remove protocol 1 since it is
deemed insecure and only use 2:

::

    Protocol 2

For the server edit /etc/ssh/sshd\_config enable:

::

    AllowUsers user1 user2 (change to appropriate user names)
    PermitRooLogin no
    Banner /etc/issue

Then add it to the DAEMONS list in /etc/rc.conf, so it starts on boot:

\*\* Don't use rc.conf anymore!! \*\*

::

    DAEMONS=( .... sshd ....)

You can also start it immediately by:

::

    sudo rc.d start sshd

To see if it worked, type:

::

    ps -e | grep sshd
