
Linux Kernel
============

:date: 2014-10-26
:summary: Configuring and updating the linux kernal for the RPi

.. figure:: {filename}/blog/raspbian/pics/linux.jpg
   :width: 200px
   :align: center

Determine Kernel version and upgrade
------------------------------------

You can determine the current linux kernel version by::

    [kevin@raspberrypi tmp]$ more /proc/version
    Linux version 3.2.27+ (dc4@dc4-arm-01) (gcc version 4.7.2 20120731 (prerelease)
    (crosstool-NG linaro-1.13.1+bzr2458 - Linaro GCC 2012.08) ) #250 PREEMPT Thu Oct
     18 19:03:02 BST 2012

or

::

    [kevin@raspberrypi tmp]$ uname -a
    Linux raspberrypi 3.2.27+ #250 PREEMPT Thu Oct 18 19:03:02 BST 2012 armv6l GNU/Linux

In both cases, this is kernel version 3.2.27. Note that the *l* in
``armv6l`` refers to little endian.

Get and install `rpi-update <http://github.com/Hexxeh/rpi-update>`__::

    sudo apt-get install rpi-update

Now you can use rpi-update to get the current kernel::

    sudo apt-get upgrade rpi-update
    sudo rpi-update
