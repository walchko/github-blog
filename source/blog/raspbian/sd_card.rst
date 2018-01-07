
SD Card
=======

:date: 2015-06-10
:summary: Setting up an SD card for RPi

.. figure:: {filename}/blog/raspbian/pics/sd.jpg
	:width: 200px
	:align: center

Backup and Restore
--------------------

Use the ``dd`` command to make a full backup of the image:

::

    dd if=/dev/sdx of=/path/to/image

or for compression:

::

    dd if=/dev/sdx | gzip > /path/to/image.gz

Where sdx is your SD card and the target could be
~/raspbian\_wheezy\_\ ``date "+%Y%m%d_%T"``. This will save it to your
home directory and append the current date and time on the end of the
filename.

To restore the backup you reverse the commands::

    dd if=/path/to/image of=/dev/sdx

or when compressed::

    gzip -dc /path/to/image.gz | dd of=/dev/sdx
