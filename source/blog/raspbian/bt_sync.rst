

BitTorrent Sync
===============

:date: 2015-07-12
:summary: Sync'ing files across machines without the cloud.


.. figure:: {filename}/blog/raspbian/pics/bt-sync.png
	:width: 200px
	:align: center

Use BitTorrent's `Sync <http://www.getsync.com>`__ program to keep files
on different computers up to date. You need to add the repository for
``apt-get`` to get the package from, first create the file.

::

    sudo nano /etc/apt/sources.list.d/btsync.list

Next, copy these lines into it:

::

    deb http://debian.yeasoft.net/btsync wheezy main contrib non-free
    deb-src http://debian.yeasoft.net/btsync wheezy main contrib non-free

Next, import the signing key:

::

    sudo gpg --keyserver pgp.mit.edu --recv-keys 6BF18B15
    sudo gpg --armor --export 6BF18B15 | sudo apt-key add -

Finally, update and install:

::

    sudo apt-get update
    sudo apt-get install btsync

Uninstall
---------

To uninstall everything do:

::

    sudo apt-get --purge remove btsync btsync-common

If you just do ``btsync`` then it will leave stuff behind.

Configure Sync
--------------

A configuration window will pop up during install, just take the
defaults. Please **do not** select https or set a password for the web
gui!!! You can re-run the installer anytime by:

::

    sudo dpkg-reconfigure btsync

The check the ``status``, ``start``, or ``stop`` with:

::

    sudo service btsync status

Finally, login into the webui at ``computer_name:8888`` to configure
``btsync``.
