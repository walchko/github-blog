

DLNA
========

:date: 2015-07-23
:summary: Using linux to serve up videos using DLNA

.. image:: pics/dlna.jpg
	:width: 200px
	:align: center

Update
-------

I now use `Plex <https://www.plex.tv>`_ to serve up media at home on my Raspberry
Pi.

Original Article
------------------

`source <http://www.htpcguides.com/install-readymedia-minidlna-1-1-4-raspberry-pi/>`__

Digital Living Network Alliance (`DLNA <http://www.dlna.org/>`__ ) servers allow you to
stream your media library on your HTPC server to any DLNA
enabled client. DLNA enabled clients include Bluray players, XBOX 360, PS3,
and some TVs. There are other DLNA servers out there for the Raspberry Pi. I was
looking at MediaTomb but it consumes several hundred megabytes of RAM when it is in use.
This it not ideal on the low spec Pi running Raspbian. I had trouble with miniDLNA 1.0.24
not displaying avi (Divx, XviD) files in its folder database, this bug has been patched
and fixed so that avi files – at least the ones I tested – were accessible by my DLNA
clients. I will assume you have already mounted a USB hard drive for this guide –
here is my Properly Mount USB Storage on Raspberry Guide in case you do need to mount.
You will be compiling miniDLNA for Raspbian from source, it only takes a few minutes.

Install ReadyMedia (miniDLNA)
------------------------------

The latest version of ReadyMedia miniDLNA in the Raspbian repos is ancient so we are
going to compile miniDLNA (now ReadyMedia) from source on Raspbian. This should fix avi
problems you may have had in the past like them not showing up in the library.

Remove your old miniDLNA 1.0.24::

	sudo apt-get purge minidlna -y
	sudo apt-get remove minidlna
	sudo apt-get autoremove -y

Make sure you have a source repository, default Raspbian does not include this ::

	echo "deb-src http://archive.raspbian.org/raspbian wheezy main contrib non-free" | sudo tee -a /etc/apt/sources.list

Update repositories so it will detect your new source repo ::

	sudo apt-get update

Grab dependencies for building it from source::

	sudo apt-get build-dep minidlna -y

If you get any errors you can install the dependencies manually::

	sudo apt-get install libjpeg-dev libsqlite3-dev libexif-dev libid3tag0-dev libvorbis-dev libflac-dev -y

Download miniDLNA 1.1.4 source or whatever the latest version is::

	wget http://sourceforge.net/projects/minidlna/files/minidlna/1.1.4/minidlna-1.1.4.tar.gz

SourceForge has some stability issues so here is a Dropbox mirror in case it is down::

	wget https://www.dropbox.com/s/hhv7ee057plec7a/minidlna-1.1.4.tar.gz

Unpack miniDLNA::

	tar -xvf minidlna-1.1.4.tar.gz

Enter the miniDLNA directory::

	cd minidlna-1.1.4

Configure, make and install miniDLNA, it will take 5 minutes::

	./configure && make && sudo make install

Copy the default configuration file::

	sudo cp minidlna.conf /etc/

Copy the startup daemon script to autostart ReadyMedia miniDLNA on boot::

	sudo cp linux/minidlna.init.d.script /etc/init.d/minidlna

Make the startup script executable::

	sudo chmod +x /etc/init.d/minidlna

Update rc to use the miniDLNA defaults::

	sudo update-rc.d minidlna defaults

Edit the configuration::

	sudo nano /etc/minidlna.conf

Edit the following to point to your media

This version of minidlna will give you multiple folders under Video. Before movies and TV
would have been under separate categories, now movies and TV will both be under the
category video. It will also show the folder structure of them instead of showing just
the video files. Avi files will also show up and be streamable.

inotify uses resources because it autoupdates your library, if you don’t use inotify you
will have to manually restart and reload the miniDLNA service

The friendly name is how your miniDLNA server will show up to its streaming clients::

	media_dir=V,/mnt/usbstorage/Movies
	media_dir=V,/mnt/usbstorage/TV
	media_dir=A,/mnt/usbstorage/Music
	media_dir=P,/mnt/usbstorage/Pictures
	# Names the DLNA server
	friendly_name=RasPi Media Server
	# Tells the DLNA to update the library when there are changes
	inotify=yes

Ctrl+X, Y and Enter to save and exit

Start the minidlna service::

	sudo service minidlna restart

Now make sure miniDLNA starts on boot

It will run on port 8200 so you can check how many media files it has indexed there::

	sudo reboot

Your Raspberry Pi DLNA server will now be accessible to stream media to your clients:
PCs, XBOX, PS3, Phones, TVs and more.
