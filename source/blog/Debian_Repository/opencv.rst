Open Computer Vision (OpenCV)
=================================

:date: 2016-12-30
:summary: Build a Debian OpenCV 3.x package

How to build a package for OpenCV 3.x.

Setup
-------

To get ``ffmpeg`` on jessie, edit ``/etc/apt/sources.list`` and add::

  deb http://www.deb-multimedia.org jessie main

These will pull in a lot of other packages, but that is ok::

  pip install -U numpy

``numpy`` will take a looong time to  build.

::

	sudo apt-get update
	$ sudo apt-get upgrade
	$ sudo apt-get install deb-multimedia-keyring

	$ sudo apt-get update
	$ sudo apt-get upgrade
	$ sudo apt-get install build-essential cmake git pkg-config python-dev swig
	$ sudo apt-get install ffmpeg
	$ sudo apt-get install libjpeg8-dev libtiff4-dev libjasper-dev libpng12-dev
	$ sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
	$ sudo apt-get install gfortran libeigen3
	$ sudo apt-get install libopenblas-dev libatlas-dev libblas-dev liblapack-dev
	$ sudo apt-get install libgtk2.0-dev

	mkdir ~/tmp/fakeroot


.bashrc
----------

::

  export LD_LIBRARY_PATH=/usr/local/lib

Compile
---------

A good reference for this is `here <http://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/>`_
Since this is running in an embedded system, VTK support is not included. Grab
the source::

	git clone https://github.com/opencv/opencv.git

Now go into the src directory and do::

	cd opencv/
	mkdir build
	cd build
	cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=$HOME/tmp/fakeroot ..
	make -j 4

Package Build
-------------

debian/control
---------------

::

	Package: libopencv-dev
	Version: 3.1.0
	Section: custom
	Priority: optional
	Architecture: all
	Essential: no
	Installed-Size: 1024
	Maintainer: me
	Description: kevin - installs opencv 3.1.0 and dev stuff

Post Install
--------------

You will need to link the library, otherwise the libraries won't be found::

	sudo ldconfig -v /usr/local/lib

Now you may have to go to ``/usr/local/lib`` and use ``sudo ldconfig`` for it to work. To
double check everything worked, try::

	ldconfig -p | grep cv

You should see the libraries.


Install `ffmpeg <http://linuxg.net/how-to-install-ffmpeg-2-2-2-muybridge-on-debian-sid-debian-jessie-and-debian-wheezy/>`_
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**This should be covered above**

Also, FFmpeg 2.2.2 is available via the Multimedia Repository, so we have to
add the repo to our system, update the local repo index and install the
deb-multimedia-keyring and ffmpeg packages. Like this::

  $ sudo sh -c 'echo "deb http://www.deb-multimedia.org jessie main" >> /etc/apt/sources.list'
  $ sudo apt-get update
  $ sudo apt-get install deb-multimedia-keyring
  $ sudo apt-get install ffmpeg


Old
----

::

	sudo apt-get install libavformat-dev libjasper-dev libjpeg8-dev libgtk2.0-dev
	sudo apt-get install ffmpeg libdc1394-22-dev
	sudo apt-get install build-essential cmake cmake-curses-gui pkg-config libpng12-0
	sudo apt-get install libpng12-dev libpng++-dev libpng3 libpnglite-dev zlib1g-dbg
	sudo apt-get install zlib1g zlib1g-dev pngtools libtiff5-dev libtiff5 libtiffxx0c2
	sudo apt-get install libtiff-tools libeigen3-dev
	sudo apt-get install libtbb libjpeg8 libjpeg8-dev libjpeg8-dbg libjpeg-progs
	sudo apt-get install ffmpeg libavcodec-dev libavformat-dev libunicap2
	sudo apt-get install libunicap2-dev swig libv4l-0 libv4l-dev python-dev
