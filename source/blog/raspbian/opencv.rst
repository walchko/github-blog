
OpenCV
============

:date: 2016-02-28
:modified: 2016-12-30
:summary: Setting up OpenCV

.. figure:: pics/opencv.png
   :width: 200px
   :align: center


Rasbian
--------

::

	pi@zoidberg ~ $ apt-cache search opencv
	libopencv-core-dev - development files for libopencv-core
	python-opencv - Python bindings for the computer vision library
	libopencv-dev - kevin - installs opencv 3.1.0 and dev stuff

::

	sudo apt-get update
	sudo apt-get upgrade
	sudo apt-get install libopencv-dev
	cd /usr/local/lib
	sudo ldconfig

OSX
-----

::

    brew install opencv3


Windoze
---------

Install

* Python 2.7
* Numpy
* Matplotlib
* Download OpenCV 3.x from `github <https://github.com/Itseez/opencv/releases>`_
* Copy ``C:\Users\Kevin.Walchko\Downloads\opencv\build\python\2.7\x64\cv2.pyd`` to ``C:\Python27\Lib\site-packages``

To double check it worked, open a python window and type::

    >>> import cv2
    >>> print cv2.__version__
    3.1.0

If you get back the correct version number all is good!
