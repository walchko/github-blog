
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

Linux seems to hate OpenCV and never has a current debian package of it. Go
`here <https://github.com/MomsFriendlyRobotCompany/dpkg_opencv>`_ to get
a current version.

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
