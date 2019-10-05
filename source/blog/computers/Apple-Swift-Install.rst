Swift Setup
============

:date: 2015-12-24
:summary: How to setup swift

.. figure:: pics/swift.png
    :width: 200px
    :align: center

Install Swift from `swift.org <http://swift.org>`_ without going through XCode.
This installs the open source compiler and libraries.

Install
--------

1. Download from swift.org
2. Installs to ``/Library/Developer/Toolchains/``
3. Add to path to use on CLI: ``$ export PATH=/Library/Developer/Toolchains/swift-latest.xctoolchain/usr/bin:"${PATH}"``

or

::

	brew install swift
