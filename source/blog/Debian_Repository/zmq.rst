ZeroMQ
=======

:date: 2016-07-28
:summary: Build a Debian zeromq package

How to build a `zmq <http://zeromq.org/>`_ package for debian.

- download src
- run ``./configure --prefix=$HOME/tmp/zroot``
- run ``make -j 4``
- run ``make install``
- build package like the others
