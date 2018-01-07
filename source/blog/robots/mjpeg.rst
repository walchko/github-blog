MJPEG Server in Python
==========================

:date: 2017-06-12
:summary: How to do an mjpeg server in python using OpenCV 3.2

Only Python and OpenCV
-------------------------

This uses *only* python for serving up the mjpeg stream. There are a couple of issues:

- Python's BaseHTTPServer sucks ... it can take 10-15 seconds (or longer) to serve up the
  mjpeg stream, but once it is going, it is fine
- It can only handle one connection at a time
- Hitting Ctrl-C to kill the program doesn't allow you to cleanly exit the program ... try/expect
  don't seem to work for this and it is because of how BaseTTPServer is designed.


.. gist:: walchko/0deed4152896e1940b0fe93c9eb4d061 mjpeg2.py python


Flask instead of BaseHTTPServer
----------------------------------

Flask sort of is a little better, but not really. Issues:

- Still has a delay in starting
- Adds a lot of packages that need to be installed

.. code-block:: python

	# have to find it :)


References
-------------

- `MJPEG Wikipedia <https://en.wikipedia.org/wiki/Motion_JPEG>`_
