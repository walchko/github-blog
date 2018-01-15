Compare File Saving
========================

:date: 2017-08-09
:summary: Compare file saving (shelve, json, gzip, and pickle) in python

Saving data to disk has several options:

- shelve
- pickle
- gzip
- json

The problem with shelve is it is not cross platform. I had issues moving shelve
between macOS and Linux. Linux uses the bsd database and setting up python to
use it on macOS was a pain.

The simple test below compares pickle, pickle with gzip, and json. Json was the
the easiest and performed just as well.

.. gist:: walchko/bb979d854ddd5197c090931a7f25a826 test.py python
