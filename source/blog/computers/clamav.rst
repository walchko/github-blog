Clamav
=======

:date: 2015-12-15
:summary: Virus scanning on linux and macOS

.. figure:: {filename}/blog/computers/pics/clamav.png
    :width: 200px
    :align: center

`Clamav.net <http://www.clamav.net/>`_

Install
--------

On OSX you can use homebrew to install clamav::

	brew install clamav

Setup
------

Go to ``/usr/local/etc/clamav`` and modify clamd.conf freshclam.conf

Update virus definitions::

	freshclam

Note, if you update clamav via ``brew`` then you have to run ``freshclam`` to
get the databases. Homebrew installs programs in different folder based on
version numbers.

Use
----

There are a million options, but some useful ones are::

	clamscan FILE
	alias clamscan='clamscan -r -i --bell --move=/Users/`whoami`/viruses '

where:

- **i** only prints infected files
- **bell** dings a bell when a virus is detected
- **r** scans directories recursively
- **move** defines the location to move infected files to
