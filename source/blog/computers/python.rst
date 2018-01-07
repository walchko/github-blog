Python Setup
==============

:date: 2012-12-22
:summary: Python setup

.. figure:: {filename}/blog/computers/pics/python.png
    :width: 200px
    :align: center

Install on RPI
----------------

Out with the Old
~~~~~~~~~~~~~~~~~

First uninstall all the installed python ``apt-get`` crap, otherwise you get warnings and
files left behind when you use ``pip``::

	sudo apt-get remove python-pip python3-pip
	sudo apt-get autoremove
	sudo apt-get install python-dev

Fix permissions so we don't need ``sudo`` which is a security issue::

	$ sudo chown -R pi:pi /usr/local

Now all of the python modules using ``pip`` don't need ``sudo`` to get installed.

Get Current Python
~~~~~~~~~~~~~~~~~~~

`Raspbian <http://sowingseasons.com/blog/building-python-2-7-10-on-raspberry-pi-2.html>`_
is currently lazy on upgrading python to a current version.

Grab Pre-requisites:

.. code-block:: bashsession

	sudo apt-get update
	sudo apt-get upgrade -y
	sudo apt-get install build-essential libncursesw5-dev libgdbm-dev libc6-dev
	sudo apt-get install zlib1g-dev libsqlite3-dev tk-dev
	sudo apt-get install libssl-dev openssl

Download and Make:

.. code-block:: bashsession

	$ mkdir tmp
	$ cd tmp
	$ wget https://www.python.org/ftp/python/2.7.10/Python-2.7.10.tgz
	$ tar -zxvf Python-2.7.10.tgz
	$ cd Python-2.7.10
	$ ./configure
	$ make -j 4
	$ sudo make install

Setup Pip
~~~~~~~~~~

Get ``pip``::

	$ cd ..
	$ wget https://bootstrap.pypa.io/get-pip.py
	$ python get-pip.py

Python Packages
---------------

Alot of very useful packages are available from `PyPI <https://pypi.python.org/pypi>`_
and can be installed using ``pip``.

You can use ``pip`` to install and keep python libraries up to date.
Unfortunately ``pip`` isn't the best package manager, but it could be
worse ... ``apt-get`` anyone? Some useful, undocumented commands:

+--------------------+--------------------------------------+
| Pip flag           | Description                          |
+====================+======================================+
| list               | list installed packages              |
+--------------------+--------------------------------------+
| list --outdated    | list packages that can be upgraded   |
+--------------------+--------------------------------------+
| install *pkg*      | install a new package                |
+--------------------+--------------------------------------+
| install -U *pkg*   | upgrade an exisiting package         |
+--------------------+--------------------------------------+

Why the people who run ``pip`` don't make useful commands like
``pip upgrade`` or ``pip outdated`` I don't know. Instead there are
duplicate commands like ``pip freeze`` which is the same as
``pip list`` and adds no real value.
