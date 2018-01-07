Python
===============

:date: 2017-02-17
:summary: Python code snippets and useful libraries

.. image:: pics/python-snake.jpg
	:width: 100%

A useful reference is `Python Module of the Week <https://pymotw.com/2/contents.html>`_

Simple hello world
---------------------

.. gist:: walchko/7dea22efcab640d0094e8e90fb851301 hello_world.py python

Linux Fixing Python Path
----------------------------

Edit ``/etc/python3.4/sitecustomize.py``:

.. code-block:: python

	import sys, os
	my_site = os.path.join(os.environ['HOME'],
                       '/usr/local/lib/python3.4/dist-packages')
	sys.path.insert(0, my_site)

For some retarded reason, I keep installing updates with ``pip3``, but it doesn't
see them in ``/usr/local/lib/python3.4/ ..`` directory, it looks in ``/usr/lib/python3.4 ..``.

The Future is Now
-------------------

The future library backports some potentially useful python3 code to python2:

.. code-block:: python

	from __future__ import print_function # changes print to a function instead of a statement
	from __future__ import division       # all division is floating point

	print('hello!')  # now a function

	1/3   # result is 0.333, float division ... similar to other languages
	10//3 # result is 3, this is integer division

Other capabilities can be found in the docs.

- `future docs <https://docs.python.org/2/library/__future__.html>`_


Temporary Files and Folders
-----------------------------

Create a simple temp file:

.. code-block:: python

	import os
	import tempfile

	print
	print 'TemporaryFile:'
	temp = tempfile.TemporaryFile()
	try:
		print 'temp:', temp
		print 'temp.name:', temp.name
	finally:
		# Automatically cleans up the file
		temp.close()

Create a temp folder:

.. code-block:: python

	import os
	import tempfile

	directory_name = tempfile.mkdtemp()
	print directory_name

	# Clean up the directory yourself
	os.removedirs(directory_name)

- `tempfile examples <https://pymotw.com/2/tempfile/>`_

Serial Comm
--------------

A simple python script is:

.. code-block:: python

	import serial

	# s = serial.Serial('/dev/ttyS0', 19200, timeout=1)
	s = serial.Serial()
	s.baudrate = 19200
	s.port = '/dev/serial0'
	s.write('hello')
	s.close()

- `pyserial docs <http://pyserial.readthedocs.io>`_

Platform
----------

Get platform info:

.. code-block:: python

	import platform

	print 'Platform     :', platform.platform()
	print 'System       :', platform.system()
	print 'Version      :', platform.python_version()
	print 'Version tuple:', platform.python_version_tuple()
	print 'Compiler     :', platform.python_compiler()
	print 'Build        :', platform.python_build()

Will output something like this for linux:

::

	Platform     : Linux-4.4.48-v7+-armv7l-with-debian-8.0
	System       : Linux
	Version      : 2.7.9
	Version tuple: ('2', '7', '9')
	Compiler     : GCC 4.9.2
	Build        : ('default', 'Sep 17 2016 20:26:04')


Netaddr
----------

`Netaddr` has a lot of useful functions for working with ip addresses.

.. code-block:: python

	>>> import netaddr
	>>> dir(netaddr)
	['AddrConversionError', 'AddrFormatError', 'EUI', 'IAB', 'INET_PTON', 'IPAddress', 'IPGlob', 'IPNetwork', 'IPRange', 'IPSet', 'N', 'NOHOST', 'NotRegisteredError', 'OUI', 'P', 'STATUS', 'SubnetSplitter', 'VERSION', 'Z', 'ZEROFILL', '__builtins__', '__doc__', '__file__', '__name__', '__package__', '__path__', '__version__', '_sys', 'all_matching_cidrs', 'base85_to_ipv6', 'cidr_abbrev_to_verbose', 'cidr_exclude', 'cidr_merge', 'cidr_to_glob', 'compat', 'contrib', 'core', 'eui', 'eui64_bare', 'eui64_base', 'eui64_cisco', 'eui64_unix', 'eui64_unix_expanded', 'glob_to_cidrs', 'glob_to_iprange', 'glob_to_iptuple', 'ip', 'iprange_to_cidrs', 'iprange_to_globs', 'ipv6_compact', 'ipv6_full', 'ipv6_to_base85', 'ipv6_verbose', 'iter_iprange', 'iter_nmap_range', 'iter_unique_ips', 'largest_matching_cidr', 'mac_bare', 'mac_cisco', 'mac_eui48', 'mac_pgsql', 'mac_unix', 'mac_unix_expanded', 'smallest_matching_cidr', 'spanning_cidr', 'strategy', 'valid_eui64', 'valid_glob', 'valid_ipv4', 'valid_ipv6', 'valid_mac', 'valid_nmap_range']
	>>> netaddr.valid_ipv4('192.168.1.1')
	True
	>>> netaddr.valid_ipv4('192.168.1.300')
	False

Get IP Address from Interface
--------------------------------

.. code-block:: python

	import os, re

	def getIP(iface):
		search_str = 'ip addr show wlan0'.format(iface)
		ipv4 = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen(search_str).read()).groups()[0]
		ipv6 = re.search(re.compile(r'(?<=inet6 )(.*)(?=\/)', re.M), os.popen(search_str).read()).groups()[0]
		return (ipv4, ipv6)


Networking
-------------

A really good resource for network programming with python is `here <{filename}/blog/programming/static/PythonNetBinder.pdf>`_

References
-------------

- `Python Operators <https://www.tutorialspoint.com/python/bitwise_operators_example.htm>`_
