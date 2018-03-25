Python
===============

:date: 2017-02-17
:summary: Python code snippets and useful libraries

.. image:: pics/python-snake.jpg
	:width: 100%

A useful reference is `Python Module of the Week <https://pymotw.com/2/contents.html>`_

Simple hello world
---------------------

```python
#!/usr/bin/env python2
from __future__ import division
from __future__ import print_function

def main():
	print('Hello World!')

if __name__ == "__main__":
	main()
```

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


References
-------------

- `Python Operators <https://www.tutorialspoint.com/python/bitwise_operators_example.htm>`_
