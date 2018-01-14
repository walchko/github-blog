
Node.js
=======

:date: 2015-09-03
:summary: Install and use Node.js
:modified: 2016-12-30

.. figure:: pics/nodejs.png
	:width: 200px
	:align: center

Installing Node.js
-------------------

The Raspbian software is old, you need to use another source. Following the `directions
on node.js <https://nodejs.org/en/download/package-manager/>`_ do the following::

	curl -sL https://deb.nodesource.com/setup_7.x | sudo -E bash -
	sudo apt-get install -y nodejs

**WARNING:** This installs ``node`` to ``/usr/lib`` instead of ``/usr/local/lib``.

**WARNING:** This only works for ARMv7 (RPi 3) and doesn't work for ARMv6 (RPi2 or Zero)

Packages
---------

Nodejs uses Node Package Manager (npm) for add/removing packages. The
best way is to build a package.json file in your project and run
``npm install`` to get what you need. See these
`tutorials <https://docs.npmjs.com/>`__ for more info.


============================== =======================================================
Command Cheatsheet             Description
============================== =======================================================
adduser                        Create a user account on npmjs.com
cache clean                    Clean up old packages in the cache
config list                    List config settings
config get prefix              Get path to global location
config set prefix=$HOME/.node  Set the global location
init                           Create a new ``package.json`` file
install [-g|--global]          Install packages locally or to the global location
install <pgk> --save           Install package and save it to ``package.json``
list [-g|--global]             List packages locally or globally
list --depth=0                 List packages, but only print top level
outdated [-g|--global]         Check for outdated packages
uninstall [-g|--global]        Uninstall packages locally or from the global location
update [-g|--global]           Update packages locally or globally
search <pkg>                   Search npmjs.com for a package
============================== =======================================================

Useful Packages
-----------------

Run a simple file server::

	npm install -g httpserver
	httpserver 8080 localhost

- httpserver
- archeyjs
