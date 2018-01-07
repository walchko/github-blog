This Website
========================


Construction
--------------

This site is created using `Pelican <https://github.com/getpelican/pelican>`_.
Pelican has a number of useful plug-ins and themes. The webpages and/or posts
can be written in restructured text (rst) or markdown (md) and uses python's
doctools to convert it into a web site.

=============== ===============
Useful Commands Descriptions
=============== ===============
make html       creates the website
make clean      deletes the current website output
=============== ===============

Setup
-----

The pelican configuration file (``peliconconf.py``) I used is shown below:

.. code-include:: ../../pelicanconf.py
	:lexer: python

Makefile
---------

To build the website I do: ``make html``

.. code-include:: ../../Makefile
	:lexer: Makefile

Publishing
-----------

I use travis-ci publisher to build and publish to my github website. The yaml
file below uses a github access token to push the pages to the gh-pages branch.

 .. code-include:: ../../.travis.yml
 	:lexer: yaml

Pygments
-----------

To setup the ``css`` for code, I do::

	kevin@Logan pelican $ pygmentize -S default -f html > pygments.css
	kevin@Logan pelican $ mv pygments.css themes/kevin/static/css/
	kevin@Logan pelican $ make html
