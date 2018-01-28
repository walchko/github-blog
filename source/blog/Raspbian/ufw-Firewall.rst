Firewall
==========

:date: 2016-04-30
:summary: protecting your computer

Uncomplicated Firewall (ufw)
-----------------------------

Install: ``sudo apt-get install ufw``

Examples:

::

	sudo ufw [--dry-run] [options] [rule syntax]
	sudo ufw allow 22
	sudo ufw allow ssh

::

	pi@zoidberg:~/tmp $ sudo ufw status
	Status: inactive

- allow
- deny
- reject
- limit
- status: displays if the firewall is active or inactive
- show: displays the current running rules on your firewall
- reset: disables and resets the firewall to default
- reload: reloads the current running firewall
- disable: disables the firewall

Resources:
-----------

* `Jack Wallen, Linux.com **An Introduction to UFW** <https://www.linux.com/learn/introduction-uncomplicated-firewall-ufw>`_
