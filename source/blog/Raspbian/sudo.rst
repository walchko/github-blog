sudo
=========

:date: 2017-08-20
:summary: pi and sudo

The default system installation you should have the following line in ``/etc/sudoers``::

  pi ALL=(ALL) NOPASSWD: ALL

It tells sudo to allow user pi to run all cammands as root user without even providing password. 
You can change last ALL and specify comma delimited list of commands (with their full path) 
allowed to run. In your case you should change this line to::

  pi ALL=(ALL) NOPASSWD: /usr/bin/apt-get, /sbin/shutdown

Note that there is one more line in sudoers affecting pi user::

  %sudo   ALL=(ALL:ALL) ALL

This line let all users in group sudo (% character in front of the name means it's a group 
name instead of user name) run ALL passwords providing they know their OWN password. If you 
leave this line, user pi will be able to run all other commands but will be asked for his password.

If you want to prevent this from happening you can either remove this line or remove user pi from  
sudo group.

After making changes to ``/etc/sudoers`` file you may want to inspect that it really does what you 
want by calling ``sudo -l -U pi command``.
