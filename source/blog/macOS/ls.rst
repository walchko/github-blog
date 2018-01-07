Making The Command Line Useful
================================

:date: 2016-06-09

ls
---

OSX uses BSD ``ls`` utility. To make the command more appealing, use::

	alias ls='ls -G'

where:

	G: colorized output
	a: displays files beginning with a ``.``
	h: human readable

To edit the colors you can::

	export CLICOLOR=1
	export LSCOLORS=ExFxCxDxBxegedabagacad          

The string is 22 characters long and comprised of 11 two letter orders as described below.

The color designators are as follows:

==== =============
Ltr  Color
==== =============
a	 black
b	 red
c	 green
d	 brown
e	 blue
f	 magenta
g	 cyan
h	 light grey
A	 bold black, usually shows up as dark grey
B	 bold red
C	 bold green
D	 bold brown, usually shows up as yellow
E	 bold blue
F	 bold magenta
G	 bold cyan
H	 bold light grey; looks	like bright white
x	 default foreground or background
==== =============

The order of the attributes are as	follows:

1.	directory
2.	symbolic link
3.	socket
4.	pipe
5.	executable
6.	block special
7.	character special
8.	executable with	setuid bit set
9.	executable with	setgid bit set
10.	directory writable to others, with sticky bit
11.	directory writable to others, without sticky bit

Homebrew
----------

You can also use the GNU coreutils (which contains ls) however, not sure this is such a 
great idea because it contains a lot of other utilities too. You could probably install it
and just do::

	alias ls='/usr/local/opt/coreutils/libexec/gnubin/ls'
