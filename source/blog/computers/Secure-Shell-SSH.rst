SSH
====

:date: 2012-09-10
:summary: SSH usage

.. figure:: pics/ssh.jpg
    :width: 200px
    :align: center


Secure Shell (`SSH <http://www.openssh.org>`__) is a cryptographic
network protocol for secure data communication, remote shell services or
command execution and other secure network services between two
networked computers that connects, via a secure channel over an insecure
network, a server and a client (running SSH server and SSH client
programs, respectively). The protocol specification distinguishes
between two major versions that are referred to as SSH-1 and SSH-2.

The best-known application of the protocol is for access to shell
accounts on Unix-like operating systems, but it can also be used in a
similar fashion for accounts on Windows. It was designed as a
replacement for Telnet and other insecure remote shell protocols such as
the Berkeley rsh and rexec protocols, which send information, notably
passwords, in plaintext, rendering them susceptible to interception and
disclosure using packet analysis. The encryption used by SSH is intended
to provide confidentiality and integrity of data over an unsecured
network, such as the Internet. [1]_

Key Types
------------

`Source <http://stackoverflow.com/questions/2841094/what-is-the-difference-between-dsa-and-rsa>`__

DSA is faster in signing, but slower in verifying. A DSA key of the same strength as
RSA (1024 bits) generates a smaller signature. An RSA 512 bit key has been cracked, but
only a 280 DSA key.

Also note that DSA can only be used for **signing/verification**, whereas RSA can be
used for **encryption/decrypt** as well.


Summary of Useful Commands
--------------------------

=========== ======================================  ==========================
Command     Example                                 Use
=========== ======================================  ==========================
ssh         ssh kevin@thor.local                    login to a computer
ssh-keygen  ssh-keygen                              generate an ssh key
ssh-keygen  ssh-keygen -lvf                         view the key finger print
ssh-copy-id ssh-copy-id kevin@loki.local            copy key to remote server
=========== ======================================  ==========================

Key Generation
---------------

To increase security, you can disable password logins and rely on ssh
public keys. To do this, take a look
`here <https://wiki.archlinux.org/index.php/SSH_Keys>`__ for details.
Basic steps are:

1. Generate an ssh key pair using either RSA (2048-4096 bit) or DSA
   (1024 bit) both public and private keys. They will be stored in
   ``~/.ssh`` with the public key having .pub appended to the end. Two ways to
   generate keys are shown below (pick one).

   ::

       ssh-keygen -t dsa -b 1024 -C "$(whoami)@$(hostname)-$(date)"
       ssh-keygen -t rsa -b 4096 -C "$(whoami)@$(hostname)-$(date)"

   Note you can create a key for a different username if you change
   $(whoami) to the user name you want. If no type is specified, the default is RSA
   2048 bits.

   ::

		[kevin@Tardis ~]$ ssh-keygen -C "test@$(hostname)-$(date)"
		Generating public/private rsa key pair.
		Enter file in which to save the key (/Users/kevin/.ssh/id_rsa): test
		Enter passphrase (empty for no passphrase):
		Enter same passphrase again:
		Your identification has been saved in test.
		Your public key has been saved in test.pub.
		The key fingerprint is:
		00:04:27:9d:7e:33:6f:65:1c:a5:e0:c3:82:5d:7b:92 test@Tardis.local-Tue Apr 21 22:29:40 MDT 2015
		The key's randomart image is:
		+--[ RSA 2048]----+
		|  o++.  o  ..    |
		|   oo+ + +..     |
		|   .. + E.o.     |
		|    . +o ++      |
		|     . +So       |
		|        o        |
		|       .         |
		|                 |
		|                 |
		+-----------------+


   Also note, it is advisable you create a strong pass phrase that you won't forget. However,
   I typically do not create one. But it does add an added level of protection.

2. Copy the public key (.pub) to the server you will connect to:

   ::

       ssh-copy-id username@remote-server.org

   This will update ~/.ssh/authorized\_keys in the process. **Note:** ``ssh-copy-id``
   may need to be installed. Most Linux/Unix systems should have this, but for OSX do
   ``brew install ssh-copy-id``. Also ensure the correct protections are on the file by:

   ::

       chmod 600 ~/.ssh/authorized_keys

3. Edit /etc/ssh/sshd\_config to disable password logins.

   ::

       PasswordAuthentication no
       ChallengeResponseAuthentication no


SSH Key Finger Prints
---------------------

To view the finger print of a key:

::

    [kevin@Tardis ~]$ ssh-keygen -lvf ~/.ssh/id_rsa.pub
	2048 b1:58:41:c5:93:b3:bc:c7:34:5b:e8:be:bc:15:ff:55  kevin@tardis.local (RSA)
	+--[ RSA 2048]----+
	|       .oo..     |
	|         .=      |
	|        o. + .   |
	|       o oo + .  |
	|      . S  = +. E|
	|          . =  o.|
	|           o  . o|
	|           ...  o|
	|            +o  .|
	+-----------------+

This tells you the type of key (e.g., RSA or DSA), the bit size, what email/account it is
tied to, and a graphical representation of the key. In this case, the 2048 bits of my public
RSA key.

Config
-----------

By default, ssh uses id_rsa.pub. If you want to match a specific key to a specific host, you need
a ``~/.ssh/config`` file.  Example::

	Host bitbucket.org
	 IdentityFile ~/.ssh/bitbucket_rsa
	Host github.com
	 IdentityFile ~/.ssh/github_rsa


16 SSH Hacks
------------

The original source for this work is
`here <http://www.itworld.com/it-managementstrategy/261500/16-ultimate-openssh-hacks>`__

So you think you know OpenSSH inside and out? Test your chops against
this hit parade of 16 expert tips and tricks, from identifying
monkey-in-the-middle attacks to road warrior security to attaching
remote screen sessions. Follow the countdown to the all-time best
OpenSSH command!

`Running SSH on a non-standard
port <xhttp://www.itworld.com/nls_unixssh0500506>`__

SSH tips #16-14:Detecting MITM attacks
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When you log into a remote computer for the first time, you are asked if
you want to accept the remote host's public key. Well how in the heck do
you know if you should or not? If someone perpetrated a successful
monkey-in-the-middle attack, and is presenting you with a fake key so
they can hijack your session and steal all your secrets, how are you
supposed to know? You can know, because when new key pairs are created
they also create a unique fingerprint and randomart image:

::

    $ ssh-keygen -t rsa -C newserver -f .ssh/newkey

    Generating public/private rsa key pair.
    Enter passphrase (empty for no passphrase):
    Enter same passphrase again:
    Your identification has been saved in .ssh/newkey.
    Your public key has been saved in .ssh/newkey.pub.
    The key fingerprint is:
    44:90:8c:62:6e:53:3b:d8:1a:67:34:2f:94:02:e4:87 newserver
    The key's randomart image is:
    +--[ RSA 2048]----+
    |oo   +.o.        |
    |. = B o.         |
    | E X +  .        |
    |  B B ..         |
    | . * o  S        |
    |  .              |
    |                 |
    |                 |
    |                 |
    +-----------------+

SSH tip #16: Retrieve the fingerprint and randomart image of an SSH key
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you make a copy of this when you create new encryption keys, then you
can fetch a key's fingerprint and randomart image anytime to compare and
make sure they have not changed:

::

    $ ssh-keygen -lvf  keyname

SSH tip #15: View all fingerprints and randomart images in known\_hosts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

And you can see all of them in your ~/.ssh/known\_hosts file:

::

    $ ssh-keygen -lvf ~/.ssh/known_hosts

SSH tip #14: Verify server keys
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can see the fingerprint and randomart for any computer you're
logging into by configuring/etc/ssh/ssh\_config on your client computer.
Simply uncomment the VisualHostKey option and set it to yes:

::

    VisualHostKey yes

Then login to any remote computer to test it:

::

    $ ssh user@host2
    Host key fingerprint is 66:a1:2a:23:4d:5c:8b:58:e7:ef:2f:e5:49:3b:3d:32
    +--[ECDSA  256]---+
    |                 |
    |                 |
    |  . o   .        |
    | + = . . .       |
    |. + o . S        |
    | o   o oo        |
    |. + . .+ +       |
    | . o .. E o      |
    |      .o.+ .     |
    +-----------------+

    user@host2's password:

Obviously you need a secure method of getting verified copies of the
fingerprint and randomart images for the computers you want to log into.
Like a hand-delivered printed copy, encrypted email, the scp command,
secure ftp, read over the telephone...The risk of a successful MITM
attack is small, but if you can figure out a relatively painless
verification method it's cheap insurance.

SSH tip #13: Attach to a remote GNU screen session
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can attach a GNU screen session remotely over SSH; in this example
we'll open a GNU screen session on host1, and connect to it from host2.
First open and then detach a screen session on host1, named testscreen:

::

     host1 ~ $ screen -S testscreen

Then detach from your screen session with the keyboard combination
Ctrl+a+d:

::

    [detached from 3829.testscreen]

You can verify that it's still there with this command:

::

    host1 ~ $ screen -ls

There is a screen on:

::

    3941.testscreen (03/18/2012 12:43:42 PM) (Detached)
    1 Socket in /var/run/screen/S-host1.

Then re-attach to your screen session from host2:

::

    host1 ~ $ ssh -t terry@uberpc screen -r testscreen

You don't have to name the screen session if there is only one.

vSSH tip #12: Launch a remote screen session
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

What if you don't have a running screen session? No worries, because you
can launch one remotely:

::

    host1 ~ $ ssh -t user@host2 /usr/bin/screen -xRR

SSH tip #11: SSHFS is better than NFS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

sshfs is better than NFS for a single user with multiple machines. I
keep a herd of computers running because it's part of my job to always
be testing stuff. I like having nice friendly herds of computers. Some
people collect Elvis plates, I gather computers. At any rate opening
files one at a time over an SSH session for editing is slow; with sshfs
you can mount entire directories from remote computers. First create a
directory to mount your sshfs share in:

::

    $ mkdir remote2

Then mount whatever remote directory you want like this:

::

    $ sshfs user@remote2:/home/user/documents remote2/

Now you can browse the remote directory just as though it were local,
and read, copy, move, and edit files all you want. The neat thing about
sshfs is all you need is sshd running on your remote machines, and
thesshfs command installed on your client PCs.

SSH tip #10: Log in and run a command in one step
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can log in and establish your SSH session and then run commands, but
when you have a single command to run why not eliminate a step and do it
with a single command? Suppose you want to power off a remote computer;
you can log in and run the command in one step:

::

    carla@local:~$ ssh user@remotehost sudo poweroff

This works for any command or script. (The example assumes you have a
sudo user set up with appropriate restrictions, because allowing a root
login over SSH is considered an unsafe practice.) What if you want to
run a long complex command, and don't want to type it out every time?
One way is to put it in a Bash alias and use that. Another way is to put
your long complex command in a text file and run it according to tip #9.

SSH tip #9: Putting long commands in text files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Put your long command in a plain text file on your local PC, and then
use it this way to log in and run it on the remote PC:

::

    carla@local:~$ ssh user@remotehost "`cat filename.txt`"

Mind that you use straight quotations marks and not fancy ones copied
from a Web page, and back-ticks, not single apostrophes.

vSSH tip #8: Copy public keys the easy way
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ssh-copy-id command is not as well-known as it should be, which is a
shame because it is a great time-saver. This nifty command copies your
public key to a remote host in the correct format, and to the correct
directory. It even has a safety check that won't let you copy a private
key by mistake. Specify which key you want to copy, like this:

::

    $ ssh-copy-id -i .ssh/id_rsa.pub user@remote

SSH tip #7: Give SSH keys unique names
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Speaking of key names, did you know you can name them anything you want?
This helps when you're administering a number of remote computers, like
this example which creates then private key web-admin and public key
web-admin.pub:

::

    $ ssh-keygen -t rsa -f .ssh/web-admin

SSH tip #6: Give SSH keys informative comments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Another useful way to label keys is with a comment:

::

    $ ssh-keygen -t rsa -C "downtown lan webserver" -f .ssh/web-admin

Then you can read your comment which is appended to the end of the
public key.

SSH tip #5: Read public key comments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    $ less .ssh/web-admin.pub

    ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQC1

    [snip] KCLAqwTv8rhp downtown lan webserver

SSH tip #4: Logging in with server-specific keys
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Then when you log in, specify which key to use with the -i switch:

::

    $ ssh -i .ssh/web-admin.pub user@webserver

SSH tip #3: Fast easy known\_hosts key management
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

I love this one because it's a nice time-saver, and it keeps my
~/.ssh/known\_hosts files tidy: using ssh-keygen to remove host keys
from the ~/.ssh/known\_hosts file. When the remote machine gets new SSH
keys you'll get a warning, when you try to log in, that the key has
changed. Using this is much faster than manually editing the file and
counting down to the correct line to delete:

::

    $ ssh-keygen -R remote-hostname

Computers are supposed to make our lives easier, and it's ever so lovely
when they do.

SSH tip #2: SSH tunnel for road warriors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When you're at the mercy of hotel and coffee shop Internet, a nice
secure SSH tunnel makes your online adventures safer. To make this work
you need a server that you control to act as a central node for escaping
from hotspot follies. I have a server set up at home to accept remote
SSH logins, and then use an SSH tunnel to route traffic through it. This
is useful for a lot of different tasks. For example I can use my normal
email client to send email, instead of hassling with Web mail or
changing SMTP server configuration, and all traffic between my laptop
and home server is encrypted. First create the tunnel to your personal
server:

::

    carla@hotel:~$ ssh -f carla@homeserver.com -L 9999:homeserver.com:25 -N

This binds port 9999 on your mobile machine to port 25 on your remote
server. The remote port must be whatever you've configured your server
to listen on. Then configure your mail client to use localhost:9999 as
the SMTP server and you're in business. I use Kmail, which lets me
configure multiple SMTP server accounts and then choose which one I want
to use when I send messages, or simply change the default with a mouse
click. You can adapt this for any kind of service that you normally use
from your home base, and need access to when you're on the road.

1 Favorite SSH tip: Evading silly web restrictions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The wise assumption is that any public Internet is untrustworthy, so you
can tunnel your Web surfing too. My #1 SSH tip gets you past
untrustworthy networks that might have snoopers, and past any barriers
to unfettered Web-surfing. Just like in tip #2 you need a server that
you control to act as a secure relay; first setup an SSH tunnel to this
server:

::

    carla@hotel:~$ ssh -D 9999 -C carla@homeserver.com

Then configure your Web browser to use port 9999 as a SOCKS 5 proxy.
Figure 1 shows how this looks in Firefox.

Figure 1: Configuring Firefox to use your SSH tunnel as a SOCKS proxy.
An easy way to test this is on your home or business network. Set up the
tunnel to a neighboring PC and surf some external Web sites. When this
works go back and change the SOCKS port number to the wrong number. This
should prevent your Web browser from connecting to any sites, and you'll
know you set up your tunnel correctly. How do you know which port
numbers to use? Port numbers above 1024 do not require root privileges,
so use these on your laptop or whatever you're using in your travels.
Always check /etc/services first to find unassigned ports. The remote
port you're binding to must be a port a server is listening on, and
there has to be a path through your firewall to get to it.

To learn more try the excellent [Pro OpenSSH by Michael Stahnke]
(http://www.apress.com/networking/openssh/9781590594766), and my own
`Linux Networking
Cookbook <http://www.amazon.com/Linux-Networking-Cookbook-Carla-Schroder/dp/0596102488>`__
has more on secure remote administration including SSH, OpenVPN, and
remote graphical sessions, and configuring firewalls.

.. [1]
   `Wikipedia entry
   source <http://en.wikipedia.org/wiki/Secure_Shell>`__


Crypto Keys
-------------

`Source <http://crypto.stackexchange.com/questions/6585/gpg-vs-pgp-vs-openssh-and-management-of-them>`__

::

	What is the main difference of the three? Can I use only one of them for everything
	(e.g. GPG for SSH authentication)

- GnuPG is an free and open-source implementation of the OpenPGP standard.
- Symantec PGP is a proprietary implementation of the OpenPGP standard.
- The OpenPGP standard defines ways to sign and encrypt information (like mail, other documents and code/software).
- OpenSSH is about connection securely to remote computers. For authenticating you need some secret, usually this is a passphrase or SSH key.

With OpenPGP, you hold a secret (private key) which also can be used for authenticating
yourself. It needs software support for that, and I haven't heard of some code doing this
for (Symantec) PGP, `but there is a way doing this with GnuPG <http://budts.be/weblog/2012/08/ssh-authentication-with-your-pgp-key>`__.

::

	If I encrypt my private key with a pass-phrase, is it strong enough so that if someone
	steals my laptop or private key, I'm safe?

Your password encrypts your private key. The key is safe as long as your password is
safe. If your password is too weak (dictionary-attacks, not long enough, easy to
brute-force for other reasons), your key is vulnerable, too.

Think about how valuable your key is for an attacker and choose fitting security measures
like storing your key offline (`in the and of this answer <http://security.stackexchange.com/a/31598/19837>`__).

::

	If not, what about encrypting my private key with the scrypt algorithm?

If doing so, security depends on the password you're using for scrypt and scrypt's algorithm. You can achieve the same amount of security with a good OpenPGP password, so there is no need for additionally encrypting your key.

Fingerprints
--------------

Generate an easier to understand fingerprint (or thumbprint) from a long public key::

	[kevin@Tardis ~]$ ssh-keygen -lf .ssh/test_rsa_key.pub
	2048 d0:4a:98:88:95:65:6e:3c:59:7d:10:db:1d:00:10:40  kevin@tardis.local (RSA)
