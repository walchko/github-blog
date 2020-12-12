---
title: Create a Local Repository
date:   2015-11-28
modified:   2016-07-28
summary:   Create a debian package and setup a repository
---

*Why would you want to do this?* Because the packages on Raspbian are
grossly outdated in most cases. This allows me to compile current
versions of software and easily install/uninstall them without
cluttering up my OS.

Let\'s use node.js as an example. **Note** I used multiple windows in
this, one to create the package/install node and another to run the
server and debug.

Get Software
------------

If you don\'t have it already get:

    apt-get install build-essential

Then grab node.js from the website (current version is 4.4.7):

    wget https://nodejs.org/dist/v4.4.7/node-v4.4.7-linux-armv6l.tar.xz
    tar -xvf node-v4.4.7-linux-armv6l.tar.xz
    cd node-v4.4.7-linux-armv6l

Create Debian Package
---------------------

The basic layout of a debian package is:

    package/
      -- usr/loca/*  (bin, lib, include, etc)
      -- DEBIAN/
          -- changelog
          -- copyright
          -- compat
          -- control
          -- install

The copyright should look like:

    Format: http://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
    Upstream-Name: package
    Upstream-Contact: Name, <email@address>

    Files: *
    Copyright: 2011, Name, <email@address>
    License: (GPL-2+ | LGPL-2 | GPL-3 | whatever)
    Full text of licence.
    .
    Unless there is a it can be found in /usr/share/common-licenses

However, since this is for my own consumption, you don\'t need all of
this to get it to work.

Now inside the `node-v4.4.7-linux-armv6l` directory do:

    mkdir DEBIAN
    mkdir usr
    mkdir usr/local

    cp bin usr/local
    cp lib usr/local
    cp include usr/local
    cp share usr/local

Setup a control file for the package with `pico DEBIAN/control`:

    Package: nodejs
    Version: 4.4.7
    Section: custom
    Priority: optional
    Architecture: all
    Essential: no
    Installed-Size: 1024
    Maintainer: me
    Description: kevin - installs node.js

Now go up one directory:

    cd ..
    dpkg-deb --build node-v4.4.7-linux-armv6l
    mv node.deb node-4.4.7_arm6l.deb

Now, if you chose an architecture other than `arm6l` like `arm7l` then
name the package that.

Test
----

Make sure it works:

    dpkg -i node-4.4.7_arm6l.deb
    node --version
    npm --version

Create Local Repository
-----------------------

Get the node static webserver using `npm`:

    npm install -g http-server

Create the repository:

    cd /some/where/you/want
    mkdir www
    cd www
    mkdir debian
    cp /path/to/node-4.4.7_arm6l.deb www/debian

While still in the `www` directory:

    dpkg-scanpackages debian /dev/null | gzip -9c > debian/Packages.gz

### Start Node

Now move out of `www`:

    cd ..
    http-server ./www

When node runs, it will print out what the server address and port is,
you will need this in the next step. Your repo should have the following
packages and repository info:

    www/
      -- debian/
          -- node-4.4.7_armv6l.deb
          -- Packages.gz

Now obviously, as you add more packages, there will be more here.

### Update apt/source

Tell your system where it is:

    sudo echo "deb http://bender.local:8081 debian/" >> /etc/apt/sources.list
    sudo update

Now a lot of stuff will spin by from node or from the update, but it
should all be fine. We built a simple repository, not a complex with for
different languages/etc. Now double check you can see it:

    apt-cache showpkg nodejs
    Package: nodejs
    Versions:
    4.4.7 (/var/lib/apt/lists/bender.local:8081_debian_Packages) (/var/lib/dpkg/status)
    Description Language:
                     File: /var/lib/apt/lists/bender.local:8081_debian_Packages
                      MD5: 27f92553e61d941b7c6e5440bc637089

    0.10.29~dfsg-2 (/var/lib/apt/lists/mirrordirector.raspbian.org_raspbian_dists_jessie_main_binary-armhf_Packages)
    Description Language:
                     File: /var/lib/apt/lists/mirrordirector.raspbian.org_raspbian_dists_jessie_main_binary-armhf_Packages
                      MD5: e507fb472d7cdaceffc5b285a62d5c1b


    Reverse Depends:
      nodered,nodejs 0.10
      twitter-recess,nodejs
      ruby-passenger,nodejs
      ruby-execjs,nodejs
      ...

Service
-------

`sudo pico /etc/systemd/system/repo.service`:

    [Service]
    ExecStart=/usr/local/bin/http-server /home/pi/www
    Restart=always
    StandardOutput=syslog
    StandardError=syslog
    SyslogIdentifier=repository
    User=pi
    Group=pi
    Environment=NODE_ENV=production

    [Install]
    WantedBy=multi-user.target

Then do:

    sudo systemctl enable repo.service
    sudo systemctl start repo.service

or perhaps easier to remember:

    sudo service repo start
    service repo status

Now you can use `sudo systemctl start|stop|status repo.service` to
start, stop, or find the current status of the server.

You can change the port, by:
`ExecStart=/usr/local/bin/http-server /home/pi/www -p 9000`

Double check things are working by launching a web browser and pointing
it to: <http://host:8081>

References
----------

-   <http://linuxconfig.org/easy-way-to-create-a-debian-package-and-local-package-repository>
-   <http://askubuntu.com/questions/90764/how-do-i-create-a-deb-package-for-a-single-python-script>
