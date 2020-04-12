---
title: Lighttpd Static Page Webserver
---

Security, speed, compliance, and flexibility -- all of these describe lighttpd (pron. lighty) which is rapidly redefining efficiency of a webserver; as it is designed and optimized for high performance environments. With a small memory footprint compared to other web-servers, effective management of the cpu-load, and advanced feature set (FastCGI, SCGI, Auth, Output-Compression, URL-Rewriting and many more) lighttpd is the perfect solution for every server that is suffering load problems. And best of all it's Open Source licensed under the revised BSD license.

Website: [Lighttpd](https://www.lighttpd.net/)

## Install

To install and be up and running do:

```bash
sudo apt-get install lighttpd
```

The default config is:

```bash
pi@raspberrypi:~ $ cat /etc/lighttpd/lighttpd.conf
server.modules = (
	"mod_access",
	"mod_alias",
	"mod_compress",
 	"mod_redirect",
)

server.document-root        = "/var/www/html"
server.upload-dirs          = ( "/var/cache/lighttpd/uploads" )
server.errorlog             = "/var/log/lighttpd/error.log"
server.pid-file             = "/var/run/lighttpd.pid"
server.username             = "www-data"
server.groupname            = "www-data"
server.port                 = 80


index-file.names            = ( "index.php", "index.html", "index.lighttpd.html" )
url.access-deny             = ( "~", ".inc" )
static-file.exclude-extensions = ( ".php", ".pl", ".fcgi" )

compress.cache-dir          = "/var/cache/lighttpd/compress/"
compress.filetype           = ( "application/javascript", "text/css", "text/html", "text/plain" )

# default listening port for IPv6 falls back to the IPv4 port
include_shell "/usr/share/lighttpd/use-ipv6.pl " + server.port
include_shell "/usr/share/lighttpd/create-mime.assign.pl"
include_shell "/usr/share/lighttpd/include-conf-enabled.pl"
```

If you navigate to `http://raspberrypi.local` you will be greeted with a default
web page from `/var/www/html` (see above in the config file).

# Service

```bash
pi@raspberrypi:~ $ service lighttpd status
● lighttpd.service - Lighttpd Daemon
   Loaded: loaded (/lib/systemd/system/lighttpd.service; enabled; vendor preset:
   Active: active (running) since Wed 2018-06-13 12:30:41 MST; 10min ago
 Main PID: 1417 (lighttpd)
   CGroup: /system.slice/lighttpd.service
           └─1417 /usr/sbin/lighttpd -D -f /etc/lighttpd/lighttpd.conf

Jun 13 12:30:41 raspberrypi systemd[1]: Starting Lighttpd Daemon...
Jun 13 12:30:41 raspberrypi systemd[1]: Started Lighttpd Daemon.
```

# References

- [Arch Linux Lighttpd page](https://wiki.archlinux.org/index.php/lighttpd)
