---
title: Debian SystemV Services
date: 2015-11-29
---

To have a service `archeyjs` run at boot and reload if it goes down, create
a file `/etc/systemd/system/archeyjs.service`:


```
[Service]
ExecStart=/usr/local/bin/archeyjs
Restart=always
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=archeyjs
User=pi
Group=pi
Environment=NODE_ENV=production

[Install]
WantedBy=multi-user.target
```

Then do:

	sudo systemctl enable archeyjs
	sudo systemctl start archeyjs

Now you can use `sudo systemctl start|stop|status archeyjs.service` to start,
stop, or find the current status of the server.

	pi@zoidberg:~/tmp/node $ systemctl status archeyjs
	● archeyjs.service
	   Loaded: loaded (/etc/systemd/system/archeyjs.service; enabled)
	   Active: active (running) since Fri 2016-12-30 22:54:10 MST; 6s ago
	 Main PID: 2873 (node)
	   CGroup: /system.slice/archeyjs.service
		   └─2873 node /usr/bin/archeyjs

	Dec 30 22:54:10 zoidberg systemd[1]: Started archeyjs.service.

If you make any changes to the service file, you need to reload it:

	sudo systemctl daemon-reload

You can also do:

	pi@zoidberg ~ $ service archeyjs status
	● archeyjs.service
	   Loaded: loaded (/etc/systemd/system/archeyjs.service; enabled)
	   Active: active (running) since Fri 2016-12-30 22:54:10 MST; 6 days ago
	 Main PID: 2873 (node)
	   CGroup: /system.slice/archeyjs.service
	           └─2873 node /usr/bin/archeyjs



	pi@zoidberg ~ $ service bluetooth status
	● bluetooth.service - Bluetooth service
	   Loaded: loaded (/lib/systemd/system/bluetooth.service; enabled)
	   Active: active (running) since Fri 2016-12-30 20:35:36 MST; 1 weeks 0 days ago
	     Docs: man:bluetoothd(8)
	 Main PID: 586 (bluetoothd)
	   Status: "Running"
	   CGroup: /system.slice/bluetooth.service
	           └─586 /usr/lib/bluetooth/bluetoothd


# References

- [thegeekstuff](http://www.thegeekstuff.com/2012/03/lsbinit-script/)
- [debian-administration.org](https://www.debian-administration.org/article/28/Making_scripts_run_at_boot_time_with_Debian)
