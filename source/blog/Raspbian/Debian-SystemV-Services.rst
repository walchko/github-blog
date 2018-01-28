
Services
========

:date: 2015-11-29
:modified: 2016-12-30
:summary: Setting up services on Raspbian/Jessie

To have a service ``archeyjs`` run at boot and reload if it goes down, create
a file ``/etc/systemd/system/archeyjs.service``::

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

Then do::

	sudo systemctl enable archeyjs
	sudo systemctl start archeyjs

Now you can use ``sudo systemctl start|stop|status archeyjs.service`` to start,
stop, or find the current status of the server.

::

	pi@zoidberg:~/tmp/node $ systemctl status archeyjs
	● archeyjs.service
	   Loaded: loaded (/etc/systemd/system/archeyjs.service; enabled)
	   Active: active (running) since Fri 2016-12-30 22:54:10 MST; 6s ago
	 Main PID: 2873 (node)
	   CGroup: /system.slice/archeyjs.service
		   └─2873 node /usr/bin/archeyjs

	Dec 30 22:54:10 zoidberg systemd[1]: Started archeyjs.service.

If you make any changes to the service file, you need to reload it::

	sudo systemctl daemon-reload

You can also do::

	pi@zoidberg ~ $ service archeyjs status
	● archeyjs.service
	   Loaded: loaded (/etc/systemd/system/archeyjs.service; enabled)
	   Active: active (running) since Fri 2016-12-30 22:54:10 MST; 6 days ago
	 Main PID: 2873 (node)
	   CGroup: /system.slice/archeyjs.service
	           └─2873 node /usr/bin/archeyjs

::

	pi@zoidberg ~ $ service bluetooth status
	● bluetooth.service - Bluetooth service
	   Loaded: loaded (/lib/systemd/system/bluetooth.service; enabled)
	   Active: active (running) since Fri 2016-12-30 20:35:36 MST; 1 weeks 0 days ago
	     Docs: man:bluetoothd(8)
	 Main PID: 586 (bluetoothd)
	   Status: "Running"
	   CGroup: /system.slice/bluetooth.service
	           └─586 /usr/lib/bluetooth/bluetoothd

**This has changed with Jessie, don't use anything below with current versions of Raspbian**

------

To setup daemons to run at boot time or be able to easily start/stop
them, we need to the init system. Create a file in ``/etc/init.d`` like
the one below:

::

    # /etc/init.d/netscan
    #

    # Some things that run always
    DAEMON_USER=root
    DIR=/home/pi/github/netscan
    DAEMON_NAME=netscan
    SERVER_NAME=simple_server
    DAEMON=$DIR/$DAEMON_NAME.py
    SERVER=$DIR/$SERVER_NAME.py
    PIDFILE=/var/run/$DAEMON_NAME.pid
    SERVER_PIDFILE=/var/run/$SERVER_NAME.pid

    . /lib/lsb/init-functions

    # Carry out specific functions when asked to by the system
    case "$1" in
      start)
        echo "Starting netscan"
        log_daemon_msg "Starting system $DAEMON_NAME daemon"
        start-stop-daemon --start --background --pidfile $PIDFILE --make-pidfile --u
    ser $DAEMON_USER --chuid $DAEMON_USER --startas $DAEMON
        start-stop-daemon --start --background --pidfile $SERVER_PIDFILE --make-pidf
    ile --user pi --chuid pi --startas $SERVER
        log_end_msg $?
        ;;
      stop)
        log_daemon_msg "Stopping system $DAEMON_NAME daemon"
        start-stop-daemon --stop --pidfile $PIDFILE --retry 10
        start-stop-daemon --stop --pidfile $SERVER_PIDFILE --retry 10
        log_end_msg $?
        ;;
      status)
        status_of_proc $SERVER_NAME $SERVER && status_of_proc $DAEMON_NAME $DAEMON &
    & exit 0 || exit $?
        ;;
      *)
        echo "Usage: /etc/init.d/netscan {start|status|stop}"
        exit 1
        ;;
    esac

    exit 0

Another example:

::

    # /etc/init.d/nodesjs
    #

    # Some things that run always
    DAEMON_USER=root
    DIR=/usr/local/bin
    DAEMON_NAME=http-server
    DAEMON=$DIR/$DAEMON_NAME
    PIDFILE=/var/run/$DAEMON_NAME.pid
    DAEMON_full="$DAEMON -- /mnt/usbdrive -p 9000 -s"

    . /lib/lsb/init-functions

    # Carry out specific functions when asked to by the system
    case "$1" in
      start)
            echo "Starting Nodejs HTTP Server for movies"
            echo $DAEMON_full
            log_daemon_msg "Starting system $DAEMON_NAME daemon"
            start-stop-daemon --start --background --pidfile $PIDFILE --make-pidfile --user $DAEMON_USER --chuid $DAEMON_USER --startas $DAEMON_full
            log_end_msg $?
            ;;
      stop)
            log_daemon_msg "Stopping system $DAEMON_NAME daemon"
            start-stop-daemon --stop --pidfile $PIDFILE --retry 10
            log_end_msg $?
            ;;
      status)
            status_of_proc status_of_proc $DAEMON_NAME $DAEMON && exit 0 || exit $?
            ;;
      *)
            echo "Usage: /etc/init.d/nodejs-movies {start|status|stop}"
            exit 1
            ;;
    esac

    exit 0

Change the permissions with:

::

    chmod 755 /etc/init.d/netscan

Add the service to the proper run levels:

::

    update-rc.d netscan defaults

References
----------

-  `thegeekstuff <http://www.thegeekstuff.com/2012/03/lsbinit-script/>`__
-  `debian-administration.org <https://www.debian-administration.org/article/28/Making_scripts_run_at_boot_time_with_Debian>`__
