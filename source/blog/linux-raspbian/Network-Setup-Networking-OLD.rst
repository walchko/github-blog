
Networking
====================

:date: 2015-07-23
:summary: Setting up our network

WiFi
----

.. figure:: pics/wifi.png
	:width: 200px
	:align: center

D-Link wireless N 150 (DWA-121) Pico USB adaptor install.

::

    [kevin@raspberrypi ~]$ lsusb
    Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    Bus 001 Device 002: ID 0424:9512 Standard Microsystems Corp.
    Bus 001 Device 003: ID 0424:ec00 Standard Microsystems Corp.
    Bus 001 Device 004: ID 2001:3308 D-Link Corp. DWA-121 802.11n Wireless N 150 Pico Adapter [Realtek RTL8188CUS]

**Note:** If you don't see it, make sure it is the only USB device
plugged in because it takes a lot of power. Otherwise attach to a
powered USB hub and you should be fine.

WPA2
~~~~~

First create a file with the following information, but substitute in
the correct ssid and psk (with quotes around them) for your network.

::

    [kevin@raspberrypi ~]$ more /etc/wpa_supplicant/wpa_supplicant.conf
    ctrl_interface=/var/run/wpa_supplicant
    ctrl_interface_group=0
    ap_scan=2

    network={
        ssid="wireless access point name in quotes"
        key_mgmt=WPA-PSK
        proto=WPA2
        pairwise=CCMP TKIP
        group=CCMP TKIP
        psk="pass phrase in quotes"
    }

**Note:** The ssid is case sensitive!!

Static IP Address
~~~~~~~~~~~~~~~~~~~~

If you will need to change your network interface for a static IP to:

::

    [kevin@raspberrypi ~]$ more /etc/network/interfaces
    auto lo
    iface lo inet loopback

    # dynamic interface
    #iface eth0 inet dhcp

    # static interface
    iface eth0 inet static
        address 192.168.1.120
        netmask 255.255.255.0
        gateway 192.168.1.1

    auto wlan0
    iface wlan0 inet static
        address 192.168.1.121
        wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
        netmask 255.255.255.0
        gateway 192.168.1.1

Note that the wifi interface (wlan0) points to the WPA config file from
above. Also there is an example dynamic interface commented out
(``iface eth0 inet dhcp``) to show you how to use DHCP. The ``lo`` is
the loopback interface, eth0 is the wired interface, with the wlan0
being the wireless interface. Also, the lines with ``auto`` in them tell
linux to automatically start those interfaces during the bootup process.

Dynamic IP Address
~~~~~~~~~~~~~~~~~~~~

If you are fine with DHCP determining all your IP addresses:

::

    auto lo

    iface lo inet loopback
    iface eth0 inet dhcp

    allow-hotplug wlan0
    iface wlan0 inet manual
    wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
    iface default inet dhcp

Finally to get the wireless up and running, use ``ifup`` to get things
started.

::

    [kevin@raspberrypi ~]$ sudo ifup wlan0
    ioctl[SIOCSIWAP]: Operation not permitted
    ioctl[SIOCSIWENCODEEXT]: Invalid argument
    ioctl[SIOCSIWENCODEEXT]: Invalid argument

These errors don't seem to effect the wifi adaptor. You can double check
all is well by using ``iwconfig``.

::

    [kevin@raspberrypi ~]$ iwconfig
    lo        no wireless extensions.

    wlan0     IEEE 802.11bgn  ESSID:"GC9J2"  Nickname:"<WIFI@REALTEK>"
              Mode:Managed  Frequency:2.437 GHz  Access Point: 00:7F:28:05:4D:D9
              Bit Rate:150 Mb/s   Sensitivity:0/0
              Retry:off   RTS thr:off   Fragment thr:off
              Power Management:off
              Link Quality=100/100  Signal level=76/100  Noise level=0/100
              Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
              Tx excessive retries:0  Invalid misc:0   Missed beacon:0

    eth0      no wireless extensions.

Looking at the wlan0 interface, it has a 150 Mb/s data rate (802.11n),
and sees a signal strength of 76/100.

::

    [kevin@raspberrypi ~]$ ifconfig wlan0
    wlan0     Link encap:Ethernet  HWaddr fc:75:16:04:96:5f
              inet addr:192.168.1.121  Bcast:192.168.1.255  Mask:255.255.255.0
              UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
              RX packets:59222 errors:0 dropped:63403 overruns:0 frame:0
              TX packets:11365 errors:0 dropped:0 overruns:0 carrier:0
              collisions:0 txqueuelen:1000
              RX bytes:92009000 (87.7 MiB)  TX bytes:1154992 (1.1 MiB)

Notice here a lot of dropped packets on the receive (RX).
