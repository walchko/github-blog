# Setup a New Raspbian Image for Headless Operation

:date: 27 May 2019

After you burn a Raspbian Image, you should see the `boot` directory on your desktop.

## Enable SSH

```bash
touch <path-to-boot-dir> ssh
```

## Setup Wifi

```bash
pico <path-to-boot-dir>/wpa_supplicant.conf
```

When Raspbian boots, it will copy this file over to `/etc` where it belongs. The should look like:

```bash
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=US
network={
    ssid="YourNetworkSSID"
    psk="password"
    key_mgmt=WPA-PSK
    priority = 1
}
```

# Loggin In

On OSX install `ssh-copy-id` via `brew` and in a terminal window on OSX:

```bash
ssh-copy-id pi@raspberry.local
```

# Issues

## Bypass known_hosts

Since all RPi's hostname are raspberrypi.local, it **sucks** when you try to connect
to a new one and you get the man-in-the-middle attack warning.

You can disable the check with:

```bash
ssh -o UserKnownHostsFile=/dev/null pi@raspberrypi.local
```

## Unknown Interface

Unfortunately the morons that make decisions in the linux world have changed things
and the old `ifup`/`ifdown` stuff is broken. If you make changes to your network
and get an error talking about *unknown interface* with `wlan0` (and maybe others), 
try:

```
sudo service dhcpcd restart
wpa_cli -i wlan0 reconfigure
sudo ip link set wlan0 up
```

This seems to work for me ... at least right now.
