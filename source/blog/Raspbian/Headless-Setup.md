# Setup a New Raspbian Image for Headless Operation

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

## Bypass known_hosts

Since all RPi's hostname are raspberrypi.local, it **sucks** when you try to connect
to a new one and you get the man-in-the-middle attack warning.

You can disable the check with:

```bash
	ssh -o UserKnownHostsFile=/dev/null pi@raspberrypi.local
```
