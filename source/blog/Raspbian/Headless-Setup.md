# Setup a New Raspbian Image for Headless Operation

After you burn a Raspbian Image, you should see the `boot` directory on your desktop.

## Enable SSH

  touch <path-to-boot-dir> ssh

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
}
```
