---
title: Running Pi-Hole Docker
date: 7 Mar 2020
image: "https://i.pinimg.com/564x/1c/8b/fd/1c8bfd2b80080c4bbe20feae282f5d31.jpg"
---

So I run pi-hole on my Ubuntu desktop in a docker container and a second pi-hole on
a Raspberry Pi Zero (connected to my network via USB ethernet) for backup.

Official [directions](https://github.com/pi-hole/docker-pi-hole) are here, pay close attention
to the Ubuntu steps if running it on that linux OS.

## Ubuntu

- Edit `/etc/systemd/resolved.conf` by uncommenting/changing the stub line to: `DNSStubListener=no`
- Redirect the `/etc/resolv.conf` (`/run/systemd/resolve/stub-resolv.conf`) link by: `ln -s /run/systemd/resolve/resolv.conf /etc/resolv.conf`

## Docker-Compose

```yaml
version: "3"

# More info at https://github.com/pi-hole/docker-pi-hole/ and https://docs.pi-hole.net/
services:
  pihole:
    container_name: pihole
    image: pihole/pihole:latest
    ports:
      - "53:53/tcp"
      - "53:53/udp"
      # - "67:67/udp"  # dhcp
      - "80:80/tcp"
      - "443:443/tcp"
    environment:
      TZ: 'America/Chicago'
      VIRTUAL_HOST: pihole-docker.local
      ServerIP: 10.0.1.154
      WEBPASSWORD: 'kevin'
    # Volumes store your data between container upgrades
    volumes:
      - './etc-pihole/:/etc/pihole/'
      - './etc-dnsmasq.d/:/etc/dnsmasq.d/'
    dns:
      - 127.0.0.1
      - 10.0.1.200
    # Recommended but not required (DHCP needs NET_ADMIN)
    #   https://github.com/pi-hole/docker-pi-hole#note-on-capabilities
    cap_add:
      - NET_ADMIN
    restart: unless-stopped
```
