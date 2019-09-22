---
title: Install and Setup Docker on RPi
date: 22 Sept 2019
---

Docker is a light weight abstraction of a system overlaid onto a
host computer. Various perforance assessments show there is little
perforance lost by using docker compaired to either a VM or KVM.

## Install

1. Get and install: `curl -sSL https://get.docker.com | sudo -E sh`
1. Add user `pi` to the `docker` group: `sudo usermod -aG docker pi`
1. Reboot

- If you need to access GPIO from a docker image: 
    - `docker run --privileged -d mygreatimage`
    - `docker run --device /dev/gpiomem -d mygreatimage`

## Buster Issue

Docker hasn't created packages yet for buster, but you can use the stretch packages and
they appear to work.

```
wget https://download.docker.com/linux/debian/dists/buster/pool/stable/armhf/containerd.io_1.2.6-3_armhf.deb
wget https://download.docker.com/linux/debian/dists/buster/pool/stable/armhf/docker-ce-cli_18.09.7~3-0~debian-buster_armhf.deb
wget https://download.docker.com/linux/debian/dists/buster/pool/stable/armhf/docker-ce_18.09.7~3-0~debian-buster_armhf.deb

dpkg -i containerd.io_1.2.6-3_armhf.deb
dpkg -i docker-ce-cli_18.09.7~3-0~debian-buster_armhf.deb
dpkg -i docker-ce_18.09.7~3-0~debian-buster_armhf.deb
sudo usermod pi -aG docker
```

# References

- [raspberry valley: Docker on RPi](https://raspberry-valley.azurewebsites.net/Docker-on-Raspberry-Pi/)
- [Alexellis fixes for Buster until docker creates packages/scripts](https://blog.alexellis.io/how-to-fix-docker-for-raspbian-buster/)
