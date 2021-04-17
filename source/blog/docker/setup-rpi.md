---
title: Setup Docker on RPi
date: 22 Sept 2019
---

Docker is a light weight abstraction of a system overlaid onto a
host computer. Various perforance assessments show there is little
perforance lost by using docker compaired to either a VM or KVM.

## Install

1. Get and install: `curl -sSL https://get.docker.com | sudo -E sh`
1. Add user `pi` to the `docker` group: `sudo usermod -aG docker pi`
1. Log out and back in again
1. To test your setup, do: `docker container run hello-world`
    ```pi@raspberrypi:~ $ docker container run hello-world
    Unable to find image 'hello-world:latest' locally
    latest: Pulling from library/hello-world
    4ee5c797bcd7: Pull complete 
    Digest: sha256:9572f7cdcee8591948c2963463447a53466950b3fc15a247fcad1917ca215a2f
    Status: Downloaded newer image for hello-world:latest

    Hello from Docker!
    This message shows that your installation appears to be working correctly.

    To generate this message, Docker took the following steps:
     1. The Docker client contacted the Docker daemon.
     2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
        (arm32v7)
     3. The Docker daemon created a new container from that image which runs the
        executable that produces the output you are currently reading.
     4. The Docker daemon streamed that output to the Docker client, which sent it
        to your terminal.

    To try something more ambitious, you can run an Ubuntu container with:
     $ docker run -it ubuntu bash

    Share images, automate workflows, and more with a free Docker ID:
     https://hub.docker.com/

    For more examples and ideas, visit:
     https://docs.docker.com/get-started/
    ```


- If you need to access GPIO from a docker image: 
    - `docker run --privileged -d mygreatimage`
    - `docker run --device /dev/gpiomem -d mygreatimage`

## Non-Privileged User

To run Docker as a non-privileged user, consider setting up the
Docker daemon in rootless mode for your user:

```
dockerd-rootless-setuptool.sh install
```

Visit https://docs.docker.com/go/rootless/ to learn about rootless mode.

## Fully Privileged Service

To run the Docker daemon as a fully privileged service, but granting non-root
users access, refer to https://docs.docker.com/go/daemon-access/


**WARNING:** Access to the remote API on a privileged Docker daemon is equivalent
         to root access on the host. Refer to the 'Docker daemon attack surface'
         documentation for details: https://docs.docker.com/go/attack-surface/


## Buster Issue [fixed?]

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

- [Linuxize: install docker on pi](https://linuxize.com/post/how-to-install-and-use-docker-on-raspberry-pi/)
- [raspberry valley: Docker on RPi](https://raspberry-valley.azurewebsites.net/Docker-on-Raspberry-Pi/)
- [Alexellis fixes for Buster until docker creates packages/scripts](https://blog.alexellis.io/how-to-fix-docker-for-raspbian-buster/)
