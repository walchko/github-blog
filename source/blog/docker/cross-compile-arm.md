---
title: Cross Compiling ARM Binaries for Raspberry Pi
date: 21 Sept 2019
---

Steps:

1. Enable ARM emulation with: `docker run --rm --privileged hypriot/qemu-register`
1. Run an ARM image with: `docker run --rm -it balenalib/raspberrypi3-debian:latest bash`
1. At the bash prompt do: 
    ```
    $ uname -a
    Linux ffc27f604ee6 4.14.29-1-lts #1 SMP Wed Mar 21 17:41:42 CET 2018 armv7l GNU/Linux
    
    $ cat /etc/os-release
    PRETTY_NAME="Debian GNU/Linux 10 (buster)"
    NAME="Debian GNU/Linux"
    VERSION_ID="10"
    VERSION="10 (buster)"
    VERSION_CODENAME=buster
    ID=debian
    HOME_URL="https://www.debian.org/"
    SUPPORT_URL="https://www.debian.org/support"
    BUG_REPORT_URL="https://bugs.debian.org/"
    ```

The `balenalib/raspberrypi3-debian:latest` is a stripped down image, no python or gcc. The
`balenalib/raspberrypi3-debian-build` has a lot of development tools/libries installed.

## Image Naming

```
balenalib/<hw>-<distro>-<lang_stack>:<lang_ver>-<distro_ver>-(build|run)
balenalib/i386-ubuntu-python:latest-trusty-build
balenalib/raspberrypi3-debian-python:buster-build
```

- *hw*: raspberrypi3
- *distro*: defaults to debian
- *lang_stack*: python
- *lang_ver*: 3.7.4
- *distro_ver*: buster
- type
    - *build*: fatter image with building tools and libraries like `build-essential` and `gcc`
    - *run*: light weight version with minimal packages

- [**raspberry pi 3**(armv7hf)](https://github.com/balena-os/balena-raspberrypi)
- [**raspberry pi 4**(aarch64)](https://github.com/balena-os/balena-raspberrypi)

```docker
FROM balenalib/raspberrypi3-debian-node:10.10-stretch-build as build
RUN npm install --only=production  i2c

# The run time container that will go to devices
FROM balenalib/raspberrypi3-debian-node:10.10-stretch-run

# Grab our node modules for the build step
COPY --from=build ./node_modules ./node_modules
COPY main.js main.js

CMD ["node", "main.js"]
```

# Example

```docker
FROM resin/armv7hf-debian-qemu

RUN [ "cross-build-start" ]

RUN apt-get update && apt-get install -y \
		autoconf \
		build-essential \
		ca-certificates \
		curl \
		imagemagick \
		libbz2-dev \
		libcurl4-openssl-dev \
		libevent-dev \
		libffi-dev \
		libglib2.0-dev \
		libjpeg-dev \
		libmagickcore-dev \
		libmagickwand-dev \
		libmysqlclient-dev \
		libncurses-dev \
		libpq-dev \
		libreadline-dev \
		libsqlite3-dev \
		libssl-dev \
		libxml2-dev \
		libxslt-dev \
		libyaml-dev \
		zlib1g-dev \
	&& rm -rf /var/lib/apt/lists/*

# remove several traces of debian python
RUN apt-get purge -y python.*

# http://bugs.python.org/issue19846
# > At the moment, setting "LANG=C" on a Linux system *fundamentally breaks Python 3*, and that's not OK.
ENV LANG C.UTF-8

# import gpg keys
RUN gpg --keyserver ha.pool.sks-keyservers.net --recv-keys C01E1CAD5EA2C4F0B8E3571504C367C218ADD4FF

# key 63C7CC90: public key "Simon McVittie <smcv@pseudorandom.co.uk>" imported
RUN gpg --keyserver keyring.debian.org --recv-keys 4DE8FF2A63C7CC90

# key 3372DCFA: public key "Donald Stufft (dstufft) <donald@stufft.io>" imported
RUN gpg --keyserver pgp.mit.edu  --recv-key 6E3CBCE93372DCFA

ENV PYTHON_VERSION 2.7.10

# if this is called "PIP_VERSION", pip explodes with "ValueError: invalid truth value '<VERSION>'"
ENV PYTHON_PIP_VERSION 7.1.2

ENV SETUPTOOLS_SHA256 4846755f18c0528d87583342d5e1221052858ce9922c5c38acbadd5015bd683d
ENV SETUPTOOLS_VERSION 18.5

RUN set -x \
	&& mkdir -p /usr/src/python \
	&& curl -SL "https://www.python.org/ftp/python/${PYTHON_VERSION%%[a-z]*}/Python-$PYTHON_VERSION.tar.xz" -o python.tar.xz \
	&& curl -SL "https://www.python.org/ftp/python/${PYTHON_VERSION%%[a-z]*}/Python-$PYTHON_VERSION.tar.xz.asc" -o python.tar.xz.asc \
	&& gpg --verify python.tar.xz.asc \
	&& tar -xJC /usr/src/python --strip-components=1 -f python.tar.xz \
	&& rm python.tar.xz* \
	&& cd /usr/src/python \
	&& ./configure --enable-shared --enable-unicode=ucs4 \
	&& make -j$(nproc) \
	&& make install \
	&& ldconfig \
	&& mkdir -p /usr/src/python/setuptools \
	&& curl -SLO https://pypi.python.org/packages/source/s/setuptools/setuptools-$SETUPTOOLS_VERSION.tar.gz \
	&& echo "$SETUPTOOLS_SHA256  setuptools-$SETUPTOOLS_VERSION.tar.gz" > setuptools-$SETUPTOOLS_VERSION.tar.gz.sha256sum \
	&& sha256sum -c setuptools-$SETUPTOOLS_VERSION.tar.gz.sha256sum \
	&& tar -xzC /usr/src/python/setuptools --strip-components=1 -f setuptools-$SETUPTOOLS_VERSION.tar.gz \
	&& cd /usr/src/python/setuptools \
	&& python2 ez_setup.py \
	&& mkdir -p /usr/src/python/pip \
	&& curl -SL "https://pypi.python.org/packages/source/p/pip/pip-$PYTHON_PIP_VERSION.tar.gz" -o pip.tar.gz \
	&& curl -SL "https://pypi.python.org/packages/source/p/pip/pip-$PYTHON_PIP_VERSION.tar.gz.asc" -o pip.tar.gz.asc \
	&& gpg --verify pip.tar.gz.asc \
	&& tar -xzC /usr/src/python/pip --strip-components=1 -f pip.tar.gz \
	&& rm pip.tar.gz* \
	&& cd /usr/src/python/pip \
	&& python2 setup.py install \
	&& cd .. \
	&& find /usr/local \
		\( -type d -a -name test -o -name tests \) \
		-o \( -type f -a -name '*.pyc' -o -name '*.pyo' \) \
		-exec rm -rf '{}' + \
	&& cd / \
	&& rm -rf /usr/src/python

RUN [ "cross-build-end" ]  
```

# References

- [example: cross-compile docker python arm on x86](https://github.com/petrosagg/armv7hf-python-dockerhub)
- [balena.io naming convention](https://www.balena.io/docs/reference/base-images/base-images/)
- [balena.io image types](https://www.balena.io/docs/reference/base-images/devicetypes/)
- [hypriot blog post](https://blog.hypriot.com/post/docker-intel-runs-arm-containers/)
- [hypriot qemu github repo](https://github.com/hypriot/qemu-register)
- [balenalib armv7hf debian image](https://hub.docker.com/r/balenalib/armv7hf-debian)
- [balenalib armv7hf ubuntu image](https://hub.docker.com/r/balenalib/armv7hf-ubuntu)
- [another way to do it using qemu](https://github.com/petrosagg/armv7hf-python-dockerhub/blob/master/Dockerfile)
- [cross compile opencv](https://github.com/sgtwilko/rpi-raspbian-opencv)
- [live docker prompt](https://www.katacoda.com/contino/courses/docker/basics)
- [ros docker and qemu blog post](https://discourse.ros.org/t/announcing-ros-docker-images-for-arm-and-debian/2467)
