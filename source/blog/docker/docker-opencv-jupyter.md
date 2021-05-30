---
title: Setting Up a Docker Container with OpenCV and Jupyter Notebooks
date: 4 Mar 2017
---

![](pics/docker.png)

Here is a `Dockerfile` to build a container with opencv, python, and jupyter:

```dockerfile
# Installs Jupyter Notebook and IPython kernel from the current branch
# Another Docker container should inherit with `FROM jupyter/notebook`
# to run actual services.
#
# For opinionated stacks of ready-to-run Jupyter applications in Docker,
# check out docker-stacks <https://github.com/jupyter/docker-stacks>
#
# build: docker build -t rpi .
# run: docker run -ti -p 8888:8888  -v `pwd`/notebooks:/notebooks rpi /bin/bash
#
# https://www.devopsskills.info/docker/docker-remove-none-images/
# docker images | egrep "^<none>" | awk '{print $3}' | xargs docker rmi -f

FROM debian:stretch

# Not essential, but wise to set the lang
# Note: Users with other languages should set this in their derivative image
ENV LANGUAGE en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV PYTHONIOENCODING UTF-8

# Python binary and source dependencies
RUN apt-get update -qq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        git \
        htop \
        nodejs \
        libcurl4-openssl-dev \
        libffi-dev \
        libsqlite3-dev \
        libzmq3-dev \
        nodejs \
        pandoc \
        python \
        python3 \
        python-dev \
        python3-dev \
        sqlite3 \
        texlive-fonts-recommended \
        texlive-latex-base \
        texlive-latex-extra \
        wget \
        zlib1g-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install the recent pip release
RUN curl -O https://bootstrap.pypa.io/get-pip.py && \
    python2 get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py && \
    pip2 --no-cache-dir install requests[security] && \
    pip3 --no-cache-dir install requests[security] && \
    rm -rf /root/.cache

# Install some dependencies.
RUN pip2 --no-cache-dir install jupyter numpy matplotlib && \
    pip3 --no-cache-dir install jupyter numpy matplotlib && \
    \
    rm -rf /root/.cache

# Move notebook contents into place.
#ADD . /usr/src/jupyter-notebook
ADD ./.bashrc /root
ADD ./hostname /etc

#pip3 install --no-cache-dir /usr/src/jupyter-notebook && \
#pip2 install --no-cache-dir widgetsnbextension && \
#pip3 install --no-cache-dir widgetsnbextension && \

# Install dependencies and run tests.
#RUN BUILD_DEPS="nodejs" && \
RUN apt-get update -qq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq $BUILD_DEPS && \
    \
    apt-get clean && \
    rm -rf /root/.cache && \
    rm -rf /root/.config && \
    rm -rf /root/.local && \
    rm -rf /root/tmp && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get purge -y --auto-remove \
        -o APT::AutoRemove::RecommendsImportant=false -o APT::AutoRemove::SuggestsImportant=false $BUILD_DEPS

# Run tests.
#RUN pip3 install --no-cache-dir notebook[test] && nosetests -v notebook

# Add a notebook profile.
RUN mkdir -p -m 700 /root/.jupyter/ && \
    echo "c.NotebookApp.ip = '*'" >> /root/.jupyter/jupyter_notebook_config.py

VOLUME /notebooks
WORKDIR /notebooks

EXPOSE 8888

CMD ["jupyter", "notebook", "--no-browser", "--allow-root"]
```
