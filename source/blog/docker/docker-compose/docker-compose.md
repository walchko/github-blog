---
title: Docker-Compose Cheatsheet
date: 24 Apr 2021
---

I hate using the `docker` command, it is so complex and difficult. `docker-compose` 
on the hand is very simple and straight forward. 

## Commands

```
docker-compose <cmd> <flag>
```
- Commands
    - `start` | `up`
    - `stop` | `down`
    - `pause` | `unpause`
    - `ps`: print a container's stats
    - `top`: show running containers
- Flags
    - `-d`: detatch from the command line and run in the background

## Basic Interactive `bash` Example

- `docker-compose build`
- `docker-compose run dev`
    - You have to do `docker-compose run` instead of `docker-compose up` if you want an interactive session, because `up` is meant to run multiple containers.

### `docker-compose.yml`

```yaml
version: '3'

services:
  dev:
    build:
      context: .
    container_name: foo
    volumes:
      - ./foo:/src/foo
    stdin_open: true
```
### `Dockerfile`

```yaml
FROM debian:buster

RUN mkdir /src

WORKDIR /src

RUN apt-get update -qq \
    && apt-get install -yq --no-install-recommends \
        build-essential \
        cmake \
        git \
    && apt-get purge -y --auto-remove \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT /bin/bash
```

## Env File

```yaml
# docker-compose.yml
version: '2'

services:
  env_file: .env # can also be an array of files to store variables 
  web:
    build: .
    # build from Dockerfile
    context: ./Path
    dockerfile: Dockerfile
    ports:
     - "5000:5000" # host:container
    volumes:
     - .:/code # host_path:container_path
     - ./etc:/etc
  # environment vars
  environment:
    RACK_ENV: development
  redis:
    image: redis
```

## Non-Root

You can create a user account:

```yaml
RUN useradd -ms /bin/bash bob \
    && chsh --shell /bin/bash bob

USER bob # now everything will be done as bob

COPY --chown=bob:bob some.file .
```

# Refrences

- devhints.io: [docker-compose cheatsheet](https://devhints.io/docker-compose)
- github gist: [simple docker-compose setup](https://gist.github.com/margaret/ac79bbab2234143d08abf605e9eddad5)
