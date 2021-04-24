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
- Flags
    - `-d`: detatch from the command line and run in the background

## Basic Example

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

# Refrences

- devhints.io: [docker-compose cheatsheet](https://devhints.io/docker-compose)
