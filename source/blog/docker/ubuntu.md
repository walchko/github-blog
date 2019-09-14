---
title: Ubuntu Setup
date: 14 Sept 2019
---

The offical instructions are [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) 
for the Community Edition (CE). The basics are:

1. `sudo apt update`
1. Remove old version: `sudo apt remove docker docker-engine docker.io containerd runc`
1. Get neede apts: `sudo apt-get install apt-transport-https ca-certificates curl software-properties-common`
    1. **apt-transport-https:** Allows the package manager to transfer files and data over https
    1. **ca-certificates:** Allows the system (and web browser) to check security certificates
    1. **curl:** This is a tool for transferring data
    1. **software-properties-common:** Adds scripts for managing software
1. Get key: `curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add –`
1. Add repo: `sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu  $(lsb_release -cs)  stable"`
1. `sudo apt update`
1. Install latest version: `sudo apt install docker-ce docker-ce-cli containerd.io`
    1. Apparently there were some name changes, docker is now `docker-ce`
    
# References

- [18.04 setup](https://phoenixnap.com/kb/how-to-install-docker-on-ubuntu-18-04)
- [Docker install directions](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
