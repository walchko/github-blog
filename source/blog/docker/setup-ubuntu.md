---
title: Setup Docker on Ubuntu
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

Make sure the daemon is running:

```
$ service docker status
● docker.service - Docker Application Container Engine
   Loaded: loaded (/lib/systemd/system/docker.service; enabled; vendor preset: e
   Active: active (running) since Sat 2019-09-14 09:27:39 EDT; 34min ago
     Docs: https://docs.docker.com
 Main PID: 19165 (dockerd)
    Tasks: 13
   Memory: 56.6M
   CGroup: /system.slice/docker.service
           └─19165 /usr/bin/dockerd -H fd:// --containerd=/run/containerd/contai

Sep 14 09:27:34 dalek dockerd[19165]: time="2019-09-14T09:27:34.799641714-04:00"
Sep 14 09:27:34 dalek dockerd[19165]: time="2019-09-14T09:27:34.799655354-04:00"
Sep 14 09:27:34 dalek dockerd[19165]: time="2019-09-14T09:27:34.799668573-04:00"
Sep 14 09:27:34 dalek dockerd[19165]: time="2019-09-14T09:27:34.799988416-04:00"
Sep 14 09:27:35 dalek dockerd[19165]: time="2019-09-14T09:27:35.368428381-04:00"
Sep 14 09:27:35 dalek dockerd[19165]: time="2019-09-14T09:27:35.533850622-04:00"
Sep 14 09:27:38 dalek dockerd[19165]: time="2019-09-14T09:27:38.487402589-04:00"
Sep 14 09:27:38 dalek dockerd[19165]: time="2019-09-14T09:27:38.487588431-04:00"
Sep 14 09:27:39 dalek dockerd[19165]: time="2019-09-14T09:27:39.332237091-04:00"
Sep 14 09:27:39 dalek systemd[1]: Started Docker Application Container Engine.
```

To prevent issues with not being able to connect to the daemon, do:

```
sudo usermod -aG docker $(whoami)
```
Note, you may need to log out and back in again.

# References

- [18.04 setup](https://phoenixnap.com/kb/how-to-install-docker-on-ubuntu-18-04)
- [Docker install directions](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
