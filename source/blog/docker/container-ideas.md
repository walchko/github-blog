---
title: Container Ideas
date: 14 Sept 2019
---

Here are some some cool containers to play with:

- [plex media server](https://github.com/plexinc/pms-docker)[*works*]
- [ros: robotic operating system](http://wiki.ros.org/docker/Tutorials/Docker)[*works*]
- [pihole: ad/tracker blocker](https://github.com/pi-hole/docker-pi-hole) [*works*]
    - difficulty getting it to work Ubuntu, but works with some changes
- [droppy: nodejs web based fileserver](https://github.com/silverwind/droppy)
    - not sure how useful this is ... samba is really all I need
- [cloudcmd: nodejs web based fileserver](https://github.com/coderaiser/cloudcmd)[*works*]
    - not sure how useful this is ... samba is really all I need
- [octoprint: 3d printing](https://github.com/OctoPrint/docker)[*works*]
- snort: intruder detection

For the containers I have tested, I noted with *works*, but my testing was rather simple.

My uses are:

- Cross compiling OpenCV and building debian packages
    - this is really slow ... not sure it is any faster
    - trying to get [this working](https://github.com/MomsFriendlyRobotCompany/dpkg_opencv) better
- Octoprint
- pi-hole
