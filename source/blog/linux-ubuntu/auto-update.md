---
title: Stopping the Auto Updater
date: 3 Nov 2019
---

The auto updater is a POS built by morons who can't even code. 
When you tell it to not update in the GUI application, it still
does because the programmers suck!

So here is how to kill this POS:

```
systemctl stop apt-daily.timer
systemctl disable apt-daily.timer
systemctl disable apt-daily.service
systemctl stop apt-daily-upgrade.timer
systemctl disable apt-daily-upgrade.timer
systemctl disable apt-daily-upgrade.service
```
