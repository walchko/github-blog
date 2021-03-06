---
title: Determine Raspbian Version Info
date: 2017-01-07
---

Some of the ways to get info on your system are
  
```
pi@fry ~ $ uname -a
Linux fry 4.9.43-v7+ #1026 SMP Wed Aug 16 22:35:51 BST 2017 armv7l GNU/Linux
```
```
pi@fry ~ $ cat /etc/issue
Raspbian GNU/Linux 9 \n \l
```
```
pi@fry ~ $ cat /etc/os-release
PRETTY_NAME="Raspbian GNU/Linux 9 (stretch)"
NAME="Raspbian GNU/Linux"
VERSION_ID="9"
VERSION="9 (stretch)"
ID=raspbian
ID_LIKE=debian
HOME_URL="http://www.raspbian.org/"
SUPPORT_URL="http://www.raspbian.org/RaspbianForums"
BUG_REPORT_URL="http://www.raspbian.org/RaspbianBugs"
```
```
pi@fry ~ $ lsb_release -da
No LSB modules are available.
Distributor ID:	Raspbian
Description:	Raspbian GNU/Linux 9.1 (stretch)
Release:	9.1
Codename:	stretch
```
```
pi@fry ~ $ hostnamectl
 Static hostname: fry
       Icon name: computer
      Machine ID: 7bece7e7fb154e8d97398443dc8bc1ae
         Boot ID: fdd8c91cbb7148138f7b0e830f626d6b
Operating System: Raspbian GNU/Linux 9 (stretch)
          Kernel: Linux 4.9.43-v7+
    Architecture: arm
```
