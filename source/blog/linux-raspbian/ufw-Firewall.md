---
title: Simple `ufw` Firewall
date: 2016-04-30
---

## Uncomplicated Firewall (ufw)

`iptables` are hard to manage, but `ufw` is an easier to manage firewall
that uses the `iptables`. It is not installed by default, so install the 
firewall with: `sudo apt-get install ufw`. The basic commands are:

```
sudo apt install ufw
sudo ufw enable
sudo ufw [--dry-run] [options] [rule syntax]
sudo ufw allow 22
sudo ufw allow ssh
```

```
pi@docker:~ $ sudo ufw status
Status: active

To                         Action      From
--                         ------      ----
22/tcp                     ALLOW       Anywhere                  
32400                      ALLOW       Anywhere                  
22/tcp (v6)                ALLOW       Anywhere (v6)             
32400 (v6)                 ALLOW       Anywhere (v6)  
```

```
pi@docker:~ $ sudo ufw status verbose
Status: active
Logging: on (low)
Default: deny (incoming), allow (outgoing), deny (routed)
New profiles: skip

To                         Action      From
--                         ------      ----
22/tcp                     ALLOW IN    Anywhere                  
32400                      ALLOW IN    Anywhere                  
22/tcp (v6)                ALLOW IN    Anywhere (v6)             
32400 (v6)                 ALLOW IN    Anywhere (v6)  
```

- **allow:** ex: `sudo ufw allow from <ip addr>`
- **deny:** ex: `sudo ufw deny from <ip addr>`
- **reject**
- **limit**
- **status:** displays if the firewall is active or inactive
- **show raw:** displays the current running rules on your firewall
- **reset:** disables and resets the firewall to default
- **reload:** reloads the current running firewall
- **disable:** disables the firewall and will not start at boot
- **enable:** sets the firewall to start at boot

# Resources:

- [ubuntu ufw help page](https://help.ubuntu.com/community/UFW)
- [Jack Wallen, Linux.com **An Introduction to UFW**](https://www.linux.com/learn/introduction-uncomplicated-firewall-ufw>)
