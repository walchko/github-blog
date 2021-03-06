---
title: Automatically Updating Raspbian with a Service
date: 2017-10-08
---

# My Current Method

I wrote a bash script called `setup-auto-upgrade.sh` (shown below) that:
- creates a service which runs weekly
    - the service calls a script located in the `pi` home directory called
	`autoupgrade.sh` ... you will need to change this location for your system
- updates:
    - the OS using `apt-get`
    - python 2 libraries using `pip`
	- python 3 libraries using `pip3`
	- node libraries using `npm`
- when it is done, it saves a empty file to the `pi` user's home directory
called `system-updated-on-XXX` where XXX is the date the system was updated on

```bash
#!/bin/bash
set -e

# check if we are root
if [ "$EUID" -ne 0 ]
	then echo "Please run as root"
	exit 1
fi

echo ""
echo "============================="
echo "| Setting Up Auto Upgrade   |"
echo "============================="
echo ""

echo "*** setup script ***"

# setup the service
REREAD=""
SCRIPT="static/autoupgrade.sh"
TIMER="/etc/systemd/system/autoupgrade.timer"
SERVICE="/etc/systemd/system/autoupgrade.service"


# if the file exists, remove it ... going to dynamically create it
if [[ -f "${TIMER}" ]]; then
  REREAD="true"
fi

cat <<'EOF' >${SCRIPT}
#!/bin/bash

# check online
wget -q --spider http://google.com

if [ $? -eq 0 ]; then
  echo "*** Online ***"
else
  echo "*** Offline, no access to internet ***"
  return 1
fi

echo "*** upgrade base system ***"
apt-get update
apt-get upgrade -y
apt-get autoclean

echo "*** upgrade python 2 ***"
pip list --format=legacy --outdated | cut -d' ' -f1 | xargs pip install --upgrade

echo "*** upgrade python 3 ***"
pip3 list --format=legacy --outdated | cut -d' ' -f1 | xargs pip3 install --upgrade

echo "*** update npm packages ***"
npm i -g npm
npm update -g

# remove the old file, so we don't get a million of them
UFILE=`find /home/pi -name 'system-updated-on*'`
if [[ ! -z "${UFILE}" ]]; then
  rm ${UFILE}
fi

# let people know we upgraded things
FNAME=`date | { read x; echo "${x// /-}"; }`
touch "/home/pi/system-updated-on-${FNAME}"
chown pi:pi "/home/pi/system-updated-on-${FNAME}"

chown -R pi:pi /usr/local

exit 0
EOF

# fix permissions
chmod 755 ${SCRIPT}

echo "*** setup timer ***"

cat<<'EOF' > ${TIMER}
[Unit]
Description=Weekly update

[Timer]
OnCalendar=weekly
Persistent=true

[Install]
WantedBy=timers.target
EOF

echo "*** setup service ***"

cat <<EOF > ${SERVICE}
[Unit]
Description=Updates system weekly
Wants=audoupgrade.timer

[Service]
Type=simple
ExecStart=/home/pi/github/mote/software/static/autoupgrade.sh
EOF

if [[ -z "${REREAD}" ]]; then
  echo "*** enabling/starting timer and service ***"
  systemctl start autoupgrade.timer
  systemctl enable autoupgrade.timer
  systemctl start autoupgrade.service
  echo " to see timers, run: sudo systemctl list-timers --all"
  echo " to see output, run: sudo journalctl -u autoupgrade"
else
  echo "*** need to reload service due to changes ***"
  systemctl daemon-reload
  systemctl start autoupgrade.timer
  systemctl enable autoupgrade.timer
  systemctl start autoupgrade.service
fi

echo ""
echo "*** $0 Done ***"
echo ""
```

# The Old Way

## Keeping Things Updated


- [Cron ref](https://help.ubuntu.com/community/AutoWeeklyUpdateHowTo)

Basically, running the below script as root, it will:

- setup a cron weekly job
- update the os, python, and node
- place an empty file in pi's home directory with time/date stamp saying when the update ran

```bash

	#!/bin/bash
	set -e

	# check if we are root
	if [ "$EUID" -ne 0 ]
		then echo "Please run as root"
		exit 1
	fi

	echo ""
	echo "============================="
	echo "| Setting Up Auto Upgrade   |"
	echo "============================="
	echo ""

	# setup the service
	AUTOUPGRADE_FILE="/etc/cron.weekly/autoupgrade.sh"

	# if the file exists, remove it ... going to dynamically create it
	if [[ -f "${AUTOUPGRADE_FILE}" ]]; then
		rm ${AUTOUPGRADE_FILE}
	fi

	cat <<EOF >${AUTOUPGRADE_FILE}
	#!/bin/bash

	echo "*** upgrade base system ***"
	apt-get update
	apt-get upgrade -y
	apt-get autoclean

	echo "*** upgrade python 2 ***"
	pip list --outdated | cut -d' ' -f1 | xargs pip install --upgrade

	echo "*** upgrade python 3 ***"
	pip3 list --outdated | cut -d' ' -f1 | xargs pip3 install --upgrade

	echo "*** update npm packages ***"
	npm install npm@latest -g

	# remove the old file, so we don't get a million of them
	FILE=`find /home/pi -name 'system-update-on*'`
	if [[ ! -z "${FILE}" ]]; then
		rm ${FILE}
	fi

	# let people know we upgraded things
	FNAME=`date | { read x; echo "${x// /-}"; }`
	touch "/home/pi/system-updated-on-${FNAME}"

	exit 0
	EOF

	# fix permissions
	chmod 755 ${AUTOUPGRADE_FILE}
```
