Updating Raspbian
============================

:date: 2017-10-08
:summary: Keeping raspbian updated

Keeping Things Updated
------------------------

- `Cron ref <https://help.ubuntu.com/community/AutoWeeklyUpdateHowTo>`_

Basically, running the below script as root, it will:

- setup a cron weekly job
- update the os, python, and node
- place an empty file in pi's home directory with time/date stamp saying when the update ran

.. code-block:: bash

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
