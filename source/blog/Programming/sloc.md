SLOC
======

:date: 25-01-2018

Software lines of code (sloc) is a measure of how large a program is

	brew install cloc


```bash
#!/usr/bin/env bash

set -e

loc () {
	echo "=========================================="
    echo "   $1"
    echo "=========================================="

	TMP=/var/tmp/temp-linecount-repo

	git clone --quiet --depth 1 "$1" ${TMP}
	printf "('${TMP}' will be deleted automatically)\n\n\n"
	cloc ${TMP}
	rm -rf ${TMP}
}

BASE="https://github.com/MomsFriendlyRobotCompany/"

for REPO in "pycreate2" "nxp_imu" "opencvutils" "fake_rpi" "ins_nav"
do
  loc ${BASE}${REPO}
done


loc "https://github.com/MarsUniversity/ece387"
```

The output is:

    ==========================================
	   https://github.com/MomsFriendlyRobotCompany/ins_nav
	==========================================
	('/var/tmp/temp-linecount-repo' will be deleted automatically)


	      23 text files.
	      23 unique files.                              
	      11 files ignored.

	github.com/AlDanial/cloc v 1.74  T=0.12 s (130.4 files/s, 10029.3 lines/s)
	-------------------------------------------------------------------------------
	Language                     files          blank        comment           code
	-------------------------------------------------------------------------------
	Python                          14            173            417            539
	YAML                             1              5             10             10
	-------------------------------------------------------------------------------
	SUM:                            15            178            427            549
	-------------------------------------------------------------------------------
