#!/bin/bash
# https://github.com/travis-ci/dpl/blob/master/lib/dpl/provider/pages.rb
#
# WARNING: deploy will wipe out everything, so setup your deployment
# to gh-pages branch in the github settings drop down ... NOT master!
#
# fix: git push origin master --force
#      where master is the branch name

# repo and branch to save website too
REPO="github.com/MarsUniversity/ece382"
BRANCH="gh-pages"

# setup travis or local env for pushing html to webpage
if [[ ${TRAVIS} ]]; then
	TMP="../tmpdir"

	# git info
	USER="walchko"
	EMAIL="${USER}@users.noreply.github.com"
	git config user.email ${EMAIL}
	git config user.name ${USER}
	REMOTE="https://${GITHUB_TOKEN}@${REPO}"
# setup for local manual push
else
	TMP="/var/tmp/tmpdir"
	REMOTE="git@github.com:MarsUniversity/ece382.git"
	rm -fr ${TMP}
fi

# website data is stored here:
FOLDER="html"

set -e

mkdir -p ${TMP}

if [[ -d ${FOLDER} ]]; then
	cp -R ${FOLDER}/* ${TMP}
else
	echo "${FOLDER} not found"
	ls
	pwd
	exit 1
fi

cd ${TMP}

echo "======================"
echo "  Redoing git"
echo "======================"

# create readme with data
echo "deployed at `date`" > README.md

# setup new git repo
git init
git add *
git commit -m "Deploying to ${REPO}:${BRANCH}"

echo "======================"
echo "  git publish"
echo "======================"

# force the contents of this master branch to gh-pages branch
# git push --force --quiet "https://${GITHUB_TOKEN}@${REPO}" master:${BRANCH} > /dev/null 2>&1
# git push --force  "https://${GITHUB_TOKEN}@${REPO}" master:${BRANCH}
# git push --force origin master:${BRANCH}

git push --force ${REMOTE} master:${BRANCH}
