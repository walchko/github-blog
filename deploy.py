#!/usr/bin/env python3
from subprocess import check_output
import platform
import os
import tempfile
import shutil
import time


def run(cmd):
    # given a command string, it runs it
    cmds = cmd.split()
    return check_output(cmds)


REMOTE = "git@github.com:walchko/walchko.github.io.git"
REPO = "github.com/walchko/walchko.github.io"
BRANCH = "master"
FOLDER = "html"

tmp = tempfile.mkdtemp()

# recursively copy everything to the tmp directory
# make a new git repo and push everything to the branch
# this will make github display the webpages
shutil.copytree('html', tmp + '/html')

os.chdir(tmp + '/html')

print('====================')
print('    Redoing git')
print('====================')

# create readme with data
readme = "# Blog\n\nThis site is automatically generated\n\nUpdated on {}".format(time.strftime("%Y-%m-%d %H:%M"))
with open('readme.md', 'w') as fd:
    fd.write(readme)

run('git init')
run('git add *')
run('git commit -m "updated-on-{}:{}-:space_invader:"'.format(REPO, BRANCH))


print('====================')
print('    git push')
print('====================')

run('git push --force {} master:{}'.format(REMOTE, BRANCH))

# from glob import glob
# print(os.getcwd())
# for g in glob('*'):
#     print(g)
