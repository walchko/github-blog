Git
====

:date: 2013-08-17
:modified: 2017-01-12
:summary: Using ``git``

.. figure:: {filename}/blog/computers/pics/github.png
    :width: 200px
    :align: center

Git is a version control system (VCS).

**Note**: For some reason the ass clowns at Github quietly put in support for
large files, so you have to install an extension to github. If you don't, it
can be painful to work with github from the command line.

For OSX::

    brew install git-lfs

Git Cheat Sheet
---------------

+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Command   | Example                                                    | Definition                                                                                                                                       |
+===========+============================================================+==================================================================================================================================================+
| Init      | ``git init``                                               |Start a repository                                                                                                                                |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Clone     | ``git clone git://somewhere.com/something.git [new_name]`` |Create a copy of a repository                                                                                                                     |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Remote    | ``git remote -v``                                          |Display remote repository                                                                                                                         |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Push      | ``git push origin [master]``                               | Share changes to an upstream remote (origin) for a branch                                                                                        |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Commit    | ``git commit -m 'update'``                                 | One step add/push upstream                                                                                                                       |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Add       | ``git add *.c``                                            | Add files to the repository or to stage files for commit                                                                                         |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Fetch     | ``git fetch [origin]``                                     | Pull all changes from a remote repository that have been pushed since you cloned it. Note this doesn't merge any of the changes.                 |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Pull      | ``git pull``                                               | Will automatically ``fetch`` and merge changes from upstream into current branch                                                                 |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Tag       | ``git tag -a v1.4 -m 'my Version 1.4'``                    | Create an annotated tag, note you may have to: ``git push origin v1.4`` or ``git push origin --tags`` to get tags push on upstream server.       |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Status    | ``git status``                                             | Reports the status of untracked changes in your working repository                                                                               |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Diff      | ``git diff``                                               | Display changes between working directory and repository                                                                                         |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Rm        | ``git rm file.c``                                          | Remove files from repository                                                                                                                     |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Mv        | ``git mv from_file to_file``                               | Move files                                                                                                                                       |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
| Log       | ``git log``                                                | View the commit log                                                                                                                              |
+-----------+------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+


Git Setup
---------

Let's start off with making git look nice::

    git config --global color.ui true

Since we will work remotely, we need to tell git who we are. Git stores
that info in ``~/.gitconfig`` ::

    git config --global user.name "walchko"
    git config --global user.email kevin.walchko@hotmail.com

Working with Git
----------------

You may need to create an ssh keys following the `github directions <https://help.github.com/articles/generating-ssh-keys>`__

Basically:

1. create a key::

	ssh-keygen -t rsa -C "pi@bender.local"

2. Add the key to the key agent::

       pi@bender ~ $ eval "$(ssh-agent -s)"
       Agent pid 12480
       pi@bender ~ $ ssh-add ~/.ssh/id_rsa
       Identity added: /home/pi/.ssh/id_rsa (/home/pi/.ssh/id_rsa)

   You can also run ``ssh-add -l`` to see what keys have been loaded.

3. Go to github and add a new ssh key under your profile. Copy/paste in
   the key (use ``more ~/.ssh/id_rsa.pub``) making sure not to add or
   remove white space. You can also use ``pbcopy < ~/.ssh/id_rsa.pub`` to copy it to your
   clip board on macOS.

4. Then try to ssh in::

       pi@bender ~ $ ssh -T git@github.com
       The authenticity of host 'github.com (192.30.252.128)' can't be established.
       RSA key fingerprint is 1d:57:ac:a4:76:23:2d:34:63:1b:56:4d:74:7f:76:48.
       Are you sure you want to continue connecting (yes/no)? yes
       Warning: Permanently added 'github.com,192.30.252.128' (RSA) to the list of known hosts.
       Hi walchko! You've successfully authenticated, but GitHub does not provide shell access.

**Success!**

Now clone a repository, make sure you use the ``ssh`` address and not
the default https one ::

    git clone git@github.com:walchko/soccer.git

**Note:** The https one has ``https`` in the address: ``https://github.com/walchko/soccer2.git``

if you accidentally clone the ``https`` one, you can switch to ``ssh`` by ::

    git remote set-url origin git@github.com:walchko/soccer2.git

Git Workflow
------------

Read `this <http://rogerdudler.github.io/git-guide/>`__ awesome guide

1. Make sure your current copy is up to date ::

       git pull

2. Create a new branch to hold your new feature ::

       git checkout -b my-cool-new-thing

3. Edit your code. To see status::

	git status
	On branch master
	Your branch is up-to-date with 'origin/master'.
	Changes not staged for commit:
	  (use "git add <file>..." to update what will be committed)
	  (use "git checkout -- <file>..." to discard changes in working directory)

		modified:   docs/computers/git.rst

	no changes added to commit (use "git add" and/or "git commit -a")

4. Add files to stash (AKA index or cache)::

	git add *

5. Commit files (locally) to HEAD in your ``.git`` repository::

       git commit -m "what did you do?"
       pi@bender ~/soccer/IMU $ git push origin master
       Counting objects: 12, done.
       Compressing objects: 100% (8/8), done.
       Writing objects: 100% (8/8), 736 bytes, done.
       Total 8 (delta 6), reused 0 (delta 0)
       To git@github.com:walchko/soccer.git
          8162ade..cd9a476  master -> master

6. Push changes upstream to github (or where ever) so everyone can use them ::

       git push origin master

   or ``git push`` because ``origin master`` is the default.

7. Create a tag ::

		git tag -a v0.5.3 -m "update"
		git push origin v0.5.3

To undo what you have committed already and basically create an anti-patch for each commit ::

    git revert 0766c053 25eee4ca a867b4af

Submodules
------------

Sometimes you need to include another git in your project ... how do you do that?

::

    git submodule add git@github.com:adafruit/Adafruit_Python_PureIO.git

This will include it. Now you can track development and update as necessary to
the latest code. This will also add a ``.gitmodules`` to your project which
will track what submodules (other git projects) you are tracking.

When you need to clone your project (and the submodules too) though, you now
have to do::

    git clone --recursive git@github.com:...

This will pull your project and any submodules your project has too.


Setting up your own server on the RPi
-----------------------------------------

`tutorial <https://www.sitepoint.com/setting-up-your-raspberry-pi-as-a-git-server/>`_

References
-------------

* `git - the simple guide <http://rogerdudler.github.io/git-guide/>`_
* `A visual git Reference <http://marklodato.github.io/visual-git-guide/index-en.html>`_
* `Git for ages 4 and up <https://www.youtube.com/watch?v=5Q7omG_9RkI>`_
* `Interactive cheatsheet <http://ndpsoftware.com/git-cheatsheet.html>`_
* `git-scm.com <https://git-scm.com/docs>`_
