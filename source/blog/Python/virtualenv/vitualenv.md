![](virt.jpg)

# Python Virtual Environments

These virtual environments allow you to keep different libraries separated or
test out new libraries without polluting your OS `site-packages`.

## References

- [Pyimagsearch.com tutorial](https://www.pyimagesearch.com/2018/09/19/pip-install-opencv/)
- [Virtual environment docs](https://virtualenvwrapper.readthedocs.io/en/latest/)
- [another tutorial](https://realpython.com/python-virtual-environments-a-primer/)

# Setup

Install the `pip3` package:

    pip3 install virtualenv virtualenvwrapper

Then adjust your `.bash_profile`:

    nano ~/.bash_profile
    # virtualenv and virtualenvwrapper
    export WORKON_HOME=$HOME/.virtualenvs
    export VIRTUALENVWRAPPER_PYTHON=/usr/local/bin/python3
    source /usr/local/bin/virtualenvwrapper.sh

If you look in `~/.virtualenvs` you will see the envs you created.

# Create a Python3 Virtual Env

Install packages you need. Here we are creating a setup for OpenCV:

    mkvirtualenv cv -p python3
    pip install opencv-contrib-python
    pip install jupyter matplotlib

# Environments

- *Create:* `mkvirtualenv`
- *Start:* `workon cv`
- *Stop:* `deactivate`
- *Remove:* `rmvitualenv`
