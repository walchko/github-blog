---
title: Poetry and The New Way to Maintain a Project
date: 20 Jan 2020
---

![](install.gif)

Nice things

- Reduces all of the various config files (`setup.py`, `requirements.txt`, `setup.cfg`, etc) 
to one `pyproject.toml` in [PEP518](https://www.python.org/dev/peps/pep-0518/) so it is *official*
- Poetry is designed to use the new toml file and reduce the headache of packaging

Not so nice things

-  Install doesn't use `pip` but rather a script you pull with curl: `curl -sSL https://raw.githubusercontent.com/sdispater/poetry/master/get-poetry.py | python`
    - This allows `poetry` to isolate itself from the system distro ... which isn't a bad thing


# References

- Poetry [website](https://python-poetry.org/)
- Poetry [github](https://github.com/python-poetry/poetry)
- Overview and thoughts of poetry in a [blog](https://hackersandslackers.com/python-poetry/)
