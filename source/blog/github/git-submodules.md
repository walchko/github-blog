---
title: Submodules
---

Simple cheatsheet

- Creating: `git submodule add git@github.com:url_to/awesome_submodule.git path_to_awesome_submodule`
- Clone a repo with a submodule:
    - `git clone --recursive [URL to Git repo]`
    - `git submodule update --init --recursive`
- Updating: `git submodule update`
- Moving: `git mv old/submod new/submod`
- To remove a submodule you need to:
    - Delete the relevant line from the `.gitmodules` file
    - Delete the relevant section from `.git/config`
    - Run `git rm --cached pathtosubmodule` (no trailing slash)
    - Commit the super project.
    - Delete the now untracked submodule files.

# References

- [remove a submodule](https://gist.github.com/myusuf3/7f645819ded92bda6677)
