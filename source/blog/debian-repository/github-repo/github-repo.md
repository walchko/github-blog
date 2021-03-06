---
title: Using Github as a Debian Repository
date: 15 Feb 2020
---

This is mainly a summary of the reference:

1. Create a github repo for this
1. Build a debian PACKAGE
1. Make a public key and put it in the github repo
1. Build a DEB repo in the github repo and import PACKAGE
1. Push everything to github
1. Add GPG key to apt source keyring: `wget -qO - https://USERNAME.github.io/REPOSITORY/PUBLIC.KEY | sudo apt-key add -`
1. Add repo to `/etc/apt/sources.list`: `deb http://USERNAME.github.io/REPOSITORY CODENAME COMPONENT`
1. `sudo apt update`
1. Install package: `sudo apt install PACKAGE`

## Setup Repo with `reprepo` Tool

1. Install tools: `sudo install reprepo`
1. `mkdir debian && cd debian`
1. `mkdir conf && cd conf`
1. `touch distributions`
    1. Example:
    ```
    Origin: walchko.github.io/debian
    Label: walchko.github.io/debian
    Codename: buster
    Architectures: amd64
    Components: main
    Description: Personal repository
    SignWith: A2123E3E
    ```
    1. `SignWith` value comes from `gpg --list-secret-keys` and get the `ssb` value
    1. `Codename` is the version, `buster`, `sid`, `jessie`, etc
1. Now manage repo with:
    1. `reprepro --basedir REPOSITORY.PATH includedeb CODENAME PACKAGE`
    1. `reprepro --basedir REPOSITORY.PATH list CODENAME`
    1. `reprepro --basedir REPOSITORY.PATH remove CODENAME PACKAGE`

## Apt Keyring

- `apt-key list`
- `sudo apt-key del KEY.ID`

## GPG

1. Install tools: `sudo apt install gnupg`
1. if `cat /proc/sys/kernel/random/entropy_avail` has an entropy less than 3000 (which you need for a 4096 bit key), then install `sudo apt install rng-tools`
1. Either:
    1. Generate key: `gpg --gen-key`
    1. Or use and existing key: `gpg --allow-secret-key-import --import private.key`
1. Export public key: `gpg --output PUBLIC.KEY --armor --export EMAIL.ADDRESS`

| GPG Cheat Sheet                                                  | 
|------------------------------------------------------------------|
| `gpg --gen-key`
| `gpg --output public.key --armor --export email.address`
| `gpg --output private.key --armor --export-secret-key email.address`
| `gpg --import public.key`
| `gpg --allow-secret-key-import --import private.key`
| `gpg --delete-key email.address`
| `gpg --delete-secret-key email.address`
| `gpg --ist-keys`
| `gpg --list-secret-keys`

- **Note:** the `--armor` is used to export a key in ASCII format

# References

- [Personal DEB Package Repo using Github Pages](https://pmateusz.github.io/linux/2017/06/30/linux-secure-apt-repository.html)
