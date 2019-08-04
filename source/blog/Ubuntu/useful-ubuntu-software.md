---
title: Software Setup
date: 3 Aug 2019
---

Here are some of the key steps to installing some useful engineering
software on Ubuntu.

## KiCad

```
sudo add-apt-repository --yes ppa:js-reynaud/kicad-5.1
sudo apt update
sudo apt install --install-suggests kicad
```

## OpenSCAD

```
sudo add-apt-repository ppa:openscad/releases
sudo apt update
sudo apt install openscad
```

## Chrome

```
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' | sudo tee /etc/apt/sources.list.d/google-chrome.list
sudo apt update 
sudo apt install google-chrome-stable
```

## Handbrake

```
sudo add-apt-repository ppa:stebbins/handbrake-releases
sudo apt install handbrake-gtk handbrake-cli
```

## Atom.io

```
sudo apt install software-properties-common apt-transport-https wget
wget -q https://packagecloud.io/AtomEditor/atom/gpgkey -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main"
sudo apt install atom
```

## Node.js

```
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt-get install -y nodejs
```
