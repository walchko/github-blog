---
title: Arduino Setup on macOS and Linux
date: 16 Aug 2020
image: "https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Arduino_Logo.svg/800px-Arduino_Logo.svg.png"
image-height: "300px"
---

Here are a few keynotes for installing Arduino

- macOS: [download](https://www.arduino.cc/en/Main/Software) for macOS
- Linux: Software (brief case) -> arduino

## Setup

![](https://cdn-learn.adafruit.com/assets/assets/000/032/477/original/flora_bsp.png?1463422146)

![](https://cdn-learn.adafruit.com/assets/assets/000/026/781/original/flora_install.gif?1448319282)

- Preferences, Additional Board Managers URLs: `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`
- Tools, Board Managers, add:
    - Adafruit SAMD Boards
    - Arduino SAMD Boards

## Serial Port Access

If you're on Linux, and are seeing multi-second delays connecting to the serial console, or are seeing "AT" and other gibberish when you connect, then the modemmanager service might be interfering. Just remove it; it doesn't have much use unless you're still using dial-up modems. To remove, type this command at a shell: `sudo apt purge modemmanager`

# References

- Adafruit Learn: [Arduino Setup](https://learn.adafruit.com/adafruit-trinket-m0-circuitpython-arduino/arduino-ide-setup)
- Adafruit Learn: [udev Linux Setup](https://learn.adafruit.com/adafruit-arduino-ide-setup/linux-setup#udev-rules)
