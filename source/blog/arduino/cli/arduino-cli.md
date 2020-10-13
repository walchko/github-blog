---
title: Arduino-cli
date: 12 Oct 2020
---

## Install and Setup

- Install with: `brew install arduino-cli`
- Update: `arduino-cli core update-index`
- Install processor: `arduino-cli core install arduino:samd`
- Get 3rd party boards, create a yaml file:
```yaml
# arduino-cli.yaml
board_manager:
  additional_urls:
    - http://arduino.esp8266.com/stable/package_esp8266com_index.json
    - https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
    - https://raw.githubusercontent.com/Seeed-Studio/Seeed_Platform/master/package_seeeduino_boards_index.json
```
- Update: `arduino-cli core update-index`
- Install Adafruit boards: `arduino-cli core install adafruit:avr adafruit:samd`
- Double check your board is available: `arduino-cli board listall`

## Create Project

- Start new project: `arduino-cli sketch new blink`
    - Sketch is created in `blink/blink.ino`
    - It will be empty, so add code to it
- `arduino-cli compile --fqbn arduino:avr:leonardo blink`
- `arduino-cli upload --port COM9 --fqbn arduino:avr:leonardo blink`
    - Make sure the board is attached

# References

- DEV_Dungeon: [Arduino CLI Tutorial](https://www.devdungeon.com/content/arduino-cli-tutorial)
