---
title: Displaying Status of Pi Hole using RPi 7in LCD
date: 23 Sept 2019
---

![](https://camo.githubusercontent.com/fe3dec9387952987a6a7fd5c7d9f7217a578737a/68747470733a2f2f6a706d636b2e636f6d2f696d672f626c6f672f706164642e706e67)

The software is available [here](https://github.com/jpmck/PADD)

1. Get project: `git clone https://github.com/jpmck/PADD.git`
1. Append the following to the end of pi's `.bashrc` file. If `pi` logs in on the LCD
then `padd.sh` will run, else if `pi` is logged in on an `ssh` connection it will be
normal login.
    ```
    # Run PADD
    # If we’re on the PiTFT screen (ssh is xterm)
    if [ "$TERM" == "linux" ] ; then
      while :
      do
        ./padd.sh
        sleep 1
      done
    fi
    ```
1. Enable `pi` autologin
    1. Run: sudo raspi-config
    1. Choose option 3: Boot Options
    1. Choose option B1: Desktop / CLI
    1. Choose option B2: Console Autologin
    1. Select Finish, and reboot the pi
    
Now, you should see `padd.sh` running on the LCD.
