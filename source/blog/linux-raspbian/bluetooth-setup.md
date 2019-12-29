---
title: Setting-up Bluetooth
date: 29  Dec 2019
---

## Using Bluetooth within the Terminal

In this section, we are going to run you through the steps to using Bluetooth on your Raspberry Pi by using the terminal.

1. To load the Bluetooth command-line tool, you need to enter the following command on your Raspberry Pi.
    ```bluetoothctl```
1. Now that we are in the Bluetooth command-line tool, we need to go ahead and turn the agent on. Switching the agent 
    on will allow us to search for and pair with other Bluetooth devices. You can do this by using the command below.
    ```
    agent on
    ```
1. The next step is to tell the Bluetooth device on our Raspberry Pi to scan for other devices. By scanning for 
    devices, we can retrieve their MAC address and begin the process of pairing that device with the Raspberry Pi.
    To start the scan process, all you need to do is enter the following command.
    ```
    scan on
    ```
    From this command, you should start seeing a result like what we have below.
    ```
    [bluetooth]# scan on
    Discovery started
    [CHG] Controller DC:A6:32:05:7F:06 Discovering: yes
    [NEW] Device 51:B8:16:6A:6F:C6 51-B8-16-6A-6F-C6
    [NEW] Device 40:23:43:3F:4E:58 BRAVIA 4K UR2
    ```
    The two columns you will probably want to pay attention to the most are the third and fourth columns. The third 
    column specifies the MAC address of the device that triggered the event. This address is what you will use 
    if you want to pair the device. The fourth column is the descriptor. For a newly found device, this is 
    typically the device name.

1. Once you have found the MAC address of the device you want to connect to, you can now proceed to pair 
    your Raspberry Pi with it. To get Bluetooth to pair the device to your Raspberry Pi, you need to make 
    use of the following command.
    ```
    pair [XX:XX:XX:XX:XX:XX]
    ```
1. When you first pair a device, you will be immediately connected to it. However, once you have gone 
    out of range of the Raspberry Pi’s Bluetooth, you will need to re-connect the device by using the 
    following command.
    ```
    connect [XX:XX:XX:XX:XX:XX]
    ```
1. If you don’t want to have to re-pair your device, then you can make use of the trust command.  This command 
    works just like the other two commands and requires the MAC address of the Bluetooth device that you want 
    to trust.
    ```
    trust [XX:XX:XX:XX:XX:XX]
    ```
    You can only trust a Bluetooth device on your Raspberry Pi after you have completed the initial pairing.

Hopefully, at this point, you will now have Bluetooth up and running without any issue.

# References

- Copied from [pimylifeup.com](https://pimylifeup.com/raspberry-pi-bluetooth/)
