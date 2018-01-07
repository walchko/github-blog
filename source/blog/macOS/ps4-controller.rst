PS4 Controller Setup on Mac macOS
====================================

:date: 2016-03-06
:modified: 2016-07-31
:summary: How to hook up your PS4 controller to macOS

You can use bluetooth, SDL2, and python to talk with a controller.

.. figure:: {filename}/blog/macOS/pics/ps4-controller.jpg
   :alt:

First we need to pair the controller with your Mac:

1. Use a Micro-USB cable to connect the PS4 controller to your Mac.
2. Press the PlayStation button in the middle of the gamepad (to turn it on).
3. Choose Apple > About this Mac.
4. Click System Report.
5. Choose USB and look for Wireless Controller under USB. If you can see Wireless Controller (it's called "wireless" even though it's connected via a cable), then your PS4 controller is connected to the Mac.

.. figure:: {filename}/blog/macOS/pics/sys-info.jpg
   :alt:

Next, connect the controller via bluetooth:

1. Open System Preferences (Apple menu > System Preferences).
2. Click Bluetooth.
3. Put the PS4 controller in Discovery Mode by holding down the PlayStation button and Share button at the same time. The light on the front of the controller will flash quickly, and Wireless Controller will appear in the Bluetooth window.
4. Click Pair. The device will now say connected, and you'll see how.
