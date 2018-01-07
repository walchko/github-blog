macOS Boot Disk
=====================

:date: 2016-02-15
:summary: Making a USB boot disk

Since operating systems do not come with installation disks anymore, it can be
useful to have something (non-networked) in re-install macOS from just incase
you don't have access to the internet. This will make a simple bootable USB
thumb drive with everything you need to re-install macOS with.

Need
-----

* 8 GB or larger thumb drive
* Your user account needs to have admin rights
* The macOS installer from the App Store

Steps
------
1. Ensure your drive is named ``Untitled``, otherwise rename it.

2. Run the Terminal command in the same location as the installer::

    sudo Install\ OS\ X\ Yosemite.app/Contents/Resources/createinstallmedia --volume /Volumes/Untitled --applicationpath /Applications/Install\ OS\ X\ Yosemite.app --nointeraction

3. **Warning:** This step will erase the destination drive or partition, so make
sure that it doesn’t contain any valuable data.

4. The Terminal will display some info, but, depending on the speed or your
mac, it could take 20-30 mins. Just wait until it says ``Copy Complete``.

You can rename the thumb drive to something other than ``Untitled`` now.

References
----------

1. `Macworld <http://www.macworld.com/article/2367748/os-x/how-to-make-a-bootable-os-x-10-10-yosemite-install-drive.html>`_
