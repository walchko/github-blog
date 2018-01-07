
External USB Drive
==================

:date: 2015-11-28
:modified: 2016-09-04
:summary: Setting up a hard drive to work with Raspbian

.. figure:: {filename}/blog/raspbian/pics/usb.png
	:width: 200px
	:align: center

A good resource is
`here <http://devtidbits.com/2013/03/21/using-usb-external-hard-disk-flash-drives-with-to-your-raspberry-pi/>`__

Setup
-----

First connect the drive and find it:

::

    pi@calculon ~ $ sudo fdisk -l

    Disk /dev/mmcblk0: 15.9 GB, 15931539456 bytes
    4 heads, 16 sectors/track, 486192 cylinders, total 31116288 sectors
    Units = sectors of 1 * 512 = 512 bytes
    Sector size (logical/physical): 512 bytes / 512 bytes
    I/O size (minimum/optimal): 512 bytes / 512 bytes
    Disk identifier: 0x000981cb

            Device Boot      Start         End      Blocks   Id  System
    /dev/mmcblk0p1            8192      122879       57344    c  W95 FAT32 (LBA)
    /dev/mmcblk0p2          122880    31116287    15496704   83  Linux

    WARNING: GPT (GUID Partition Table) detected on '/dev/sda'! The util fdisk doesn't support GPT. Use GNU Parted.


    Disk /dev/sda: 640.1 GB, 640135028736 bytes
    255 heads, 63 sectors/track, 77825 cylinders, total 1250263728 sectors
    Units = sectors of 1 * 512 = 512 bytes
    Sector size (logical/physical): 512 bytes / 512 bytes
    I/O size (minimum/optimal): 512 bytes / 512 bytes
    Disk identifier: 0x00000000

       Device Boot      Start         End      Blocks   Id  System
    /dev/sda1               1  1250263727   625131863+  ee  GPT

Make a mount point and mount and/or unmount the drive.

::

    pi@calculon ~ $ sudo mkdir /mnt/usbdrive
    pi@calculon ~ $ sudo mount /dev/sda1 /mnt/usbdrive
    pi@calculon ~ $ sudo umount /dev/sda1

Formatting Disk
---------------

Starting with a USB hard drive 640GB formated in OSX's HSF+, I needed to
change it to ext4:

::

    pi@calculon ~ $ sudo parted /dev/sda
    GNU Parted 2.3
    Using /dev/sda
    Welcome to GNU Parted! Type 'help' to view a list of commands.
    (parted) p
    Model: Maxtor OneTouch (scsi)
    Disk /dev/sda: 640GB
    Sector size (logical/physical): 512B/512B
    Partition Table: gpt

    Number  Start  End  Size  File system  Name  Flags

    (parted) mkpart primary ext4 1 640000
    (parted) p
    Model: Maxtor OneTouch (scsi)
    Disk /dev/sda: 640GB
    Sector size (logical/physical): 512B/512B
    Partition Table: gpt

    Number  Start   End    Size   File system  Name     Flags
     1      1049kB  640GB  640GB               primary

    (parted) quit
    Information: You may need to update /etc/fstab.

**Note** in the ``mkpart`` line, the start and stop locations are in MB.

::

    pi@calculon ~ $ sudo mkfs.ext4 /dev/sda1 -L Calculon
    mke2fs 1.42.5 (29-Jul-2012)
    Filesystem label=Calculon
    OS type: Linux
    Block size=4096 (log=2)
    Fragment size=4096 (log=2)
    Stride=0 blocks, Stripe width=0 blocks
    39067648 inodes, 156249856 blocks
    7812492 blocks (5.00%) reserved for the super user
    First data block=0
    Maximum filesystem blocks=0
    4769 block groups
    32768 blocks per group, 32768 fragments per group
    8192 inodes per group
    Superblock backups stored on blocks:
        32768, 98304, 163840, 229376, 294912, 819200, 884736, 1605632, 2654208,
        4096000, 7962624, 11239424, 20480000, 23887872, 71663616, 78675968,
        102400000

    Allocating group tables: done
    Writing inode tables: done
    Creating journal (32768 blocks): done
    Writing superblocks and filesystem accounting information: done

Now mount the drive and double check it:

::

    pi@calculon ~ $ sudo mount /dev/sda1 /mnt/usbdrive
    pi@calculon ~ $ df -h
    Filesystem      Size  Used Avail Use% Mounted on
    rootfs           15G  3.7G   11G  27% /
    /dev/root        15G  3.7G   11G  27% /
    devtmpfs        112M     0  112M   0% /dev
    tmpfs            24M  240K   23M   2% /run
    tmpfs           5.0M     0  5.0M   0% /run/lock
    tmpfs            47M     0   47M   0% /run/shm
    /dev/mmcblk0p1   56M  9.8M   47M  18% /boot
    /dev/sda1       587G   70M  557G   1% /mnt/usbdrive

Fix permissions:

::

    pi@calculon ~ $ sudo chown pi:pi /mnt/usbdrive
    pi@calculon ~ $ sudo chmod 777 /mnt/usbdrive
    pi@calculon ~ $ sudo ls /mnt/usbdrive -alh
    total 24K
    drwxrwxrwx 3 pi   pi   4.0K Dec 14 20:45 .
    drwxr-xr-x 3 root root 4.0K Dec 14 18:24 ..
    drwx------ 2 root root  16K Dec 14 20:45 lost+found

Automounting
------------

Add the following to fstab so it mounts on boot.

::

    sudo nano /etc/fstab
    /dev/sda1 /mnt/usbdisk auto defaults,user 0 1

This sets the file system to ``auto`` and ``user`` enables write
permissions for all users. The 0 is for debugging and 1 is for a file
system check at boot. You can test this out by:

For a USB thumb drive formated in ``vfat`` you can do::

	# <file system> <mount pt>     <type>   <options>                  <dump>  <pass>
	/dev/sda1       /mnt/usbdisk    auto    auto,user,uid=1000,gid=1000  0       2

Since I leave the usb drive in all the time, I have the option ``auto`` to always mount it. 
When the drive is mounted, user ``pi`` has ownership with the ``uid`` and ``gid``
options. How do you find the user/grp id? ::

	pi@calculon /mnt $ id -u pi
	1000
	pi@calculon /mnt $ id -g pi
	1000
	pi@calculon /mnt $ id pi
	uid=1000(pi) gid=1000(pi) groups=1000(pi),4(adm),20(dialout),24(cdrom),27(sudo),
	29(audio),44(video),46(plugdev),60(games),100(users),101(input),108(netdev),
	999(spi),998(i2c),997(gpio)


::

    sudo mount -a

Swap Partition on Hard Drive
----------------------------

Don't ever make a swap partition on the sd card ... it is too slow and
will reduce the card's life span.

1. Create a partition for swap on say ``/dev/sda2`` following the method
   above, then exit ``parted``.
2. Use ``mkswap /dev/sda2`` to set it up.
3. Edit ``/etc/fstab`` and add the following line:
   ``/dev/sda2 none swap sw 0 0``
4. Get rid of RPi's file base swap by removing the packages:
   ``sudo apt-get remove dphy-swapfile``
5. Make sure swap is working: ``swapon -s`` ::

   	pi@calculon ~ $ swapon -s Filename Type Size Used Priority /dev/sda2
   	partition 4295676 0 -1
