---
title: Setup Multiple External USB Disks with Labels
date: 2018-05-05
---

Since USB drives are dynamically assigned drive letters, you need to find the UUID
number.

```bash
pi@ultron ~ $ sudo blkid
/dev/mmcblk0p1: LABEL="boot" UUID="A365-6756" TYPE="vfat" PARTUUID="4338245d-01"
/dev/mmcblk0p2: LABEL="rootfs" UUID="90a83158-560d-48ee-9de9-40c51d93c287" TYPE="ext4" PARTUUID="4338245d-02"
/dev/mmcblk0: PTUUID="4338245d" PTTYPE="dos"
/dev/sda1: UUID="4e9f9996-b491-4ec5-a0df-360eaae9f74c" TYPE="ext4" PARTUUID="6d95cef4-01"
/dev/sdb1: UUID="20ad368c-a224-423e-9ca6-4ecbca595ad5" TYPE="ext4" PARTUUID="a0d8dba5-01"
```

Here, we see `sda1` and `sdb1` UUID numbers which we can use in `/etc/fstab` to ensure
we always get the same drive.

## Update `/etc/fstab`

```bash
pi@ultron ~ $ cat /etc/fstab
proc            /proc           proc    defaults          0       0
PARTUUID=4338245d-01  /boot           vfat    defaults          0       2
PARTUUID=4338245d-02  /               ext4    defaults,noatime  0       1
# a swapfile is not a swap partition, no line here
#   use  dphys-swapfile swap[on|off]  for that
#/dev/sda1 /mnt/a ext4 defaults,auto,user,rw 0 0
#/dev/sdb1 /mnt/b ext4 defaults,auto,user,rw 0 0
UUID=20ad368c-a224-423e-9ca6-4ecbca595ad5 /mnt/a ext4 defaults,auto,user,rw 0 0
UUID=4e9f9996-b491-4ec5-a0df-360eaae9f74c /mnt/b ext4 defaults,auto,user,rw 0 0
```

After a quick reboot and alowing the drives to be automounted, we can check them out:

```bash
pi@ultron ~ $ lsblk
NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINT
sda           8:0    0 238.5G  0 disk 
└─sda1        8:1    0 238.5G  0 part /mnt/b
sdb           8:16   0 238.5G  0 disk 
└─sdb1        8:17   0 238.5G  0 part /mnt/a
mmcblk0     179:0    0  14.9G  0 disk 
├─mmcblk0p1 179:1    0  43.1M  0 part /boot
└─mmcblk0p2 179:2    0  14.8G  0 part /
```

## Fix Permissions

Repeat this for each disk:

- Create mount point: `sudo mkdir /mnt/a`
- Allow user to own it: `sudo chown pi:pi /mnt/a`
- Change permissions: `sudo chmod 777 /mnt/a`

When ready, mount everything with:

```
sudo mount -a
```
