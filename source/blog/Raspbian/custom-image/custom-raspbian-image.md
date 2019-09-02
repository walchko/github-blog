---
title: Creating a Custom Raspbian Image
date: 30 Aug 2019
---

This assumes you have already setup a "gold" copy of the image you
want. Now let's make a copy of that image so you can easily replicate
on other SD cards.

**Note:** I am using Ubuntu (Linux) for this because I want to shrink the image
once it is made.

1. Setup an image on an SD card and  insert it into the computer
1. Do `df -h` or `sudo fdisk -l` to see what new mounts are there
    1. You cannot copy a mounted drive, so you will have to do `umount /dev/sdbX`
    1. On linux, both the boot and root partitions will be mounted, so unmount both of them
1. Clone the image: `sudo dd if=/dev/sdb of=/path/to/image.img`
1. Shrink the image
    1. Get the [shrink script](https://github.com/Drewsif/PiShrink) and make it executable:
    ```
    wget  https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
    chmod +x pishrink.sh
    ```
    2. `sudo pishrink.sh /path/to/image.img /path/to/shrunk/image/img`
1. *[Optional]* Further compression can be done with: `gzip -9 /path/to/shrunk/image.img`

## References

- [Creating Custom Images for Production](https://medium.com/platformer-blog/creating-a-custom-raspbian-os-image-for-production-3fcb43ff3630)
