---
title: Creating a Custom Raspbian Image
date: 30 Aug 2019
---

1. Setup an image on an SD card
1. Clone the image: `sudo dd if=/dev/sdb of=/path/to/image.img`
1. Shrink the image
    1. Get the shrink script and make it executable:
    ```
    wget  https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
    chmod +x pishrink.sh
    ```
    2. `sudo pishrink.sh /path/to/image.img /path/to/shrunk/image/img`
1. *[Optional]* Further compression can be done with: `gzip -9 /path/to/shrunk/image.img`

## References

- [Creating Custom Images for Production](https://medium.com/platformer-blog/creating-a-custom-raspbian-os-image-for-production-3fcb43ff3630)
