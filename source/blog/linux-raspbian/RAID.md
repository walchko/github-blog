# RAID

Here I am looking at RAID-1 or mirroring.

## References

- [tecmint](https://www.tecmint.com/create-raid1-in-linux/)
- [pchelp](https://pchelp.ricmedia.com/build-raspberry-pi3-raid-nas-server/3/)
- [eltechs](https://eltechs.com/raspberry-pi-nas-guide/)

# Individual Disk Performance

Spinning 2.5" drive:

```bash
pi@plex ~ $ sudo hdparm -t /dev/sda1

/dev/sda1:
 Timing buffered disk reads:  78 MB in  3.04 seconds =  25.66 MB/sec
pi@plex ~ $ sudo hdparm -t /dev/sda1

/dev/sda1:
 Timing buffered disk reads:  84 MB in  3.02 seconds =  27.85 MB/sec
pi@plex ~ $ sudo hdparm -t /dev/sda1

/dev/sda1:
 Timing buffered disk reads:  84 MB in  3.03 seconds =  27.69 MB/sec
 ```
 
 SSD 2.5" drive:
 
 ```bash
pi@ultron ~ $ sudo hdparm -t /dev/sdb1

/dev/sdb1:
 Timing buffered disk reads:  66 MB in  3.00 seconds =  21.98 MB/sec
pi@ultron ~ $ sudo hdparm -t /dev/sdb1

/dev/sdb1:
 Timing buffered disk reads:  68 MB in  3.07 seconds =  22.15 MB/sec
pi@ultron ~ $ sudo hdparm -t /dev/sdb1

/dev/sdb1:
 Timing buffered disk reads:  72 MB in  3.01 seconds =  23.96 MB/sec
pi@ultron ~ $ sudo hdparm -t /dev/sdb1

/dev/sdb1:
 Timing buffered disk reads:  76 MB in  3.02 seconds =  25.17 MB/sec
 ```
