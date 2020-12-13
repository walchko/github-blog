---
title: Creating a RAM Disk
---

## Create Mount Point

You can call the mount point what you want, but mine is `/ram`.

```bash
sudo mkdir -p /ram
sudo chown -R pi:pi /ram
```

## Edit `/etc/fstab`

```bash
tmpfs /ram tmpfs defaults,user,noatime,nosuid,mode=0755,size=100m 0 0
```

- `user` alows `pi` to mount/unmount it
- `size=100m` limits the max size to 100 MB so we don't consume all of our memory
- `noatime` option fully disables writing file access times to the drive every time you read a file
- `nosuid` specifies that the filesystem cannot contain set userid files. This prevents setuid binaries
  on a world-writable filesystem and makes sense because there's a risk of root escalation or other
  awfulness there.
