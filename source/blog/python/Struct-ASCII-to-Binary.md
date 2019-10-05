# Struct

```bash
>>> import struct
>>> d=[0xff,0xaa,0x99,0x55,0x11,0x00]
>>> d
[255, 170, 153, 85, 17, 0]
>>> bin = struct.pack('B'*len(d),*d)
>>> bin
'\xff\xaa\x99U\x11\x00'
```

```bin
>>> struct.unpack('B'*len(bin),bin)
(255, 170, 153, 85, 17, 0)
```

# Reference

- [Python 2.7 struct](https://docs.python.org/2/library/struct.html)
