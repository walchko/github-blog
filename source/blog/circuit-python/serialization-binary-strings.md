---
title: Serialization with Binary Strings
date: 26 Apr 2020
---

Unfortunately, for some dumb reason, the only way to get data over the USB
is via the `print` command which sends ASCII characters only ... **wtf**!
Thus if you `print(pi)` is will send the ASCII codes `['3','.','1','4', ... 
'\r','\n']` and now you have to put this text back together and loose whatever
bits didn't end up in the ASCII string. Since I like to read sensors, I
want all of my bits. :smile:

## `struct`

Circuitpython contains a `struct` library and allows you to format data
into binary strings.

```python
>>> import struct
>>> d = struct.pack("fff",*(-1.0, 1e-9, 15e8))
>>> d
b'\x00\x00\x80\xbf\\p\x890\\\xd0\xb2N'
>>> struct.unpack("fff",d)
(-1.0, 1e-09, 1.5e+09)
>>> 
>>> d=struct.pack('hhl', 1, 2, 3)
>>> struct.unpack('hhl', d)
(1, 2, 3)
```
## `adafruit_binascii`

However, this alone won't do it, because they will be sent as ASCII
characters still. So let's make things even easier and turn these 
into simple hex characters that ASCII likes.

```python
from adafruit_binascii import hexlify, unhexlify

# convert data (d) and send h with: print(h)
h = hexlify(d)

# on the computer side, unpack h to get output, which itself can
# be unpacked again with struct.unpack ... holy crap this sucks!
output = unhexlify(h)
```

You can also do:

- [Byte alignment](https://docs.python.org/3.7/library/struct.html#byte-order-size-and-alignment)
- Pack/unpack various [data types](https://docs.python.org/3.7/library/struct.html#format-characters)
