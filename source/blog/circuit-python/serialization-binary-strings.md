---
title: Serialization with Binary Strings
date: 26 Apr 2020
---

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

You can also do:

- [Byte alignment](https://docs.python.org/3.7/library/struct.html#byte-order-size-and-alignment)
- Pack/unpack various [data types](https://docs.python.org/3.7/library/struct.html#format-characters)
