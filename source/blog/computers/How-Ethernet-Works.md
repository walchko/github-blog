---
title: Ethernet Standard
date:   2017-06-14
summary:   How ethernet and cat5 cables work
---

## BaseT

A standard ethernet cable (Cat 5, CAT 5E, or Cat 6) has 4 pairs of
cables (8 wires total). For 10BaseT (10Mbps) and 100BaseT (100Mbps) they
only use 2 pairs of wires:

  Pin   Color               Function
  ----- ------------------- ----------
  1     White with Green    +TD
  2     Green               -TD
  3     White with Orange   +RD
  4     Blue                Not Used
  5     White with Blue     Not Used
  6     Orange              -RD
  7     White with Brown    Not Used
  8     Brown               Not Used

From the table above, you can see that 10/100BaseT only uses the Green
and Orange twisted pairs of wires. Where TD and RD are transmit and
receive. The +/- is an inverted polarity system to reduce noise and
increase performance. Basically, if +TD is at +5V, then -TD is at 0V.
This mirroring, helps to reduce reduce noise effects that could drown
out a normal signal over long distances.

Now for 1000BaseT, it operates the same way, but gigabit uses all 4
pairs (8 wires) to communicate.

#References

-   [How Gigabit Ethernet Works](http://www.hardwaresecrets.com/how-gigabit-ethernet-works/)
