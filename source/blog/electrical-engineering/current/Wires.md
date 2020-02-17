---
title: Wires
date: 17 Feb 2020
---

## Soldering Wires

Sparkfun has great info on [working with wiring](https://learn.sparkfun.com/tutorials/working-with-wire/all).

![](https://cdn.thingiverse.com/renders/30/f7/d6/21/95/be63798c5a99cbccb5720450970581b3_preview_featured.jpg)
[3d printed helping hands](https://www.thingiverse.com/thing:2757071)

## Wiring Current Limits

[Cooner Wire](https://www.coonerwire.com/amp-chart/) has a good site to understand current limits. Most places only
spec solid core wire, so for stranded wire cut it in half.

- For high current applications (i.e., robotics, quadcopters, etc), use silicone wire which has a 200 deg C rating
    - *Note:* these are about half the Cooner Wire estimates because they are stranded wire.
    - 16 AWG silicone: ~15A
    - 14 AWG silicone: ~20A
    - 12 AWG silicone: ~30A
- Use good connectors like XT30 or XT60 which handle 30A or 60A peak and half that in normal operation respectively
- Also, as the wire gets longer, internal resistance causes both voltage and current losses. However, for robotics
and things I care about, this generally isn't a big deal
