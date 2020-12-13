---
title: Wire Current Limits
date: 17 Feb 2020
image: "https://www.pinterest.com/pin/447967494192391995/"
---

## Soldering Wires

Sparkfun has great info on [working with wiring](https://learn.sparkfun.com/tutorials/working-with-wire/all).

![](https://cdn.thingiverse.com/renders/30/f7/d6/21/95/be63798c5a99cbccb5720450970581b3_preview_featured.jpg)

Thingiverse: [3d printed helping hands](https://www.thingiverse.com/thing:2757071)

## Wiring Current Limits

- You can *theoretically* run large amounts of current through a wire until you hit its melting  point (~2000C for copper). However,
there are other limitations like: conductor size, ambient temperature, number of conductors, internal resistance of wire, and insulation.
    - PVC insulation melts around 100C
    - Silicon insulation melts around 200C
- For high current applications (i.e., robotics, quadcopters, etc), use silicone wire which has a 200C rating
    - *Note:* these are about half the Cooner Wire estimates because they are stranded wire.
    - 16 AWG silicone: ~15A
    - 14 AWG silicone: ~20A
    - 12 AWG silicone: ~30A
- Use good connectors like XT30 or XT60 which handle 30A or 60A peak and half that in normal operation respectively
- Also, as the wire gets longer, internal resistance causes both voltage and current losses. However, for robotics
and things I care about, this generally isn't a big deal

# References

- [JST Current Limits for Wires](https://www.jst.fr/doc/jst/pdf/current_rating.pdf)
- [Cooner Wire](https://www.coonerwire.com/amp-chart/)
