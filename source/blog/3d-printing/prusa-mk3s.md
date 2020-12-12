---
title: Prusa MK3S
date: 18 Aug 2019
---

![](https://cdn.shop.prusa3d.com/1303-home_default/original-prusa-i3-mk3-kit.jpg)

I just got my printer and it works great! (Even 6 months later)

- building and calibration: 1.5 days
    - but I could do it faster next time, this was my first time building a printer kit
- First layer print z-offset: -1.200 mm
- Print volume: 250 x 210 x 210 mm
- Use Octoprint in a docker container
    - I use docker because as of today, it still uses Python 2.7 for some stupid reason ... only had a decade plus to get ready for 3.0 :smile:

## Reviews

[![](https://img.youtube.com/vi/M73uIMDlvvk/0.jpg)](https://www.youtube.com/watch?v=M73uIMDlvvk)

## Thoughts

- Very quiet compared to the Lulzbot TAZ 6 we had at the Air Force Academy
- Far cheaper ($800 US) and just as good as Lulzbot
- Love the removable print bed ... things pop right off it

# Lessons Learned

- z-setting for first layer: -1.200 mm
- sensative to brown outs when printing from Octoprint or the SD card ... will go nuts
    - plugged into powercell helps little power hickups
- Prusa PLA is very temperature sensative
    - Only a concern on robotic parts requiring high mechanical accuracy
    - Setting a 10 mm brim around your parts will help them stick to the bed  better and not warp as much
    - Close air conditioning vents helps too
    - Turn off ceiling fans helps too
    - I haven't gone as far as using a printer enclosure, but that might be the next step if I continue to have problems with parts
        - Nope, above temperature fixes work ... also like to keep door closed during winter :smile:
