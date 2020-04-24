---
title: Simple OpenCV Camera in Python
date: 21 Dec 2019
---

```python
#!/usr/bin/env python

import numpy as np
import cv2
from matplotlib import pyplot as plt
import time

cam = cv2.VideoCapture(0)
# print(dir(cam))
# print(help(cam.getBackendName))
if not cam.isOpened():
    print("... ops, failed to open camera")
    exit(1)

# print(cam.getBackendName())
while True:
    ok, frame = cam.read()
    if ok:
        # cv2.imshow('movie', frame)
        k = cv2.waitKey(1)
        if k == ord('q'):
            cam.release()            # close camera
            cv2.destroyAllWindows()  # clean up GUI
            exit()
        time.sleep(0.033)
```
