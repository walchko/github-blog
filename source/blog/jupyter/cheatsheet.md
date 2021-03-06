---
title: Cheatsheet of Useful Jupyter Commands
date: 8 Nov 2020
image: "https://i.pinimg.com/564x/f7/6f/61/f76f612d5c5236232259c991806f3504.jpg"
image-height: "400px"
---

## Reload Imports

```python
# reload library
%load_ext autoreload
%autoreload 2
```

## numpy

```python
import numpy as np                        # matrix manipulations
from numpy.testing import assert_allclose # compare numpy arrays
from numpy.linalg import norm             # normalize numpy vectors
np.set_printoptions(precision=1)
np.set_printoptions(suppress=True)
```

## matplotlib

```python
%matplotlib inline
import matplotlib.pyplot as plt
plt.rcParams["figure.figsize"] = (20,3) # set default figure width
```

## Embed Video

```python
from ipywidgets import Video
Video.from_file("Megamind.mp4")
```

## OpenCV

```python
# simple function to save a video
import platform
def videoWrite(frames, fname='out.mp4'):
    frame_height, frame_width, _ = frames[0].shape
    
    # pick a good encoder for the current OS
    sys = platform.system()
    if sys in ['Darwin']:  # this is on macOS
        fourcc = 'avc1'
    else:  # this is for Windoze
        fourcc = 'mjpg'
        
    out = cv2.VideoWriter(
        fname,
        cv2.VideoWriter_fourcc(*fourcc), 
        30, 
        (frame_width,frame_height))
    for frame in frames:
        out.write(frame)
    out.release()
```