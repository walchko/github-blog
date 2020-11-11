---
title: Cheatsheet of Useful Jupyter Commands
date: 8 Nov 2020
---

## Reload Imports

```python
# reload library
%load_ext autoreload
%autoreload 2
```

## numpy

```python
import numpy as np # matrix manipulations
from numpy.testing import assert_allclose
np.set_printoptions(precision=1)
np.set_printoptions(suppress=True)
```

## matplotlib

```python
%matplotlib inline
from matplotlib import pyplot as plt
```

## Embed Video

```python
from ipywidgets import Video
Video.from_file("Megamind.mp4")
```
