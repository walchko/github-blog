---
title: Pandas
date: 8 Feb 2020
---

Not totally sure this `pandas` is useful to me.

```python
import pandas as pd
import numpy as np
from random import randint

# collect data
data = [randint(1,1000) for _ in range(1000)]

# push into pandas
s = pd.Serial(data)
s.describe() # print statisics
s.to_pickle("data.pickle") # save to a file

n = pd.read_pickle("data.pickle") # read back in
```

