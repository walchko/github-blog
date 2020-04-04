---
title: Pandas
date: 8 Feb 2020
---

Here are some simple `pandas` examples. It is useful for:

- statistical understanding of lots of data
- `pandas.DataFrames` can be plotted using `matlibplot` or `seaborn`
    - `geopandas` is a derived class of `pandas` for GIS work
- useful for converting data in *object oriented* format
to:
    - *dict* 
    - `CSV`
    - saving to `pickle`

```python
import pandas as pd
import numpy as np
from random import randint

# collect data
data = {
    0: {'name': "bob", "age": 33},
    1: {'name': "bobby", "age": 43},
    2: {'name': "big bob", "age": 53}
}

df = pd.DataFrame(data)     # import object oriented data, wrong order
df = df.transpose()         # re-order the data to work for pandas
df.head()                   # markdown table of data
df.describe()               # print statisics
df.to_pickle("data.pickle") # save to a file

n = pd.read_pickle("data.pickle") # read back in
```

```python
import pandas as pd

cars = {'Brand': ['Honda Civic','Toyota Corolla','Ford Focus','Audi A4'],
        'Price': [22000,25000,27000,35000]
        }

df = pd.DataFrame(cars, columns= ['Brand', 'Price'])
df.head()

# ouput is a nice table

# ----------------------------------------------------
df.describe()

# output describes the columns mean, std, and other values

# ----------------------------------------------------
df.to_dict()

# output:
# {'Brand': {0: 'Honda Civic',
#   1: 'Toyota Corolla',
#   2: 'Ford Focus',
#   3: 'Audi A4'},
#  'Price': {0: 22000, 1: 25000, 2: 27000, 3: 35000}}

# ----------------------------------------------------
# create a cvs file
# path: filename, if none given, just returns a string
# header: column names
# index: the row numbers, 0,1,2,3 ...
s = df.to_csv(header=True, index=False)
print(s)

# output:
# Brand,Price
# Honda Civic,22000
# Toyota Corolla,25000
# Ford Focus,27000
# Audi A4,35000

# ----------------------------------------------------
# normal mode breaks up objects in columns and
# mixes them all together. This keeps each object
# together in a single "class object". df.transpose()
# allows you to convert between these 2 orientations.
dft = df.transpose()
dft.to_dict()

# output:
# {0: {'Brand': 'Honda Civic', 'Price': 22000},
#  1: {'Brand': 'Toyota Corolla', 'Price': 25000},
#  2: {'Brand': 'Ford Focus', 'Price': 27000},
#  3: {'Brand': 'Audi A4', 'Price': 35000}}


```
