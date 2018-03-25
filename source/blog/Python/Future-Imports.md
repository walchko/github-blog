# The Future is Now

The future library backports some potentially useful python3 code to python2:

```python

	from __future__ import print_function # changes print to a function instead of a statement
	from __future__ import division       # all division is floating point

	print('hello!')  # now a function

	1/3   # result is 0.333, float division ... similar to other languages
	10//3 # result is 3, this is integer division
```

Other capabilities can be found in the docs.

# References

- `future docs <https://docs.python.org/2/library/__future__.html>`_
