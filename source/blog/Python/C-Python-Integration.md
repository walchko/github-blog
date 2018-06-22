# C and Python Integration

| Approach	          | Vintage |	Representative User	| Notable Pros                                          | Notable Cons |
|---------------------|---------|---------------------|-------------------------------------------------------|--------------|
| C extension modules |	1991    | Standard library    |	Extensive docs/tutorials. Total control and fast.     |	Compilation, portability, reference management. High C knowledge. |
| SWIG                | 1996    | crfsuite	          | Generate bindings for many languages at once          | Excessive overhead if Python is the only target. |
| ctypes              | 2003    | oscrypto            | No compilation, wide availability                     | Accessing and mutating C structures cumbersome, high overhead, and error prone. |
| Cython              | 2007    | gevent, kivy        | Python-like. Highly mature. High performance.	        | Compilation, new syntax and toolchain. |
| cffi                | 2013    | cryptography, pypy  | Ease of integration, PyPy compatibility               | New/High-velocity. |

- [Paypal blog](https://www.paypal-engineering.com/2016/09/22/python-by-the-c-side/)

## Performance

- [ctypes example](http://www.dalkescientific.com/writings/NBN/c_extensions.html)

```python
import math, time, ctypes

R = range(100000)

libc = ctypes.CDLL("libc.dylib", ctypes.RTLD_GLOBAL)
libc.cos.argtypes = [ctypes.c_double]
libc.cos.restype = ctypes.c_double

def do_timings(cos):
  t1 = time.time()
  for x in R:
    cos(0.0)
  return time.time()-t1

def do_math_cos():
  return do_timings(math.cos)

def do_libc_cos():
  return do_timings(libc.cos)

print "math.cos", do_math_cos()
print "libc.cos", do_libc_cos()
```

```bash
math.cos 0.179316997528
libc.cos 0.487237215042
```

# References

- [ctypes tutorial](https://pgi-jcns.fz-juelich.de/portal/pages/using-c-from-python.html)
- [Python 3 ctypes](https://docs.python.org/3/library/ctypes.html)
- [another](http://intermediate-and-advanced-software-carpentry.readthedocs.io/en/latest/c++-wrapping.html)
