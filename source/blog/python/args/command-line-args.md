---
title: Python and Command Line Args
---

```python
import sys

sys.argv[0]        # program
sys.argv[1]        # arg 1
len(sys.argv) - 1  # number args
```

```python
import argparse

def handle_args():
    parser = argparse.ArgumentParser(description="This is a cool program")
    parser.add_argument("--width", "-w", help="set output width")
    parser.add_argument("-V", "--version", help="show program version", action="store_true")
    return vars(parser.parse_args())
```
