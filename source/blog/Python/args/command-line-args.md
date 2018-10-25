# Python and Command Line Args

```python
import sys

sys.argv[0]        # program
sys.argv[1]        # arg 1
len(sys.argv) - 1  # number args
```

```python
import argparse

parser = argparse.ArgumentParser(description="This is a cool program")
parser.add_argument("--width", "-w", help="set output width")
parser.add_argument("-V", "--version", help="show program version", action="store_true")
args = parser.parse_args()
```
