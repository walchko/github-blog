---
title: Compare File Saving
date: 2017-08-09
---

Saving data to disk has several options:

- shelve
- pickle
- gzip
- json

The problem with shelve is it is not cross platform. I had issues moving shelve
between macOS and Linux. Linux uses the bsd database and setting up python to
use it on macOS was a pain.

The simple test below compares pickle, pickle with gzip, and json. Json was the
the easiest and performed just as well.

```bash
./test.py
test.pickle: 2128 kb
test2.pickle: 105 kb
test3.json: 105 kb
------------------------------
 a: 10000
 c: 30000
 b: 20000
 d: 2000000
```


```python
#!/usr/bin/env python

from __future__ import print_function
try:
	import cPickle as pickle
except:
	import pickle
import gzip
import os
import simplejson as json

class Database(object):
	"""
	simple pickle
	"""
	def __init__(self, filename):
		self.filename = filename

	def write(self, data, protocol=pickle.HIGHEST_PROTOCOL):
		with open(self.filename, 'wb') as f:
			f.write(pickle.dumps(data, protocol))

	def read(self):
		with open(self.filename, 'rb') as f:
			data = f.read()
		data = pickle.loads(data)
		return data

	def size(self):
		size = os.path.getsize(self.filename)//(2**10)
		print('{}: {} kb'.format(self.filename, size))


class Database2(object):
	"""
	gzipped pickle
	"""
	def __init__(self, filename):
		self.filename = filename

	def write(self, data, protocol=pickle.HIGHEST_PROTOCOL):
		with gzip.open(self.filename, 'wb') as f:
			f.write(pickle.dumps(data, protocol))

	def read(self):
		with gzip.open(self.filename, 'rb') as f:
			data = f.read()
		data = pickle.loads(data)
		return data

	def size(self):
		size = os.path.getsize(self.filename)//(2**10)
		print('{}: {} kb'.format(self.filename, size))


class Database3(object):
	"""use json"""
	def __init__(self, filename):
		self.filename = filename

	def write(self, data):
		with open(self.filename, 'wb') as f:
				json.dump(data, f)

	def read(self):
		with open(self.filename, 'rb') as f:
			data = json.load(f)
		self.db = data
		return len(self.db), data

	def size(self):
		size = os.path.getsize(self.filename)//(2**10)
		print('{}: {} kb'.format(self.filename, size))


if __name__ == "__main__":
	db = Database('test.pickle')
	db2 = Database2('test2.pickle')
	db3 = Database2('test3.json')

	a = {}
	a['a'] = range(10000)
	a['b'] = range(20000)
	a['c'] = range(30000)
	a['d'] = bytearray(2000000)

	db.write(a)
	db.size()
	db2.write(a)
	db2.size()
	db3.write(a)
	db3.size()

	b = db3.read()
	print('-'*30)
	for key in b.keys():
		print(' {}: {}'.format(key, len(b[key])))
```
