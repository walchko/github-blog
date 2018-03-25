<img src="pics/python-snake.jpg" width="100%">

# Temporary Files and Folders

Create a simple temp file:

```python

	import os
	import tempfile

	print
	print 'TemporaryFile:'
	temp = tempfile.TemporaryFile()
	try:
		print 'temp:', temp
		print 'temp.name:', temp.name
	finally:
		# Automatically cleans up the file
		temp.close()
```

Create a temp folder:

```python

	import os
	import tempfile

	directory_name = tempfile.mkdtemp()
	print directory_name

	# Clean up the directory yourself
	os.removedirs(directory_name)
```

- More [tempfile examples](https://pymotw.com/2/tempfile/)
