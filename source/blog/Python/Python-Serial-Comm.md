<img src="pics/python-snake.jpg" width="100%">

# Serial Comm

A simple python serial example is:

```python

	import serial

	# s = serial.Serial('/dev/ttyS0', 19200, timeout=1)
	# or
	s = serial.Serial()
	s.baudrate = 19200
	s.port = '/dev/serial0'
	s.timeout = 1  # will wait for 1 second and return
	s.open()
	s.write('hello')
	s.close()
```

- [pyserial docs](http://pyserial.readthedocs.io)
