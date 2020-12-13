---
title: I2C Bicolor LED Matrix
date: 2 Jan 2020
image: "https://cdn-shop.adafruit.com/970x728/902-00.jpg"
---

I2C address range is 0x70 - 0x77 with 0x70 being the default value for a single matrix.

Install the following in your `library` folder:

- adafruit_ht16k33
- adafruit_bus_device
- adafruit_register

```python
import board
import busio as io

i2c = io.I2C(board.SCL, board.SDA)
import adafruit_ht16k33.matrix
matrix = adafruit_ht16k33.matrix.Matrix8x8x2(i2c) # single 8x8 bicolor LEDs
# matrix = adafruit_ht16k33.matrix.Matrix8x8(i2c, address=0x74)  # if you need to change the address

matrix.brightness = 10  # 0-15

matrix.fill(0) # off
matrix.fill(1) # green
matrix.fill(2) # red
matrix.fill(3) # yellow

matrix.auto_write = False # must manually tell the matrix to update
matrix[2,5] = 1 # led (2,5) is now green
matrix.show()

matrix.auto_write = True  # turn it back on

# 0 = no blinking
# 1 = fast blinking (~once a 1/2 second)
# 2 = moderate blinking (~once a second)
# 3 = slow blinking (~once every 2 seconds)

matrix.blink_rate = 0 # no blinking

```

# References

- Adafruit LED Backpack [Tutorial](https://learn.adafruit.com/micropython-hardware-led-backpacks-and-featherwings?view=all)
- Github: [ht16k33.py](https://github.com/adafruit/Adafruit_CircuitPython_HT16K33/blob/master/adafruit_ht16k33/ht16k33.py)
- Github: [matrix.py](https://github.com/adafruit/Adafruit_CircuitPython_HT16K33/blob/master/adafruit_ht16k33/matrix.py)
