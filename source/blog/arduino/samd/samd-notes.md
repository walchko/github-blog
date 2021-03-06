---
title: Adafruit SAMD (Trinket M0 and QT Py) Notes
date: 4 Oct 2020
image: "https://cdn-shop.adafruit.com/970x728/4600-06.jpg"
image-height: "400px"
---

Adafruit and others are using low cost ARM processors for small, cheap
Arduino boards. The SAMD models made by Adafruit have some differences:

- Enable pull-up resistors: `pinMode(pin, INPUT_PULLUP)`
- I2C speed: you can increase the default clock rate with `Wire.setClock(x)`
where `x` can be 100000, 400000, 1000000, or 3400000
    - SAMD is capabile of I2C clock of 3.4MHz ... you are really limited by the sensor

# References

- Arduino: [Changing the Wiring clock rate](https://www.arduino.cc/en/Reference/WireSetClock)
- Microchip: [ATSAMD21E18 specs and I2C clock rate](https://www.microchip.com/wwwproducts/en/ATSAMD21E18#additional-features)
