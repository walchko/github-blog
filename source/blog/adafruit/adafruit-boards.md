---
title: Adafruit Boards
date: 10 Jan 2021
image: "https://i.pinimg.com/564x/92/99/24/929924c948f6ece51e9ea6782f2dbab1.jpg"
image-height: "400px"
---


| Adafruit uC       | Processor                     | MHz | RAM   | ROM   | FPU | GPIO | 12b ADC | PWM | SPI FLASH | Price  |
|-------------------|-------------------------------|-----|-------|-------|-----|------|---------|-----|-----------|--------|
| [Trinket M0][1]   | ATSAMD21E18 32-bit Cortex M0+ | 48  | 32KB  | 256KB | No  | 5    | 3       | 2   | N/A       | $8.95  |
| [ItsyBitsy M0][2] | ATSAMD21G18 32-bit Cortex M0+ | 48  | 32KB  | 256KB | No  | 23   | 11      | 13  | 2MB       | $11.95 |
| [ItsyBitsy M4][3] | ATSAMD51 32-bit Cortex M4     | 120 | 192KB | 512KB | Yes | 23   | 12      | 18  | 2MB       | $14.95 |
| [QTPy][4]         | ATSAMD21E18 32-bit Cortex M0+ | 48  | 32KB  | 256KB | No  | 11   | 9       | 9   | N/A       | $6     |

- FPU: floating point unit

## SAMD21G

- The SAMD21G uses an ARM Cortex M0+ microcontroller, which is about as small and simple as you get with 32-bit devices.
- It runs at 48MHz, usually stabilized by an external 32,768Hz crystal oscillator, and has 38 GPIO pins.
- It has 256K of Flash memory and 32K of RAM. It doesn't have any EEPROM, but you can read and write sections of the Flash array from code.
- Unlike 8-bit microcontrollers, a 32-bit microcontroller can have multiple copies of a peripheral that are called 'instances'.

## SAMD21E

- The SAMD21E is a slightly smaller version of the SADM21G, with 32 GPIO pins.
- It has most of the same specs as the SAMD21G, but:
    - Its advanced counters can generate six PWM signals per instance instead of eight
    - It has four SERCOMs instead of six
    - Its ADC can read 10 pins instead of 14
    - Its PTC can handle a 10x6 matrix instead of 12x10


## SAMD51

The SAMD51 is a new device family from Atmel-now-Microchip, built around a very fast ARM Cortex M4 microcontroller. It's a super-powerful upgrade.
- CircuitPython and Arduino support.
- The '51 is an upgrade to the '21 so there are many similarities. Most code for the '21 will run on a '51, but 6x faster.
- It's more complex and faster than the M0+
- includes a hardware floating-point math unit and digital signal processing array (one command applies a math operation to a whole block of data). 
- It has 512K of Flash memory and 192K of RAM. 
- It runs at 120MHz and has 51 GPIO pins.

# References

- [The microcontrollers in Adafruit products](https://learn.adafruit.com/how-to-choose-a-microcontroller/the-microcontrollers-in-adafruit-products)

[1]: https://www.adafruit.com/product/3500 "buy"
[2]: https://www.adafruit.com/product/3727 "buy"
[3]: https://www.adafruit.com/product/3800 "buy"
[4]: https://www.adafruit.com/product/4600 "buy"
