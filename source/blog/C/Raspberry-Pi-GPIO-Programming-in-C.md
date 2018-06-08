![](pics/Raspberry-GPIO.jpg)

# Raspberry Pi GPIO Programming in C

I was looking around the interweb for how to program the RPi in C and found a good reference.
The reference below does a great job about talking about this. There are 3 libraries
reviewed:

1. [`WiringPi`](http://wiringpi.com/) which uses an Arduino like interface
2. [`pigio`](http://abyz.me.uk/rpi/pigpio/index.html) which is a thin library geared
  towards speed and simplicity
3. [`bcm2835`](http://www.airspayce.com/mikem/bcm2835/) another simple and fast
  library, but with little usage (according to the author)

Of the three, the author leans towards speed/efficiency and suggest `pigop` or
`bcm2835` rahter than the more complex `WiringPi`.

## WiringPi

```c
int digitalRead (int pin)
{
  char c ;
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] == -1)
	return LOW ;

      lseek  (sysFds [pin], 0L, SEEK_SET) ;
      read   (sysFds [pin], &c, 1) ;
      return (c == '0') ? LOW : HIGH ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return LOW ;

    if ((*(gpio + gpioToGPLEV [pin]) & (1 << (pin & 31))) != 0)
      return HIGH ;
    else
      return LOW ;
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead (node, pin) ;
  }
}
```

## pigio

```c
#define BANK (gpio>>5)
#define BIT  (1<<(gpio&0x1F))

int gpioRead(unsigned gpio)
{
   DBG(DBG_USER, "gpio=%d", gpio);

   CHECK_INITED;

   if (gpio > PI_MAX_GPIO)
      SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

   if ((*(gpioReg + GPLEV0 + BANK) & BIT) != 0) return PI_ON;
   else                                         return PI_OFF;
}
```

## bcm2835

```c
uint32_t bcm2835_peri_read(volatile uint32_t* paddr)
{
    uint32_t ret;
    if (debug)
    {
		printf("bcm2835_peri_read  paddr %p\n", (void *) paddr);
		return 0;
    }
    else
    {
       __sync_synchronize();
       ret = *paddr;
       __sync_synchronize();
       return ret;
    }
}

uint8_t bcm2835_gpio_lev(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPLEV0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = bcm2835_peri_read(paddr);
    return (value & (1 << shift)) ? HIGH : LOW;
}
```

# References

- [Raspberry Pi GPIO Programming in C](https://www.bigmessowires.com/2018/05/26/raspberry-pi-gpio-programming-in-c/)
