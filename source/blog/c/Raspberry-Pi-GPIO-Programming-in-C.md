---
title: Raspberry Pi GPIO Programming in C
---

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

## Linux

```c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#define ADXL345_I2C_ADDR 0x53

/*
gcc -o adxl345 adxl345.c
*/

int selectDevice(int fd, int addr, char *name)
{
   int s;
   char str[128];

    s = ioctl(fd, I2C_SLAVE, addr);

    if (s == -1)
    {
       sprintf(str, "selectDevice for %s", name);
       perror(str);
    }

    return s;
}

int writeToDevice(int fd, int reg, int val)
{
   int s;
   char buf[2];

   buf[0]=reg; buf[1]=val;

   s = write(fd, buf, 2);

   if (s == -1)
   {
      perror("writeToDevice");
   }
   else if (s != 2)
   {
      fprintf(stderr, "short write to device\n");
   }
}

int main(int argc, char **argv)
{
   unsigned int range;
   int bus;
   int count, b;
   short x, y, z;
   float xa, ya, za;
   int fd;
   unsigned char buf[16];

   if (argc > 1) bus = atoi(argv[1]);
   else bus = 0;

   sprintf(buf, "/dev/i2c-%d", bus);

   if ((fd = open(buf, O_RDWR)) < 0)
   {
      // Open port for reading and writing

      fprintf(stderr, "Failed to open i2c bus /dev/i2c-%d\n", bus);

      exit(1);
   }

   /* initialise ADXL345 */

   selectDevice(fd, ADXL345_I2C_ADDR, "ADXL345");

   writeToDevice(fd, 0x2d, 0);  /* POWER_CTL reset */
   writeToDevice(fd, 0x2d, 8);  /* POWER_CTL measure */
   writeToDevice(fd, 0x31, 0);  /* DATA_FORMAT reset */
   writeToDevice(fd, 0x31, 11); /* DATA_FORMAT full res +/- 16g */

   while (1)
   {   
      buf[0] = 0x32;

      if ((write(fd, buf, 1)) != 1)
      {
         // Send the register to read from

         fprintf(stderr, "Error writing to ADXL345\n");
      }

      if (read(fd, buf, 6) != 6)
      {
         //  X, Y, Z accelerations

         fprintf(stderr, "Error reading from ADXL345\n");
      }
      else
      {
         x = buf[1]<<8| buf[0];
         y = buf[3]<<8| buf[2];
         z = buf[5]<<8| buf[4];
         xa = (90.0 / 256.0) * (float) x;
         ya = (90.0 / 256.0) * (float) y;
         za = (90.0 / 256.0) * (float) z;
         printf("%4.0f %4.0f %4.0f\n", xa, ya, za);
      }
      usleep(10000);
   }

   return 0;
}
```

```c
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

int file = -1;

if ((file=open("/dev/i2c_1", O_RDWR))<0){
	printf("ERROR openning I2C\n");
	exit(1);
}

if (ioctl(file, I2C_SLAVE, HMC5883) < 0) {
	printf("Can not set device address %d\n", HMC5883);
	return;
}
if (i2c_smbus_write_byte_data(file, 0x02, 0) < 0) {
	printf("i2c write failed\n"); // mode register, continuous measurement mode
}
if (i2c_smbus_write_byte_data(file, 0x01, 0x20) < 0) {
	print("i2c write failed\n"); // configuration register B, set range to +/-1.3Ga (gain 1090)
}

int read_magnetometer(float &x, float &y, float &z) {
    if (ioctl(file, I2C_SLAVE, HMC5883) < 0) {
        printf("Can not set device address %d", HMC5883);
        return 0;
    }

    uint8_t buf[6] = {0};
    i2c_smbus_read_i2c_block_data(file, 0x03, 6, buf);

    uint16_t ux = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]); // big endian [hi_bit, low_bit, hi_bit, low_bit ...]
    uint16_t uz = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);
    uint16_t uy = ((uint16_t)buf[4] << 8) | ((uint16_t)buf[5]);

    int16_t xraw = *(int16_t *)(void *)&ux; // why?
    int16_t yraw = *(int16_t *)(void *)&uy;
    int16_t zraw = *(int16_t *)(void *)&uz;

    x = float(xraw)/1090. - offset_x;
    y = float(yraw)/1090. - offset_y;
    z = float(zraw)/1090. - offset_z;

    return 1;
}

int get_heading(float &heading) {
    float x, y, z;
    int res = read_magnetometer(x, y, z);
    if (!res) return false;

    heading = atan2(y, x);
    if (heading < 0     ) heading += 2*M_PI;
    if (heading > 2*M_PI) heading -= 2*M_PI;
    heading = heading*180/M_PI;
    return 1;
}
```

# References

- [Raspberry Pi GPIO Programming in C](https://www.bigmessowires.com/2018/05/26/raspberry-pi-gpio-programming-in-c/)
- [IMU C++ driver](https://github.com/ssloy/quadruped/blob/master/gy-85.cpp)
