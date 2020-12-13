---
title: Trinket M0 and Serial Ports
date: 19 Sept 2020
---

You can use the Trinket M0 as a simple USB to serial converter using the Arduino
IDE.

- USB serial: `Serial`
- Hardware serial (pins 3 and 4): `Serial1`


```c++
#include <stdint.h>

#define usleep delayMicroseconds
#define msleep delay

#define PIN 1
#define DD_WRITE HIGH
#define DD_READ  LOW

void setup(){
    Serial.begin(9600);
    Serial1.begin(1000000); // AX-12 Dynamixel servos
    pinMode(PIN, OUTPUT);
}

void loop(){
    uint8_t buffer[] = {255,255,1,5,3,30,255,1,216}; // middle position
    
    Serial.println("hello");
    
    digitalWrite(PIN, DD_WRITE);
    Serial.write(buffer, 9); // send command
    usleep(500);
    digitalWrite(PIN, DD_READ);
    
    msleep(1000);
}
```


![](https://cdn-learn.adafruit.com/assets/assets/000/049/778/large1024/adafruit_products_Adafruit_Trinket_M0.png?1514756138)
