---
title: Using RFID Tags
date: 18 Jan 2021
image: "https://is2-ssl.mzstatic.com/image/thumb/Purple114/v4/13/8f/d6/138fd652-7192-8d81-2f1d-f0e723086ff8/AppIcon-0-1x_U007emarketing-0-5-0-85-220.png/1200x630wa.png"
image-height: "400px"
---

You can also program these (and others you can get off Amazon) with an iPhone 7 or newer. Use
the NXP app.

- [Adafruit ST25DV16K I2C RFID EEPROM Breakout - STEMMA QT / Qwiic](https://www.adafruit.com/product/4701)
    - 16 kbit EEPROM
    - 40 year data duration
    - 1 million writes cycles at 25C
- [13.56MHz RFID/NFC White Tag - NTAG203 Chip](https://www.adafruit.com/product/4033)
    - datasheet: [NTAG203](https://cdn-shop.adafruit.com/product-files/4033/P4033_datasheet_NTAG_203.pdf)
- NTAG 213/215/216
    - NXP [website](https://www.nxp.com/products/rfid-nfc/nfc-hf/ntag-for-tags-labels/ntag-213-215-216-nfc-forum-type-2-tag-compliant-ic-with-144-504-888-bytes-user-memory:NTAG213_215_216)
    - Operating frequency of 13.56 MHz
    - Data transfer of 106 kbit/s
    - Data integrity of 16-bit CRC, parity, bit coding, bit counting
    - Operating distance up to 100 mm (depending on various parameters as, e.g., field strength and antenna geometry)
    - Data retention time of 10 years
    - 144, 504 or 888 bytes freely available user Read/Write area
    - Write endurance 100,000 cycles
- [ISO/IEC 15693](https://en.wikipedia.org/wiki/ISO/IEC_15693)
    - ISO/IEC 15693, is an ISO standard for vicinity cards, i.e. cards which can be read from a greater distance as 
    compared with proximity cards. Such cards can normally be read out by a reader without being powered themselves, 
    as the reader will supply the necessary power to the card over the air (wireless). ISO/IEC 15693 systems operate 
    at the 13.56 MHz frequency, and offer maximum read distance of 1–1.5 meters. As the vicinity cards have to 
    operate at a greater distance, the necessary magnetic field is less (0.15 to 5 A/m) than that for a proximity 
    card (1.5 to 7.5 A/m).

## Simple Arduino Example for the ST25DV16K

```c++
#include "ST25DVSensor.h"

void setup() {
  const char uri_write_message[] = "Hello world!";       // Uri message to write in the tag
  const char uri_write_protocol[] = URI_ID_0x01_STRING; // Uri protocol to write in the tag
  String uri_read;
  
  Serial.begin(115200);

  // Using Wire, so don't have to pass an instance of it.
  // No clue what the two 0's do ... you don't seem to need them
  if(st25dv.begin(0, 0) == 0) {
    Serial.println("System Init done!");
  } 
  
  // write
  if(st25dv.writeURI(uri_write_protocol, uri_write_message, "")) {
    Serial.println("Write failed!");
    while(1);
  }
  
  char* s = "hello there also";
  NDEF_WriteText(s);
  NDEF_WriteText("Or this way");
  
  // read
  if(st25dv.readURI(&uri_read)) {
    Serial.println("Read failed!");
    while(1);
  }

}
```
