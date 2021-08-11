// OLED_I2C_Scrolling_Text
// Copyright (C)2018 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// **********************************************
// *** This demo is for 128x32 pixel displays ***
// **********************************************
//
// A quick demo of how to use my OLED_I2C library.
//
// To use the hardware I2C (TWI) interface of the chipKit you must connect
// the pins as follows:
//
// chipKit Uno32/uC32:
// ----------------------
// Display:  SDA pin   -> Analog 4
//           SCL pin   -> Analog 5
// *** Please note that JP6 and JP8 must be in the I2C position (closest to the analog pins)
//
// chipKit Max32:
// ----------------------
// Display:  SDA pin   -> Digital 20 (the pin labeled SDA)
//           SCL pin   -> Digital 21 (the pin labeled SCL)
//
// The chipKit boards does not have pull-up resistors on the hardware I2C interface
// so external pull-up resistors on the data and clock signals are required.
//
// You can connect the OLED display to any available pin but if you use any
// other than what is described above the library will fall back to
// a software-based, TWI-like protocol which will require exclusive access 
// to the pins used.
//

#include <OLED_I2C.h>

OLED  myOLED(SDA, SCL); // Remember to add the RESET pin if your display requires it...

extern uint8_t SmallFont[];

void setup()
{
  if(!myOLED.begin(SSD1306_128X32))
    while(1);   // In case the library failed to allocate enough RAM for the display buffer...

  myOLED.setFont(SmallFont);
  randomSeed(analogRead(0));
}

void loop()
{
  int y = random(0, myOLED.getDisplayHeight()-8);
  for (int i=128; i>=-(34*6); i--)
  {
    myOLED.print("OLED_I2C Scrolling Text Demo ", i, y);
    myOLED.update();
    delay(50);
  }
}


