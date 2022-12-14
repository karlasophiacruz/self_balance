// #include <Wire.h>
// #include "Arduino.h"

// void setup()
// {
//   Serial.begin(9600);
//   Wire.begin();
  
//   byte Return;
  
//   Serial.println("Scanning I2C bus...");
//   for(byte I2CAddress = 1; I2CAddress <= 127; I2CAddress++)
//   {
//     Serial.print("0x");
//     if (I2CAddress<16)
//       Serial.print("0");
//     Serial.print(I2CAddress, HEX);
//     Serial.print(" (");
//     if (I2CAddress<10)
//       Serial.print("  ");
//     else if (I2CAddress<100)
//       Serial.print(" ");
//     Serial.print(I2CAddress);
//     Serial.print("): ");
    
//     Wire.beginTransmission(I2CAddress);
//     Return = Wire.endTransmission();
    
//     if (Return == 0)
//       Serial.print("OK!");
//     else
//       Serial.print("   ");
      
//     if (I2CAddress % 5)
//       Serial.print("    ");
//     else
//       Serial.println();
//   }
// }

// void loop() { }