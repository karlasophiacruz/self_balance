// // PROJECT 7 - MPU6050 TEST  

// #include <Arduino.h>
// #include <Wire.h>
 
// long accelX, accelY, accelZ;      // set to overall variable; can be used directly inside the function.
// float gForceX, gForceY, gForceZ;
 
// long gyroX, gyroY, gyroZ;
// float rotX, rotY, rotZ;
 
// void setup() {
//   Serial.begin(9600);
//   Wire.begin();
//   setupMPU();
// }
 
// void loop() {
//   recordAccelRegisters();
//   recordGyroRegisters();
//   printData();
//   delay(100);
// }
 
// void setupMPU() {
//   // REGISTER 0x6B/REGISTER 107:Power Management 1
//   Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet Sec. 9.2)
//   Wire.write(0x6B); //Accessing the register 6B/107 - Power Management (Sec. 4.30) 
//   Wire.write(0b00000000); //Setting SLEEP register to 0, using the internal 8 Mhz oscillator
//   Wire.endTransmission();
 
//   // REGISTER 0x1b/REGISTER 27:Gyroscope Configuration
//   Wire.beginTransmission(0b1101000); //I2C address of the MPU
//   Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
//   Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s (转化为rpm:250/360 * 60 = 41.67rpm) ;The highest can be converted to 2000deg./s 
//   Wire.endTransmission();
  
//   // REGISTER 0x1C/REGISTER 28:ACCELEROMETER CONFIGURATION
//   Wire.beginTransmission(0b1101000); //I2C address of the MPU
//   Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
//   Wire.write(0b00000000); //Setting the accel to +/- 2g（if choose +/- 16g，the value would be 0b00011000）
//   Wire.endTransmission(); 
// }
 
// void recordAccelRegisters() {
//   // REGISTER 0x3B~0x40/REGISTER 59~64
//   Wire.beginTransmission(0b1101000); //I2C address of the MPU
//   Wire.write(0x3B); //Starting register for Accel Readings
//   Wire.endTransmission();
//   Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
 
//   // Use left shift << and bit operations |  Wire.read() read once 1bytes，and automatically read the data of the next address on the next call.
//   while(Wire.available() < 6);  // Waiting for all the 6 bytes data to be sent from the slave machine （Must wait for all data to be stored in the buffer before reading） 
//   accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX （Automatically stored as a defined long value）
//   accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
//   accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
//   processAccelData();
// }
 
// void processAccelData() {
//   gForceX = accelX / 16384.0;     //float = long / float
//   gForceY = accelY / 16384.0; 
//   gForceZ = accelZ / 16384.0;
// }
 
// void recordGyroRegisters() {
//   // REGISTER 0x43~0x48/REGISTER 67~72
//   Wire.beginTransmission(0b1101000); //I2C address of the MPU
//   Wire.write(0x43); //Starting register for Gyro Readings
//   Wire.endTransmission();
//   Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 ~ 48)
//   while(Wire.available() < 6);
//   gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
//   gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
//   gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
//   processGyroData();
// }
 
// void processGyroData() {
//   rotX = gyroX / 131.0;
//   rotY = gyroY / 131.0; 
//   rotZ = gyroZ / 131.0;
// }
 
// void printData() {
//   Serial.print("Gyro (deg)");
//   Serial.print(" X=");
//   Serial.print(rotX);
//   Serial.print(" Y=");
//   Serial.print(rotY);
//   Serial.print(" Z=");
//   Serial.print(rotZ);
//   Serial.print(" Accel (g)");
//   Serial.print(" X=");
//   Serial.print(gForceX);
//   Serial.print(" Y=");
//   Serial.print(gForceY);
//   Serial.print(" Z=");
//   Serial.println(gForceZ);
// }