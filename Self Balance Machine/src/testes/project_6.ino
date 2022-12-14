// // PROJECT 6 - BLUETOOTH TEST  

// #include <Arduino.h>

// //TB6612 pins
// const int right_R1 = 8;    
// const int right_R2 = 12;
// const int PWM_R = 10;
// const int left_L1 = 7;
// const int left_L2 = 6;
// const int PWM_L = 9;
// char val;   // Bluetooth variable

// void setup() {
//   Serial.begin(9600);
  
//   pinMode(right_R1, OUTPUT);   //set TB6612 pins OUTPUT
//   pinMode(right_R2, OUTPUT);
//   pinMode(PWM_R, OUTPUT);
//   pinMode(left_L1, OUTPUT);
//   pinMode(left_L2, OUTPUT);
//   pinMode(PWM_L, OUTPUT);
// }

// void loop() {
//   if(Serial.available()) {    //if serial buffer value is available
//     val = Serial.read();      //assign the value read from serial port to val
//     Serial.println(val);
//     switch(val) {             //switch statement
//       case 'F': 
//         front(); 
//         break;     //motor turns front
//       case 'B': 
//         back(); 
//         break;       //turn back
//       case 'S': 
//         Stop();
//         break;    //stop
//     }
//   }
// }

// //turn front
// void front() {
//   digitalWrite(right_R1, HIGH);
//   digitalWrite(right_R2, LOW);
//   digitalWrite(left_L1, HIGH);
//   digitalWrite(left_L2, LOW);
//   analogWrite(PWM_R, 100);
//   analogWrite(PWM_L, 100);
// }
// //turn back
// void back() {
//   digitalWrite(right_R1, LOW);
//   digitalWrite(right_R2, HIGH);
//   digitalWrite(left_L1, LOW);
//   digitalWrite(left_L2, HIGH);
//   analogWrite(PWM_R, 100);
//   analogWrite(PWM_L, 100);
// }
// //stop
// void Stop() {
//   digitalWrite(right_R1, LOW);
//   digitalWrite(right_R2, HIGH);
//   digitalWrite(left_L1, LOW);
//   digitalWrite(left_L2, HIGH);
//   analogWrite(PWM_R, 0);
//   analogWrite(PWM_L, 0);
// }