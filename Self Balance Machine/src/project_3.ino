// // PROJECT 3 - TB6612 MOTOR DRIVING TEST

// #include <Arduino.h>

// //TB6612 pins
// const int right_R1 = 8;    
// const int right_R2 = 12;
// const int PWM_R = 10;
// const int left_L1 = 7;
// const int left_L2 = 6;
// const int PWM_L = 9;

// void setup() {
//   Serial.begin(9600);          //set the baud rate to 9600
  
//   pinMode(right_R1, OUTPUT);     // set all TB6612pins to OUTPUT
//   pinMode(right_R2, OUTPUT);
//   pinMode(PWM_R, OUTPUT);
//   pinMode(left_L1, OUTPUT);
//   pinMode(left_L2, OUTPUT);
//   pinMode(PWM_L, OUTPUT);
// }

// void loop() {
//   // two motors turn forward
//   digitalWrite(right_R1, HIGH);
//   digitalWrite(right_R2, LOW);
//   digitalWrite(left_L1, HIGH);
//   digitalWrite(left_L2, LOW);
//   analogWrite(PWM_R, 100);   // write into PWM value 0~255（speed）
//   analogWrite(PWM_L, 100);
// }