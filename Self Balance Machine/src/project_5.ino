// // PROJECT 5 - INTERNAL TIMER INTERRUPT TEST  

// #include <Arduino.h>
// #include <MsTimer2.h>

// //TB6612 pins
// const int right_R1 = 8;    
// const int right_R2 = 12;
// const int PWM_R = 10;
// const int left_L1 = 7;
// const int left_L2 = 6;
// const int PWM_L = 9;

// const int PinA_left = 5;        //set the left motor’s pulse pin to D5
// const int PinA_right = 4;       //set the right motor’s pulse pin to D4

// int times = 0, newtime = 0, d_time = 100;   // time, new time, time interval
// int valA = 0, valB = 0, flagA = 0, flagB = 0;   //variable valA and valB for calculating the number of pulse

// void setup() {
//   Serial.begin(9600);
  
//   pinMode(right_R1, OUTPUT);     //set the TB6612 pins to OUTPUT
//   pinMode(right_R2, OUTPUT);
//   pinMode(PWM_R, OUTPUT);
//   pinMode(left_L1, OUTPUT);
//   pinMode(left_L2, OUTPUT);
//   pinMode(PWM_L, OUTPUT);

//   pinMode(PinA_left, INPUT);      // set the pulse pin to INPUT
//   pinMode(PinA_right, INPUT);
  
//   MsTimer2::set(100, inter);      // trigger an interrupt per 100ms 
//   MsTimer2::start();              // start interrupt
// }

// void loop() {
//   //both motors turn forward
//   digitalWrite(right_R1, HIGH);
//   digitalWrite(right_R2, LOW);
//   digitalWrite(left_L1, HIGH);
//   digitalWrite(left_L2, LOW);
//   analogWrite(PWM_R, 100);    //write into PWM value 0~255（speed）
//   analogWrite(PWM_L, 200);

//   if(digitalRead(PinA_left) == HIGH && flagA == 0) {   // calculate the pulse value
//     valA++;
//     flagA = 1;
//   }
//   if(digitalRead(PinA_left) == LOW && flagA == 1) {
//     valA++;
//     flagA = 0;
//   }
//   if(digitalRead(PinA_right) == HIGH && flagB == 0) {
//     valB++;
//     flagB = 1;
//   }
//   if(digitalRead(PinA_right) == LOW && flagB == 1){
//     valB++;
//     flagB = 0;
//   }  
// }

// //interrupt function
// void inter()    
// {
//     sei();                       //allow whole interrupts
//     Serial.print("valA = ");     //print out the pulse value on serial monitor
//     Serial.println(valA);
//     Serial.print("valB = ");
//     Serial.println(valB);
//     valA = valB = 0;
// }
