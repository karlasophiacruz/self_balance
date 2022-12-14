// // PROJECT 13 - BLUEETOOH CONTROL WITH ADJUSTING BALANCE ANGLE AND PID

// #include <Arduino.h>
// #include <MsTimer2.h>        //internal timer 2
// // #include <PinChangeInt.h>    //this library can make all pins of arduino REV4 as external interrupt
// #include <EnableInterrupt.h>
// #include <LiquidCrystal_I2C.h>
// #include <MPU6050.h>      //MPU6050 library
// #include <Wire.h>        //IIC communication library

// LiquidCrystal_I2C lcd(0x27,16,2);  // Criando um LCD de 16x2 no endereço 0x20

// MPU6050 mpu6050;     //Instantiate an MPU6050 object; name mpu6050
// int16_t ax, ay, az, gx, gy, gz;     //Define three-axis acceleration, three-axis gyroscope variables

// //TB6612 pins
// const int right_R1 = 8;    
// const int right_R2 = 12;
// const int PWM_R = 10;
// const int left_L1 = 7;
// const int left_L2 = 6;
// const int PWM_L = 9;

// ///////////////////////angle parameters//////////////////////////////
// float angle_X; //calculate the inclined angle variable of X-axis by accelerometer
// float angle_Y; //calculate the inclined angle variable of Y-axis by accelerometer
// int angle0 = 0; //Actual measured angle (ideally 0 degrees) 
// float Gyro_x, Gyro_y, Gyro_z;  //Angular angular velocity for gyroscope calculation 
// ///////////////////////angle parameters//////////////////////////////

// ///////////////////////Kalman_Filter////////////////////////////
// float Q_angle = 0.001;  //Covariance of gyroscope noise
// float Q_gyro = 0.003;    //Covariance of gyroscope drift noise
// float R_angle = 0.5;    //Covariance of accelerometer
// char C_0 = 1;
// float dt = 0.005; //The value of dt is the filter sampling time
// float K1 = 0.05; // a function containing the Kalman gain; used to calculate the deviation of the optimal estimate
// float K_0, K_1, t_0, t_1;
// float angle_err;
// float q_bias;    //gyroscope drift

// float accelz = 0;
// float angle;
// float angleY_one;
// float angle_speed;

// float Pdot[4] = {0, 0, 0, 0};
// float P[2][2] = {{1, 0}, {0, 1}};
// float  PCt_0, PCt_1, E;
// //////////////////////Kalman_Filter/////////////////////////

// //////////////////////PID parameters///////////////////////////////
// double kp = 34.00, ki = 0, kd = 0.62;                   //angle loop parameter
// double kp_speed = 3.60, ki_speed = 0.08, kd_speed = 0;   // speed loop parameter
// double kp_turn = 24.00, ki_turn = 0, kd_turn = 0.08;                 // steering loop parameter
// double setp0 = 0; //angle balance point
// int PD_pwm;  //angle output
// float pwm1 = 0, pwm2 = 0;

// //////////////////interrupt speed count/////////////////////////////
// #define PinA_left 5  //external interrupt
// #define PinA_right 4   //external interrupt
// volatile long count_right = 0;//Used to calculate the pulse value calculated by the Hall encoder (the volatile long type is to ensure that the value is valid）
// volatile long count_left = 0;
// int speedcc = 0;
// //////////////////////pulse count/////////////////////////
// int lz = 0;
// int rz = 0;
// int rpluse = 0;
// int lpluse = 0;
// int pulseright, pulseleft;
// ////////////////////////////////PI variable parameter//////////////////////////
// float speeds_filterold = 0;
// float positions = 0;
// int flag1;
// double PI_pwm;
// int cc;
// int speedout;
// float speeds_filter;

// //////////////////////////////steering PD///////////////////
// int turnmax, turnmin, turnout; 
// float Turn_pwm = 0;
// int zz = 0;
// int turncc = 0;

// //Bluetooth//
// int front = 0;//backward variable
// int back = 0;//forward variable
// int left = 0;//turn left
// int right = 0;//turn right
// char val;

// int TT;

// int lcd_cont = 0;

// void setup() {
//   //set the control pins of motor to OUTPUT
//   pinMode(right_R1, OUTPUT);       
//   pinMode(right_R2, OUTPUT);
//   pinMode(left_L1, OUTPUT);
//   pinMode(left_L2, OUTPUT);
//   pinMode(PWM_R, OUTPUT);
//   pinMode(PWM_L, OUTPUT);

//   //assign the initial value
//   digitalWrite(right_R1, 1);
//   digitalWrite(right_R2, 0);
//   digitalWrite(left_L1, 0);
//   digitalWrite(left_L2, 1);
//   analogWrite(PWM_R, 0);
//   analogWrite(PWM_L, 0);

//   pinMode(PinA_left, INPUT);  //speed encoder input
//   pinMode(PinA_right, INPUT);

//   // join I2C bus
//   Wire.begin();                            //join I2C bus sequence
//   Serial.begin(9600);                       //open the serial monitor, set the baud rate to 9600
//   delay(1500);
//   mpu6050.initialize();                       //initialize MPU6050
//   delay(2);
//   lcd.init();                 // Inicializando o LCD
//   lcd.backlight();            // Ligando o BackLight do LCD
//   delay(2);

//   //5ms; use timer2 to set timer interruption (note：using timer2 will affect the PWM output of pin3 pin11)
//   MsTimer2::set(5, DSzhongduan);    //5ms;  execute the function DSzhongduan once
//   MsTimer2::start();    //start interrupt
// }

// void loop() {
//   //Serial.println(angle);
//   //delay(100);
//   //Serial.println(PD_pwm);
//   //Serial.println(pwm1);
//   //Serial.println(pwm2);
//   //Serial.print("pulseright = ");
//   //Serial.println(pulseright);
//   //Serial.print("pulseleft = ");
//   //Serial.println(pulseleft);
//   //Serial.println(PI_pwm);
//   //Serial.println(speeds_filter);
//   //Serial.println (positions);
//   //Serial.println(Turn_pwm);
//   //Serial.println(Gyro_z);
//   //Serial.println(Turn_pwm);
  

//   if(Serial.available()) {
//     val = Serial.read();      //assign the value read from serial port to val
//     //Serial.println(val);
//     switch(val) {            //switch statement
//       case 'F': 
//         front = 250; 
//         break;       //if vale equals F，front=250，go front
//       case 'B': 
//         back = -250; 
//         break;       //go back
//       case 'L': 
//         left = 1; 
//         break;    //turn left
//       case 'R': 
//         right = 1; 
//         break;                         //turn right
//       case 'S': 
//         front = 0, back = 0, left = 0, right = 0;
//         break;    //stop
//       case 'Q': 
//         Serial.print(angle);
//         break;    //receiving‘D’，send the value of variable angle to APP
//       case 'P': 
//         kp = kp + 0.5, Serial.print(kp);
//         break;
//       case 'O': 
//         kp = kp - 0.5, Serial.print(kp);
//         break;
//       case 'I': 
//         kd = kd + 0.02, Serial.print(kd);
//         break;
//       case 'U': 
//         kd = kd - 0.02, Serial.print(kd);
//         break;
//       case 'Y': 
//         kp_speed = kp_speed + 0.05, Serial.print(kp_speed);
//         break;
//       case 'T': 
//         kp_speed = kp_speed - 0.05, Serial.print(kp_speed);
//         break;
//       case 'G': 
//         ki_speed = ki_speed + 0.01, Serial.print(ki_speed);
//         break;
//       case 'H': 
//         ki_speed = ki_speed - 0.01, Serial.print(ki_speed);
//         break;
//       case 'J': 
//         kp_turn = kp_turn + 0.4, Serial.print(kp_turn);
//         break;
//       case 'K': 
//         kp_turn = kp_turn - 0.4, Serial.print(kp_turn);
//         break;
//       case 'N': 
//         kd_turn = kd_turn + 0.01, Serial.print(kd_turn);
//         break;
//       case 'M': 
//         kd_turn = kd_turn - 0.01, Serial.print(kd_turn);
//         break;
//     }
//     if(val == 'F' || val == 'B' || val == 'L' || val == 'R' || val == 'S' || val == 'Q' || val == 'P' || val == 'O' || val == 'I' || 
//         val == 'U' || val == 'Y' || val == 'T' || val == 'G' || val == 'H' || val == 'J' || val == 'K' || val == 'N' || val == 'M'){
//       TT = angle0;
//     }
//     else {
//       TT = val;
//       angle0 = TT;
//     }
//     //Serial.println(angle0); 
//   }
  
//   //external interrupt; used to calculate the wheel speed
//   attachPinChangeInterrupt(PinA_left, Code_left, CHANGE);          //PinA_left Level change triggers external interrupt; execute subfunction Code_left
//   attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);       //PinA_right Level change triggers external interrupt; execute Code_right
// }

// /////////////////////Hall count/////////////////////////
// //left speed encoder count
// void Code_left() {
//   count_left ++;
// } 
// //right speed encoder count
// void Code_right() {
//   count_right ++;
// } 
// ////////////////////pulse count///////////////////////
// void countpluse() {
//   lz = count_left;     //assign the value counted by encoder to lz
//   rz = count_right;

//   count_left = 0;     //Clear the count quantity
//   count_right = 0;

//   lpluse = lz;
//   rpluse = rz;

//   if ((pwm1 < 0) && (pwm2 < 0)) {                    //judge the car’s moving direction; if backward (PWM /motor voltage is negative), pulse is a negative number.
//     rpluse = -rpluse;
//     lpluse = -lpluse;
//   }
//   else if ((pwm1 > 0) && (pwm2 > 0)) {                //if backward (PWM /motor voltage is positive), pulse is a positive number.
//     rpluse = rpluse;
//     lpluse = lpluse;
//   }
//   else if ((pwm1 < 0) && (pwm2 > 0)) {                //judge the car’s moving direction; if turn left, the right pulse is positive; left pulse is negative. 
//     rpluse = rpluse;
//     lpluse = -lpluse;
//   }
//   else if ((pwm1 > 0) && (pwm2 < 0)) {              //judge the car’s moving direction; if turn right, the right pulse is negative ; left pulse is positive. 
//     rpluse = -rpluse;
//     lpluse = lpluse;
//   }

//   //entering interrupt per，pulse plus
//   pulseright += rpluse;
//   pulseleft += lpluse;
// }

// //////////////////angle PD////////////////////
// void PD_angle() {
//   PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD angle loop control
// }

// /////////////////////////////////interrupt////////////////////////////
// void DSzhongduan() {
//   sei();  //allow overall interrupt
//   countpluse();        //pulse plus subfunction
//   mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data ax ay az gx gy gz
//   angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);      //get angle and Kalman filtering
//   PD_angle();         // PD control of angle loop
//   anglePWM();

//   cc++;
//   if(cc >= 8) {    //5*8=40，that is, execute the PI calculation of speed once per 40ms
//     speedpiout();   
//     cc = 0;  //Clear
//   }
//   turncc++;         
//   if(turncc > 4) {      //20ms, that is, execute the PD calculation of steering once per 40ms
//     turnspin();
//     turncc = 0;     //Clear
//   }
//   lcd_cont++;
//   if(lcd_cont >= 200) {    //5*200=1000，1000ms entering once LCD display
//     lcd_cont = 0;  //Clear
//     lcd.clear();
//     lcd.setCursor(0,0);
//     lcd.print("a:");
//     lcd.print(angle);
//     lcd.print(" s:");
//     lcd.print(speeds_filter);
//     lcd.setCursor(0,1);
//     // lcd.print("i");
//     lcd.print(ki);
//     lcd.print(" ");
//     lcd.print(kp);
//     lcd.print(" ");
//     lcd.print(kd);
    
//   }
// }
// ///////////////////////////////////////////////////////////

// /////////////////////////////tilt angle calculation ///////////////////////
// void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1) {
//   float Angle = -atan2(ay , az) * (180 / PI);           //Radial rotation angle calculation formula ; negative sign is direction processing
//   Gyro_x = -gx / 131;              //The X-axis angular velocity calculated by the gyroscope ; the negative sign is the direction processing
//   Kalman_Filter(Angle, Gyro_x);            //Kalman Filter
//   //Rotation angle Z-axis parameter 
//   Gyro_z = -gz / 131;                      //angle speed of Z-axis
//   //accelz = az / 16.4;

//   float angleAx = -atan2(ax, az) * (180 / PI); //calculate the inclined angle of X-axis
//   Gyro_y = -gy / 131.00; //angle speed of Y-axis 
//   Yiorderfilter(angleAx, Gyro_y); //first-order filtering
// }
// ////////////////////////////////////////////////////////////////

// ///////////////////////////////KalmanFilter/////////////////////
// void Kalman_Filter(double angle_m, double gyro_m) {
//   angle += (gyro_m - q_bias) * dt;          //prior estimate
//   angle_err = angle_m - angle;
  
//   Pdot[0] = Q_angle - P[0][1] - P[1][0];    //The differential of the covariance of the prior estimate error
//   Pdot[1] = - P[1][1];
//   Pdot[2] = - P[1][1];
//   Pdot[3] = Q_gyro;
  
//   P[0][0] += Pdot[0] * dt;    //A priori estimation error covariance differential integral
//   P[0][1] += Pdot[1] * dt;
//   P[1][0] += Pdot[2] * dt;
//   P[1][1] += Pdot[3] * dt;
  
//   //Intermediate variables in matrix multiplication
//   PCt_0 = C_0 * P[0][0];
//   PCt_1 = C_0 * P[1][0];
//   //denominator
//   E = R_angle + C_0 * PCt_0;
//   //gain value
//   K_0 = PCt_0 / E;
//   K_1 = PCt_1 / E;
  
//   t_0 = PCt_0;  //Intermediate variables in matrix multiplication
//   t_1 = C_0 * P[0][1];
  
//   P[0][0] -= K_0 * t_0;    //Posterior estimation error covariance 
//   P[0][1] -= K_0 * t_1;
//   P[1][0] -= K_1 * t_0;
//   P[1][1] -= K_1 * t_1;
  
//   q_bias += K_1 * angle_err;    //Posterior estimate
//   angle_speed = gyro_m - q_bias;   //The differential of the output value; get the optimal angular velocity
//   angle += K_0 * angle_err; ////Posterior estimation; get the optimal angle
// }

// /////////////////////first-order filtering/////////////////
// void Yiorderfilter(float angle_m, float gyro_m) {
//   angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
// }

// //////////////////speed PI////////////////////
// void speedpiout() {
//   float speeds = (pulseleft + pulseright) * 1.0;      //speed; pulse value
//   pulseright = pulseleft = 0;      //Clear
//   speeds_filterold *= 0.7;         //first-order complementary filtering
//   speeds_filter = speeds_filterold + speeds * 0.3;
//   speeds_filterold = speeds_filter;
//   positions += speeds_filter;
//   positions += front;             //Forward control fusion
//   positions += back;              //Backward control fusion
//   positions = constrain(positions, -3550, 3550);    //Anti-integral saturation
//   PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);      //speed loop control PI
// }
// //////////////////speed PI////////////////////

// ///////////////////////////turning/////////////////////////////////
// void turnspin() {
//   int flag = 0;      //
//   float turnspeed = 0;
//   float rotationratio = 0;
  
//   if (left == 1 || right == 1) {
//     if (flag == 0) {                            //judge the current speed before turning, to increase the flexibility.   
//       turnspeed = (pulseright + pulseleft);                      //current speed; pulse expression 
//       flag = 1;
//     }
//     if (turnspeed < 0) {                                //absolute value of current speed
//       turnspeed = -turnspeed;
//     }
//     if(left == 1 || right == 1) {         //if press the left key or right key
//      turnmax = 5;          //the maximum value of turning
//      turnmin = -5;         //the minimum value of turning
//     }
//     rotationratio = 5 / turnspeed;          //set by the speed of car
//     if (rotationratio < 0.5) {
//       rotationratio = 0.5;
//     }
     
//     if (rotationratio > 5) {
//       rotationratio = 5;
//     }
//   }
//   else {
//     rotationratio = 0.5;
//     flag = 0;
//     turnspeed = 0;
//   }
//   if (left == 1) {//add according to orientation parameter
//     turnout += rotationratio;
//   }
//   else if (right == 1 ) {//add according to orientation parameter
//     turnout -= rotationratio;
//   }
//   else turnout = 0;
//   if (turnout > turnmax) {
//     turnout = turnmax;//the max value setting of amplitude
//   }
//   if (turnout < turnmin) {
//     turnout = turnmin;//the min value setting of amplitude 
//   }

//   Turn_pwm = -turnout * kp_turn - Gyro_z * kd_turn;//The rotation PD algorithm controls the fusion speed and Z axis rotation positioning
// }
// ///////////////////////////steering/////////////////////////////////

// ////////////////////////////PWM end value/////////////////////////////
// void anglePWM() {
//   pwm2 = -PD_pwm - PI_pwm + Turn_pwm;           //assign the end value of PWM to motor
//   pwm1 = -PD_pwm - PI_pwm - Turn_pwm;
  
//   if(pwm1 > 255) {            //limit PWM value not more than 255
//     pwm1 = 255;
//   }
//   if(pwm1 < -255) {
//     pwm1 = -255;
//   }
//   if(pwm2 > 255) {
//     pwm2 = 255;
//   }
//   if(pwm2 < -255) {
//     pwm2 = -255;
//   }

//   if(angle > 80 || angle < -80) {     // if the tilt angle is greater than 45°，motor will stop turning.
//     pwm1 = pwm2 = 0;
//   }

//  if(pwm2 >= 0) {        //determine the motor’s turning and speed by the negative and positive of PWM
//     digitalWrite(left_L1, LOW);
//     digitalWrite(left_L2, HIGH);
//     analogWrite(PWM_L, pwm2);
//   }
//   else {
//     digitalWrite(left_L1, HIGH);
//     digitalWrite(left_L2, LOW);
//     analogWrite(PWM_L, -pwm2);
//   }

//   if(pwm1 >= 0) {
//     digitalWrite(right_R1, LOW);
//     digitalWrite(right_R2, HIGH);
//     analogWrite(PWM_R, pwm1);
//   }
//   else {
//     digitalWrite(right_R1, HIGH);
//     digitalWrite(right_R2, LOW);
//     analogWrite(PWM_R, -pwm1);
//   }
// }