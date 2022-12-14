// PROJECT 11 - SPEED LOOP ADJUSTMENT

#include <Arduino.h>
#include <MsTimer2.h>        //internal timer 2
//#include <PinChangeInt.h>    //This library file can make all pins on the REV4 board as external interrupts.
#include <EnableInterrupt.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>      //MPU6050 library
#include <Wire.h>        //IIC library

LiquidCrystal_I2C lcd(0x27,16,2);  // Criando um LCD de 16x2 no endereço 0x20

MPU6050 mpu6050;     //Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz;     // Define three-axis acceleration, three-axis gyroscope variables

//TB6612 pins
const int right_R1 = 8;    
const int right_R2 = 12;
const int PWM_R = 10;
const int left_L1 = 7;
const int left_L2 = 6;
const int PWM_L = 9;

///////////////////////angle parameters//////////////////////////////
float angle_X; //Calculate the tilt angle variable about the X axis from the acceleration
float angle_Y; //Calculate the tilt angle variable about the Y axis from the acceleration
float angle0 = 1; //Actual measured angle (ideally 0 degrees)
float Gyro_x, Gyro_y, Gyro_z;  //Angular angular velocity by gyroscope calculation
///////////////////////angle parameters//////////////////////////////

///////////////////////Kalman_Filter////////////////////////////
float Q_angle = 0.001;  //Covariance of gyroscope noise
float Q_gyro = 0.003;    //Covariance of gyroscope drift noise
float R_angle = 0.5;    //Covariance of accelerometer
char C_0 = 1;
float dt = 0.005; // The value of dt is the filter sampling time. 
float K1 = 0.05; //a function containing the Kalman gain is used to calculate the deviation of the optimal estimate 
float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias;    //Gyro drift

float accelz = 0;
float angle;
float angleY_one;
float angle_speed;

float Pdot[4] = {0, 0, 0, 0};
float P[2][2] = {{1, 0}, {0, 1}};
float  PCt_0, PCt_1, E;
//////////////////////Kalman_Filter/////////////////////////

//////////////////////PID parameters///////////////////////////////
double kp = 34, ki = 0, kd = 0.62;                   //angle loop parameters
double kp_speed = 3.6, ki_speed = 0.080, kd_speed = 0;   // speed loop parameters
double setp0 = 0; //angle balance point
int PD_pwm;  //angle output
float pwm1 = 0, pwm2 = 0;

//////////////////Interrupt speed measurement/////////////////////////////
#define PinA_left 5  //external interrupts
#define PinA_right 4   //external interrupts
volatile long count_right = 0;//Used to calculate the pulse value calculated by the Hall encoder (the volatile long type is to ensure the value is valid)
volatile long count_left = 0;
int speedcc = 0;
//////////////////////pulse calculation/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int pulseright, pulseleft;
////////////////////////////////PI variable parameters//////////////////////////
float speeds_filterold = 0;
float positions = 0;
int flag1;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;

int lcd_cont = 0;

void setup() {
    // set the pins of motor to OUTPUT
    pinMode(right_R1, OUTPUT);       
    pinMode(right_R2, OUTPUT);
    pinMode(left_L1, OUTPUT);
    pinMode(left_L2, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);

    //assign initial state value
    digitalWrite(right_R1, 1);
    digitalWrite(right_R2, 0);
    digitalWrite(left_L1, 0);
    digitalWrite(left_L2, 1);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);

    pinMode(PinA_left, INPUT);  //speed code wheel input
    pinMode(PinA_right, INPUT);

    // join I2C bus
    Wire.begin();                            //join I2C bus sequence
    Serial.begin(9600);                       //open the serial monitor to set the baud rate to 9600
    delay(1500);
    mpu6050.initialize();                       //initialize MPU6050
    delay(2);
    lcd.init();                 // Inicializando o LCD
    lcd.backlight();            // Ligando o BackLight do LCD

    //5ms; use timer2 to set timer interruption (note：using timer2 will affect the PWM output of pin3 pin11)
    MsTimer2::set(5, DSzhongduan);    //5ms ; execute the function DSzhongduan once
    MsTimer2::start();    //start interrupt
}

void loop() {
    // lcd.setBacklight(HIGH);
    // lcd.setCursor(1,0);
    // lcd.print("angle = ");
    // lcd.print(angle);
    //delay(100);
    //Serial.println(PD_pwm);
    //Serial.println(pwm1);
    //Serial.println(pwm2);
    //Serial.print("pulseright = ");
    //Serial.println(pulseright);
    //Serial.print("pulseleft = ");
    //Serial.println(pulseleft);
    //Serial.println(PI_pwm);
    //Serial.println(speeds_filter);
    //Serial.println (positions);

    //External interrupt for calculating wheel speed 
    attachPinChangeInterrupt(PinA_left, Code_left, CHANGE);          //PinA_left Level change triggers external interrupt; execute subfunction Code_left
    attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);       //PinA_right Level change triggers external interrupt; execute subfunction Code_right
}

/////////////////////Hall calculation/////////////////////////
//left speed code wheel count
void Code_left() {
  count_left ++;
} 
//Right speed code wheel count
void Code_right() {
  count_right ++;
} 
////////////////////pulse calculation///////////////////////
void countpluse() {
  lz = count_left;     //Assign the value counted by the code wheel to lz
  rz = count_right;

  count_left = 0;     //Clear the code counter count
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((pwm1 < 0) && (pwm2 < 0)) {                    //judge the moving direction; if backwards（PWM, namely motor voltage is negative）, pulse number is a negative number
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 > 0)) {                // if backwards（PWM, namely motor voltage is positive）, pulse number is a positive number
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((pwm1 < 0) && (pwm2 > 0)) {                //Judge turning direction of the car;  turn left; Right pulse number is a positive number; Left pulse number is a negative number.
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 < 0)) {              //Judge turning direction of the car;  turn right; Right pulse number is a negative number; Left pulse number is a positive number.
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  //enter interrupts per 5ms; pulse number superposes
  pulseright += rpluse;
  pulseleft += lpluse;
}

//////////////////angle PD////////////////////
void PD_angle() {
  PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD angle loop control
}

/////////////////////////////////interrupts////////////////////////////
void DSzhongduan() {
  sei();  //Allow global interrupts
  countpluse();        //Pulse superposition subfunction
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data  ax ay az gx gy gz
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);      //get angle and Kalman filtering
  PD_angle();         //angle loop PD control
  anglePWM();

  cc++;
  lcd_cont++;
  if(cc >= 8) {    //5*8=40，40ms entering once speed PI algorithm  
    speedpiout();   
    cc = 0;  //Clear
  }
  if(lcd_cont >= 200) {    //5*200=1000，1000ms entering once LCD display
    lcd_cont = 0;  //Clear
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("angle = ");
    lcd.print(angle);
    lcd.setCursor(0,1);
    lcd.print("speed = ");
    lcd.print(speeds_filter);
  }
}
///////////////////////////////////////////////////////////

/////////////////////////////angle calculation///////////////////////
void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1) {
  float Angle = -atan2(ay , az) * (180 / PI);           //Radial rotation angle calculation formula; negative sign is direction processing
  Gyro_x = -gx / 131;              //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);            //Kalman Filtering
  //Rotation angle Z-axis parameter
  Gyro_z = -gz / 131;                      //Z-axis angular velocity
  //accelz = az / 16.4;

  float angleAx = -atan2(ax, az) * (180 / PI); //Calculate the angle with the x-axis
  Gyro_y = -gy / 131.00; //Y-axis angular velocity
  Yiorderfilter(angleAx, Gyro_y); //first-order filtering
}
////////////////////////////////////////////////////////////////

///////////////////////////////KalmanFilter/////////////////////
void Kalman_Filter(double angle_m, double gyro_m) {
  angle += (gyro_m - q_bias) * dt;          //Prior estimate
  angle_err = angle_m - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //Differential of azimuth error covariance
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //A priori estimation error covariance differential integral
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //Intermediate variable of matrix multiplication 
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //Denominator
  E = R_angle + C_0 * PCt_0;
  //gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  //Intermediate variable of matrix multiplication
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    //Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    //Posterior estimate 
  angle_speed = gyro_m - q_bias;   //The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err; ////Posterior estimation to get the optimal angle
}

/////////////////////first-order filtering/////////////////
void Yiorderfilter(float angle_m, float gyro_m){
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

//////////////////speed PI////////////////////
void speedpiout(){
  float speeds = (pulseleft + pulseright) * 1.0;      //Vehicle speed  pulse value
  pulseright = pulseleft = 0;      //Clear
  speeds_filterold *= 0.7;         //first-order complementary filtering
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -3550,3550);    //Anti-integral saturation
  PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);      //speed loop control PI
}
//////////////////speed PI////////////////////


////////////////////////////PWM end value/////////////////////////////
void anglePWM(){
  pwm2 =-PD_pwm - PI_pwm ;           //assign the final value of PWM to motor 
  pwm1 =-PD_pwm - PI_pwm ;
  
  if(pwm1 > 255) {             //limit PWM value not greater than 255
    pwm1 = 255;
  }
  if(pwm1< -255) {
    pwm1 = -255;
  }
  if(pwm2 > 255) {
    pwm2 = 255;
  }
  if(pwm2 < -255) {
    pwm2 = -255;
  }

  if(angle > 80 || angle < -80) {     // the inclined angle of balance car is greater than 45°, motor will stop. 
    pwm1 = pwm2 = 0;
  }

 if(pwm2>=0) {        // determine the motor’s steering and speed according to the positive and negative of PWM
    digitalWrite(left_L1, LOW);
    digitalWrite(left_L2, HIGH);
    analogWrite(PWM_L, pwm2);
  }
  else {
    digitalWrite(left_L1, HIGH);
    digitalWrite(left_L2, LOW);
    analogWrite(PWM_L, -pwm2);
  }

  if(pwm1 >= 0){
    digitalWrite(right_R1, LOW);
    digitalWrite(right_R2, HIGH);
    analogWrite(PWM_R, pwm1);
  }
  else{
    digitalWrite(right_R1, HIGH);
    digitalWrite(right_R2, LOW);
    analogWrite(PWM_R, -pwm1);
  }
}