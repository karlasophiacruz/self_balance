/* |------------------------------------------------------------------------| */
/* | Projeto: Robô Self-Balancing                                           | */
/* |------------------------------------------------------------------------| */
/* | Curso: Engenharia de Computação										| */
/* | Disciplina: Microcontroladores e Aplicações                            | */
/* | Professor: Erick Barboza                                               | */
/* | Alunos: Bruno Lemos, Karla Sophia, Leticia Gabriela e Maria Fernanda   | */
/* |------------------------------------------------------------------------| */

/* --------------------------Importando Bibliotecas-------------------------- */

#include <Arduino.h>
#include <MsTimer2.h>
#include <EnableInterrupt.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>  
#include <Wire.h>      

/* --------------------------Importando Bibliotecas-------------------------- */

/* --------------------------Inicializando Variáveis------------------------- */

// Inicializando o LCD na porta I2C 0x27 para um display de 16 caracteres e 2 linhas
LiquidCrystal_I2C lcd(0x27,16,2); 

// Instanciando um objeto MPU6050
MPU6050 mpu6050;

// Definindo três variáveis para cada eixo da aceleração e giroscópio
int16_t ax, ay, az, gx, gy, gz;  

// Definindo os pinos do driver do motor (TB6612)
const int right_R1 = 8;    
const int right_R2 = 12;
const int PWM_R = 10;
const int left_L1 = 7;
const int left_L2 = 6;
const int PWM_L = 9;

/* ---------------------------Parâmetros do ângulo--------------------------- */

// Calcula o ângulo de inclinação do eixo X e Y a partir da aceleração
float angle_X;
float angle_Y;

// Ângulo medido pelo acelerômetro (idealmente 0 graus)
float angle0 = 1;

// Velocidade angular medida pelos cálculos do giroscópio
float Gyro_x, Gyro_y, Gyro_z;

/* --------------------Parâmetros do filtro de Kalman------------------------ */

// Covariância do ruído do giroscópio
float Q_angle = 0.001;
// Covariância do ruído de desvio do giroscópio
float Q_gyro = 0.003;  
// Covariância do acelerômetro
float R_angle = 0.5; 

char C_0 = 1;

// Variável dt para o cálculo do tempo de amostragem
float dt = 0.005;
// O ganho de Kalman é usado para calcular o desvio da estimativa ótima
float K1 = 0.05;

// Variáveis para o cálculo do ângulo de inclinação do giroscópio
float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias;

// Variáveis para o cálculo do ângulo de inclinação do acelerômetro
float accelz = 0;
float angle;
float angleY_one;
float angle_speed;

float Pdot[4] = {0, 0, 0, 0};
float P[2][2] = {{1, 0}, {0, 1}};
float  PCt_0, PCt_1, E;
/* --------------------Parâmetros do filtro de Kalman------------------------ */

/* ---------------------------Parâmetros do PID------------------------------ */

// Parâmetros do PID para o ângulo
double kp = 34, ki = 0, kd = 0.62;      

// Parâmetros do PID para a velocidade
double kp_speed = 3.6, ki_speed = 0.080, kd_speed = 0;

// Variáveis do PID para rotação
double kp_turn = 24, ki_turn = 0, kd_turn = 0.08;   

// Ângulo de referência  
double setp0 = 0;

// Ângulo de saída
int PD_pwm;
float pwm1 = 0, pwm2 = 0;
/* ---------------------------Parâmetros do PID------------------------------ */

/* ---------------------------Parâmetros do PI de Velocidade------------------------------ */
float speeds_filterold = 0;
float positions = 0;
int flag1;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;
/* ---------------------------Parâmetros do PI de Velocidade------------------------------ */

/* ---------------------------Parâmetros do PD de Direção------------------------------ */
int turnmax,turnmin,turnout; 
float Turn_pwm = 0;
int zz = 0;
int turncc = 0;
/* ---------------------------Parâmetros do PD de Direção------------------------------ */

/* ---------------------------Variáveis do cálculo de Pulso------------------------------ */
int lz = 0;
int rz = 0;
int rpulse = 0;
int lpulse = 0;
int pulseright, pulseleft;
/* ---------------------------Variáveis do cálculo de Pulso------------------------------ */

/* ---------------------------Parâmetros de Interrupção------------------------------ */
// Interrupções Externas (Comunicação I2C)
#define PinA_left 5 
#define PinA_right 4 

// Variáveis para o cálculo do pulso
volatile long count_right = 0;
volatile long count_left = 0;
int speedcc = 0;

//Contador de tempo do LCD
int lcd_count = 0;
/* ---------------------------Parâmetros de Interrupção------------------------------ */

/* ---------------------------Bluetooth------------------------------ */
// Variáveis para o controle de direção via Bluetooth
int front = 0;
int back = 0;
int left = 0;
int right = 0;
char val;
/* ---------------------------Bluetooth------------------------------ */

/* --------------------------Inicializando Variáveis-------------------------- */

/* --------------------------Configuração Inicial----------------------------- */
void setup() {
    // Inicializando os pinos do driver do motor como saída
    pinMode(right_R1, OUTPUT);       
    pinMode(right_R2, OUTPUT);
    pinMode(left_L1, OUTPUT);
    pinMode(left_L2, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);

    // Valores iniciais dos estados do motor
    digitalWrite(right_R1, 1);
    digitalWrite(right_R2, 0);
    digitalWrite(left_L1, 0);
    digitalWrite(left_L2, 1);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);

    // Inicializando os pinos do encoder como entrada 
    pinMode(PinA_left, INPUT); 
    pinMode(PinA_right, INPUT);

    // Unindo a sequência do barramento I2C
    Wire.begin();                          

    // Inicializando o monitor serial com a taxa de transmissão de 9600 bps
    Serial.begin(9600); 

    delay(1500);

  // Inicializando o MPU6050
    mpu6050.initialize();                      

    delay(2);

    // Inicializando o LCD
    lcd.init();   

    // Ligando o BackLight do LCD           
    lcd.backlight(); 

    // Definindo o tempo de interrupção em 5 ms utilizando o timer2
    // Atribuindo o tempo de interrupção e a função de interrupção
    MsTimer2::set(5, robot_life);   
    // Inicia a interrupção
    MsTimer2::start(); 
}

/* ---------------------------Configuração Inicial---------------------------- */

/* -------------------------------Loop Principal------------------------------ */
void loop() {
    // Checando se o Serial estava disponível
    if(Serial.available()) {
        // Lendo o valor do Serial
        val = Serial.read(); 
        
        // Casos de direção
        switch(val) {            
            case 'F': 
                front = 250; 
                break;       
            case 'B': 
                back = -250; 
                break;       
            case 'L': 
                left = 1; 
                break;   
            case 'R': 
                right = 1; 
                break;                      
            case 'S': 
                front = 0, back = 0, left = 0, right = 0;
                break;    
            case 'D': 
                break;  
        }
    }

    // Interrupção externa usada para calcular a velocidade das rodas
    attachPinChangeInterrupt(PinA_left, Code_left, CHANGE);          
    attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);      
}
/* -------------------------------Loop Principal------------------------------ */


/* --------------------------Cálculo da Velocidade---------------------------- */

/* Função: Code_left
 * Descrição: Conta o número de pulsos do encoder da roda esquerda
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void Code_left() {
    count_left ++;
} 

/* Função: Code_right
 * Descrição: Conta o número de pulsos do encoder da roda direita
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void Code_right() {
    count_right ++;
} 

/* --------------------------Cálculo da Velocidade---------------------------- */

/* --------------------------Cálculo do pulso--------------------------------- */

/* Função: countpulse
 * Descrição: Calcula o número de pulsos do encoder
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void countpulse() {
    // Atribui o valor contado pelo encoder para a variável lz
    lz = count_left;   
    // Atribui o valor contado pelo encoder para a variável rz
    rz = count_right;

    // Zera o contador do encoder
    count_left = 0;    
    count_right = 0;

    lpulse = lz;
    rpulse = rz;

    // Calculando a direção do movimento
    // Se o PWM da roda esquerda for negativo e o PWM da roda direita for negativo, o movimento é para trás
    if ((pwm1 < 0) && (pwm2 < 0)) {           
        rpulse = -rpulse;
        lpulse = -lpulse;
    }
    // Se o PWM da roda esquerda for negativo e o PWM da roda direita for positivo, o movimento é para a esquerda
    else if ((pwm1 < 0) && (pwm2 > 0)) {                //judge the moving direction of balance robot; if turn left, right pulse is positive but left pulse is negative.
        lpulse = -lpulse;
    }
    // Se o PWM da roda esquerda for positivo e o PWM da roda direita for negativo, o movimento é para a direita
    else if ((pwm1 > 0) && (pwm2 < 0)) { 
        rpulse = -rpulse;
    }

    // Entrando na interrupção a cada 5 ms supondo o número de pulsos
    pulseright += rpulse;
    pulseleft += lpulse;
}
/* --------------------------Cálculo do pulso--------------------------------- */


/* --------------------------Cálculo do ângulo PD--------------------------------- */

/* Função: PD_angle
 * Descrição: Calcula o ângulo do robô usando o PD
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void PD_angle() {
    PD_pwm = kp * (angle + angle0) + kd * angle_speed;
}
/* --------------------------Cálculo do ângulo PD--------------------------------- */

/* --------------------------Filtro de Kalman--------------------------------- */

/* Função: Kalman_Filter
 * Descrição: Utiliza o filtro de Kalman para filtrar o ângulo
 * Parâmetros: angle_m, gyro_m
 * Retorno: nenhum
 */
void Kalman_Filter(double angle_m, double gyro_m) {
    // Estimativa prévia
    angle += (gyro_m - q_bias) * dt;          
    angle_err = angle_m - angle;

    // Derivada da estimativa prévia do erro da covariância
    Pdot[0] = Q_angle - P[0][1] - P[1][0];    
    Pdot[1] = - P[1][1];
    Pdot[2] = - P[1][1];
    Pdot[3] = Q_gyro;

    // Integral da estimativa prévia do erro da covariância
    P[0][0] += Pdot[0] * dt;   
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    // Variáveis intermediárias na multiplicação de matrix
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    // Denominador
    E = R_angle + C_0 * PCt_0;
    
    // Valor de ganho
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    // Variáveis intermediárias na multiplicação de matrix
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    // Erro estimativo da covariância
    P[0][0] -= K_0 * t_0;   
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    // Estimativa posterior
    q_bias += K_1 * angle_err;

    // Derivada do valor de saída para pegar velocidade angular ótima
    angle_speed = gyro_m - q_bias;
    angle += K_0 * angle_err;
}
/* --------------------------Filtro de Kalman--------------------------------- */

/* --------------------------Filtro de Primeira Ordem--------------------------------- */

/* Função: Yiorderfilter
 * Descrição: Utiliza o filtro de 1º Ordem para filtrar o ângulo
 * Parâmetros: angle_m, gyro_m
 * Retorno: nenhum
 */
void Yiorderfilter(float angle_m, float gyro_m) {
    angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}
/* --------------------------Filtro de Primeira Ordem--------------------------------- */


/* --------------------------Velocidade PI--------------------------------- */

/* Função: speedpiout
 * Descrição: Calcula a velocidade do robô usando o PI
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void speedpiout() {
    // Valor do pulso da velocidade
    float speeds = (pulseleft + pulseright) * 1.0;    

    pulseright = pulseleft = 0; 

    // Filtro de primeira ordem complementar    
    speeds_filterold *= 0.7;        
    speeds_filter = speeds_filterold + speeds * 0.3;
    speeds_filterold = speeds_filter;
    positions += speeds_filter;

    // Controle para frente
    positions += front;  

    // Controle para trás        
    positions += back;         

    // Saturação Anti-integral     
    positions = constrain(positions, -3550, 3550); 

    // Controle de Velocidade pelo PI
    PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);     
}
/* --------------------------Velocidade PI--------------------------------- */

/* --------------------------Rotação PI--------------------------------- */

/* Função: turnspin
 * Descrição: Calcula a rotação do robô usando o PI
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void turnspin() {
    int flag = 0; 
    float turnspeed = 0;
    float rotationratio = 0;

    if (left == 1 || right == 1) {
        // Checa a velocidade antes de rotacionar
        if (flag == 0) {                          
            turnspeed = (pulseright + pulseleft);                      
            flag = 1;
        }
        // Módulo da velocidade
        if (turnspeed < 0) {                               
            turnspeed = -turnspeed;
        }

        // Se para esquerda ou para direita
        if(left == 1 || right == 1) {       
            turnmax = 3;         
            turnmin = -3;       
        }
        rotationratio = 5 / turnspeed;         
        if (rotationratio < 0.5) {
            rotationratio = 0.5;
        }
            
        if (rotationratio > 5) {
            rotationratio = 5;
        }
    }
    else {
        rotationratio = 0.5;
        flag = 0;
        turnspeed = 0;
    }
    if (left == 1) { 
        turnout += rotationratio;
    }
    else if (right == 1 ) {
        turnout -= rotationratio;
    }
    else turnout = 0;
    if (turnout > turnmax) {
        turnout = turnmax;
    }
    if (turnout < turnmin) {
        turnout = turnmin;
    }

    // O algoritmo de rotação PD controla a velocidade de fusão e a posição de rotação do eixo Z
    Turn_pwm = -turnout * kp_turn - Gyro_z * kd_turn;
}
/* --------------------------Rotação PI--------------------------------- */

/* --------------------------Cálculo do ângulo de inclinação--------------------------------- */

/* Função: angle_calculate
 * Descrição: Calcula o ângulo de inclinação do robô
 * Parâmetros: ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1
 * Retorno: nenhum
 */
void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1) {
    // Calculando a fórmula do ângulo de rotação radial
    // O sinal negativo é o processamento da direção
    float Angle = -atan2(ay , az) * (180 / PI);      

    // Calculando a velocidade angular do eixo X pelo giroscópio  
    // O sinal negativo é o processamento da direção
    Gyro_x = -gx / 131;    

    // Filtro de Kalman          
    Kalman_Filter(Angle, Gyro_x);  

    // Calculando a velocidade angular do eixo Z pelo giroscópio        
    Gyro_z = -gz / 131;                   
 
    // Calculando o ângulo de inclinação do eixo X
    float angleAx = -atan2(ax, az) * (180 / PI); 

    // Velocidade angular do eixo Y do giroscópio
    Gyro_y = -gy / 131.00;

    // Filtro de Primeira Ordem
    Yiorderfilter(angleAx, Gyro_y);
}
/* --------------------------Cálculo do ângulo de inclinação--------------------------------- */

/* --------------------------Envia o ângulo via PWM--------------------------------- */

/* Função: turnspin
 * Descrição: Calcula a rotação do robô usando o PI
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void anglePWM() {
    // Valor final do PWM para os motores
    pwm2 = -PD_pwm - PI_pwm + Turn_pwm;           
    pwm1 = -PD_pwm - PI_pwm - Turn_pwm;

    // Trunca a saída dos PWMs para 255
    if(pwm1 > 255) {            
        pwm1 = 255;
    }
    if(pwm1 < -255) {
        pwm1 = -255;
    }
    if(pwm2 > 255) {
        pwm2 = 255;
    }
    if(pwm2 < -255) {
        pwm2 = -255;
    }

    // Se o ângulo de inclinação estiver muito alto, o robô para
    if(angle > 80 || angle < -80) {     
        pwm1 = pwm2 = 0;
    }

    // A rotação e velocidade dos motores são determinadas pelo sinal positivo ou negativo do PWM
    if(pwm2 >= 0) {        
        digitalWrite(left_L1, LOW);
        digitalWrite(left_L2, HIGH);
        analogWrite(PWM_L, pwm2);
    }
    else {
        digitalWrite(left_L1, HIGH);
        digitalWrite(left_L2, LOW);
        analogWrite(PWM_L, -pwm2);
    }

    if(pwm1 >= 0) {
        digitalWrite(right_R1, LOW);
        digitalWrite(right_R2, HIGH);
        analogWrite(PWM_R, pwm1);
    }
    else {
        digitalWrite(right_R1, HIGH);
        digitalWrite(right_R2, LOW);
        analogWrite(PWM_R, -pwm1);
    }
}
/* --------------------------Envia o ângulo via PWM--------------------------------- */

/* --------------------------Função da Interrupção (Funcionamento geral do robô)--------------------------------- */
/* Função: robot_life
 * Descrição: Funcionamento geral do robô
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void robot_life() {
    // Permite qualquer tipo de interrupção
    sei(); 

    // Calcula o número de pulsos 
    countpulse();

    // Pega o movimento dos dados do MPU6050
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   

    // Cálculo do ângulo utilizando o filtro de Kalmann
    angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);     
    
    // Cálculo do ângulo utilizando o PD
    PD_angle();  

    // Envia o valor da velocidade para o motor em bits       
    anglePWM();

    // A cada 40ms, atualiza o PI de velocidade
    cc++;
    if(cc >= 8) {   
        speedpiout();   
        cc = 0;
    }

    // A cada 20ms, atualiza o PI de posição
    turncc++; 
    if(turncc > 4) {  
        turnspin();
        turncc = 0; 
    }

    // A cada 2s, envia os valores de ângulo e velocidade para o display LCD
    lcd_count++;
    if(lcd_count >= 400) {  
        lcd.clear();

        lcd.setCursor(0,0);
        lcd.print("angle = ");
        lcd.print(angle);

        lcd.setCursor(0,1);
        lcd.print("speed = ");
        lcd.print(speeds_filter);

        lcd_count = 0;
    }
}
/* --------------------------Função da Interrupção (Funcionamento geral do robô)--------------------------------- */

