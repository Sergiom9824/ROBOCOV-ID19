#include <PID_v1.h>
#include "MeanFilterLib.h"
#include <TimerOne.h>
#include <LiquidCrystal.h>

// Specification
const uint16_t cuentas_max = 3400;
const float Pi = 3.141592;
const float dt = 2.5; // ms

// Mean Filter
MeanFilter<float> meanFilter(10);
double velocidadf = 0;

// Pines
int Pin_encoder_A = 2, Pin_encoder_B = 3;
int PWM_R = 6, PWM_L= 5;
int rs = 8, en = 9, d4 = 4, d5 = 10, d6 = 11, d7 = 7;

// Encoder
volatile int32_t cuentas = 0;
volatile int8_t last_state = 0, current_state = 0;

// Position
float current_position = 0.0, last_position = 0.0;
float position_max= 0.436, position_min = 0, position_des = 0.0;
float error_position = 0.0;

// Velocidad
float velocidad=0.0;

// Control PID
float velocidad_max = 2.9544, velocidad_min = -0.363, velocidad_des = 0.0; 
float Kp = 5 , Ki = 17.5, Kd = 0;
double Setpoint = 0.0, PWM = 0;

PID VelPID(&velocidadf, &PWM, &Setpoint, Kp, Ki, Kd, DIRECT);
int U =0;

// LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() 
{
    TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    VelPID.SetMode(AUTOMATIC);
    Serial.begin(9600); 
    pinMode(Pin_encoder_A,INPUT); // Pin A
    pinMode(Pin_encoder_B,INPUT); // Pin B
    pinMode(PWM_R,OUTPUT); // Pin PWM1
    pinMode(PWM_L,OUTPUT); // Pin PWM2
    attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), Lectura_Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), Lectura_Encoder, CHANGE);
    Timer1.initialize(dt*1000); 
    Timer1.attachInterrupt(ISR_Timer) ; 
    lcd.begin(16, 2);
    lcd.print("Ventilador UTEC"); 
    lcd.setCursor(0, 1);
    lcd.print("BPM:40,Ratio:1:4");
}

void loop()
{   
    Serial.println(current_position);
}

void ISR_Timer()
{   
    Setpoint = abs(velocidad_des);
    VelPID.SetTunings(Kp,Ki,Kd);
    VelPID.Compute();
    U = PWM;

    if(velocidad_des >= 0){
        analogWrite(PWM_L,U);
        analogWrite(PWM_R,0);
    }
    else{
        analogWrite(PWM_L,U);
        analogWrite(PWM_R,255);
    }
    
    current_position = cuentas*2*Pi/cuentas_max; 
    velocidad = ((current_position - last_position)/(dt)*1000);
    velocidad = meanFilter.AddValue(velocidad);
    velocidadf = abs(velocidad);
    
    PosVelcontrol();
    
    last_position = current_position;
}
void PosVelcontrol()
{
  error_position = position_des - current_position;
  if (error_position > 0){
    position_des = position_max; 
    velocidad_des = velocidad_max;
  }
  else{
    position_des = position_min;
    velocidad_des = velocidad_min;
  }
}

void Lectura_Encoder()
{    
    if(digitalRead(Pin_encoder_A)==1)
      bitSet(current_state,1);
    else
      bitClear(current_state,1);
     
    if(digitalRead(Pin_encoder_B)==1)
      bitSet(current_state,0);
    else
      bitClear(current_state,0);
  
    if(last_state==3 && current_state==1)
      cuentas++;
    if(last_state==1 && current_state==0) 
      cuentas++;
    if(last_state==0 && current_state==2)
      cuentas++;
    if(last_state==2 && current_state==3)
      cuentas++;   
       
    if(last_state==3 && current_state==2)
      cuentas--;
    if(last_state==2 && current_state==0)
      cuentas--;
    if(last_state==0 && current_state==1)
      cuentas--;
    if(last_state==1 && current_state==3)
      cuentas--;
      
    last_state = current_state; 
}
