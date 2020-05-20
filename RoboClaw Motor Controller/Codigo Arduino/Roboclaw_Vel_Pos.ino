#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Keypad.h>
#include "RoboClaw.h"
 
// LCD(rs, en, d4, d5, d6, d7)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Teclado matricial
const byte rowsCount = 4;
const byte columsCount = 4;

char keys[rowsCount][columsCount] = {
   { '1','2','3', 'A' },
   { '4','5','6', 'B' },
   { '7','8','9', 'C' },
   { '#','0','*', 'D' }
};
const byte rowPins[rowsCount] = { 13, 12, 3, A4};
const byte columnPins[columsCount] = { A0, A1, A2, A3 };
 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, columnPins, rowsCount, columsCount);

// Driver de motor
SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);
#define address 0x80

// Variables
int32_t motor_1_count;
int32_t velmotor1;
float posicion_ang; float vel_ang;
const float Pi = 3.141592;
int Habilitardor = 0;
int Contador = 0;
bool HabilitadorA = 0;
bool HabilitadorB = 0;
int cuentas_motor = 5264;
//Variables de control
int Ratio = 1; int BPM = 4;
int vel_insp = 0; int vel_exp = 0;
int posicion = 0; 
float periodo;
float T_in; float T_ex; 
float angulo=25.0;

void setup() {
    roboclaw.begin(38400);
    Serial.begin(9600);
    lcd.begin(16, 2);
    lcd.print("Ventilador UTEC"); 
    lcd.setCursor(0, 1);
    lcd.print("BPM:40,Ratio:1:1");
    motor_1_count = roboclaw.ResetEncoders(address);
    attachInterrupt(digitalPinToInterrupt(2), Final_de_Carrera, RISING) ;

}

void loop() {
//  motor_1_count = roboclaw.ReadEncM1(address);
//  posicion_ang = (motor_1_count*2*Pi)/5264;
//  velmotor1 = roboclaw.ReadSpeedM1(address);
//  vel_ang = (velmotor1*2*Pi)/5264;
  char key = keypad.getKey();
    //Serial.println(Ratio);
    //Serial.println(BPM);
  Serial.print("Periodo: ");
  Serial.println(periodo);
  Serial.print("T_ins: ");
  Serial.println(T_in);
  Serial.print("T_ex: "); 
  Serial.println(T_ex);
  Serial.print("Posicion: ");   
  Serial.println(posicion);
  Serial.print("Vel_insp: ");
  Serial.println(vel_insp);
  Serial.print("Vel_exp: ");
  Serial.println(vel_exp);
  Serial.print("Habilitador: ");
  Serial.println(Habilitardor);

  if (Habilitardor == 1)
  {
    roboclaw.SpeedAccelDeccelPositionM1(address, 655360, vel_insp, 655360, posicion, 0);
    roboclaw.SpeedAccelDeccelPositionM1(address, 655360, vel_exp, 655360, 0, 0);
  }

  if (key == 'A'){
    if(HabilitadorA == 0)
      HabilitadorA = 1;
    else if(HabilitadorA == 1)
      HabilitadorA = 0;
   }
   
  else if (key == 'B'){  
    if(HabilitadorB == 0)
      HabilitadorB = 1;
    else if(HabilitadorB == 1)
      HabilitadorB = 0;
  }
  
  else if (key == 'C'){
    Habilitardor = 0;  
    Contador = 0;
    roboclaw.BackwardM1(address,64);
  }
  
  else if (key == 'D') {
    if (digitalRead(2)== LOW){
      periodo = 6.0/BPM;
      T_in = periodo/(1+Ratio);
      T_ex = periodo-T_in;
      posicion = round((angulo*cuentas_motor)/360);
      vel_insp = round(posicion/T_in);
      vel_exp = round(posicion/T_ex);
      Habilitardor = 1;
    }
  }
    
  if (HabilitadorA == 1){
    if (key){
      Ratio = String(key).toInt();
      lcd.setCursor(7, 1);
      lcd.print("Ratio:1:"+String(Ratio));
    }
  }
  
  if (HabilitadorB == 1){
    if (key){
      BPM = String(key).toInt();
      lcd.setCursor(0, 1);
      lcd.print("BPM:"+String(BPM*10));
    }
  }
}


void Final_de_Carrera(){
  if (Contador == 0){
    roboclaw.ForwardM1(address,0);
    motor_1_count = roboclaw.ResetEncoders(address);
    Contador = 1;
  }
}
