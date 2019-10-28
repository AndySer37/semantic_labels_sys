/*********************************
Author: Wolf Chen
Date: 2019/09/22
Last update: 2019/09/22

PRG-100 Rotor Controller
********************************/
#define OUTPUT_PIN_1 2
#define OUTPUT_PIN_2 3

int state = 0;

void setup(){
  Serial.begin(115200);
  pinMode(OUTPUT_PIN_1, OUTPUT);
  pinMode(OUTPUT_PIN_2, OUTPUT);
  Serial.print("Start Connecting\n");
}

void loop()
{
  state = Serial.parseInt();
  // morot command loop
  if(Serial.available()) motor_status();
}

void motor_status()
{
  stop_turn();
  cw_turn();
  ccw_turn();
}

void stop_turn()
{
  if(state == 0)
  {
    Serial.print("Stop\n");
    digitalWrite(OUTPUT_PIN_1, LOW);
    digitalWrite(OUTPUT_PIN_2, LOW);
  }
}

void cw_turn()
{
  if(state == 1)
  {  
    Serial.print("CW Rotate\n");
    digitalWrite(OUTPUT_PIN_1, HIGH);
    digitalWrite(OUTPUT_PIN_2, LOW);    
  }
}

void ccw_turn()
{
  if(state == 2)
  {
    Serial.print("CCW Rotate\n");
    digitalWrite(OUTPUT_PIN_1, LOW);
    digitalWrite(OUTPUT_PIN_2, HIGH);
  }
}
