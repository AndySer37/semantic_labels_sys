/*
 *  Vacuum control
 *  Value  Vacuum Generator  Vacuum Breaker
 *      0                 0               0
 *      1                 0               1
 *      2                 1               0
 *      3                 1               1
 */

#define VACUUM_GENERATOR      7
#define VACUUM_BREAKER        6
#define PNEUMATIC_CYLINDER    5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(100);
  pinMode(VACUUM_GENERATOR, OUTPUT);
  pinMode(VACUUM_BREAKER, OUTPUT);
  pinMode(PNEUMATIC_CYLINDER, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    String command_str = Serial.readString();
    char command_type = command_str[0];
    if(command_type=='p'){
      int command = command_str[1]-'0';
      switch(command){
        case 0:
          digitalWrite(PNEUMATIC_CYLINDER, LOW);
          break;
        case 1:
          digitalWrite(PNEUMATIC_CYLINDER, HIGH);
          break;
      }
    }
    else if(command_type=='v'){ 
      int command = command_str[1]-'0';
      switch(command){
        case 0:
          digitalWrite(VACUUM_GENERATOR, LOW);
          delay(1);
          digitalWrite(VACUUM_BREAKER, LOW);
          break;
        case 1:
          digitalWrite(VACUUM_GENERATOR, LOW);
          delay(1);
          digitalWrite(VACUUM_BREAKER, HIGH);
          break;
        case 2:
          digitalWrite(VACUUM_GENERATOR, HIGH);
          delay(1);
          digitalWrite(VACUUM_BREAKER, LOW);
          break;
        case 3:
          digitalWrite(VACUUM_GENERATOR, HIGH);
          delay(1);
          digitalWrite(VACUUM_BREAKER, HIGH);
          break;
      }
    }
  }
  delay(10);
}
