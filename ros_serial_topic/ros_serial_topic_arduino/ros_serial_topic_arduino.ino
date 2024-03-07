<<<<<<< HEAD
#include<Servo.h>

int angle = 90;
String angleStr = "";
String inString = "";
Servo sv;

void setup() {
   Serial.begin(115200);
   sv.attach(2);
}

void loop() {
  if(Serial.available()){
    inString = Serial.readStringUntil('\n');
    int first = inString.indexOf('a');
    int second = inString.indexOf('b');
    angleStr = inString.substring(first+1, second);
    angle = angleStr.toInt();    
    //Serial.println(angle);
    sv.write(angle);
    delay(50);
    
  }
=======
void setup() {
    Serial.begin(115200);
    pinMode(13, OUTPUT); 
}void loop() {
    float temp = random(10, 40);
    Serial.println(temp);
    delay(1000);
>>>>>>> 0df876901954a79d2c2f459ff52fb807410a6513
}
