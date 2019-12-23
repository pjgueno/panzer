#include <Servo.h>
//#include <TFT_eSPI.h>
//#include <SPI.h>

//TFT_eSPI tft = TFT_eSPI();

#define COLLISION 2
#define SERVO 3
#define TRIG 4
#define ECHO 5

#define DATASIZE 3

uint8_t data[DATASIZE];
int posDegrees;
//int previousDeg;
bool up;
//unsigned long currentMicros;
//unsigned long previousMicros;



class Collision
{
  int pinC;
  int previous;
 
  public:
  Collision(int pin)
  {
  pinC = pin;
  previous = 0; 
  
  }
 
  void Update()
  {

    if(isTriggered())
    {
    data[0]= 1;
    }
    else data[0]= 0;

    if (data[0] != previous){
      
      previous = data[0];
      //sendData();
      Serial.print("collision: ");
      Serial.println(data[0]);
      }   
  }

  bool isTriggered()
{
    if(!digitalRead(pinC))
    {
      //delay(5);
        if(!digitalRead(pinC))
        return true;//the collision sensor triggers
    }
    return false;
}
};

//
class Radar
{
  Servo servo;
  int pinS;
  int pinT;
  int pinE;
  int previous1;
  int previous2;
  long duration;
  int distance;
  bool trig;
  bool echo;
  //int pos;              
  //int increment;        
  //int  updateInterval;     
  unsigned long previousMicros;
  //unsigned long timeOut;
  
 
  public:
  Radar(int pin1, int pin2)
  {
  pinT = pin1;
  pinE = pin2; 
  //unsigned long delayMicros1 = 2;
  //unsigned long delayMicros1 = 10;
  trig = false;
  echo = false;
  previous1 = 0;
  previous2 = 0;
  //lastUpdate = 0;
  //updateInterval = interval;
  //increment = 1;
  previousMicros = 0;
  //timeOut = 5000;
  }

  void Attach(int pin)
  {
    servo.attach(pin);
  }

  void Detach()
  {
    servo.detach();
  }
 
  bool Update(int deg)
  {
    //currentMicros = micros();
    
    if (trig == false && echo == false){
      servo.write(deg);
      data[1] = deg;
      trig = true;
      digitalWrite(pinT, LOW);
      previousMicros = micros();
      }

      if (trig == true && echo == false && (micros() - previousMicros >= 2)){
      digitalWrite(pinT, HIGH);
      trig = false;
      echo = true;
      previousMicros = micros();
      }

      if (trig == false && echo == true && (micros() - previousMicros >= 10)){
      digitalWrite(pinT, LOW);
      duration = pulseIn(pinE, HIGH,5000UL);

      //Serial.print("duration: ");
      //Serial.println(duration);
      distance= duration*0.034/2;
      data[2] = distance;
      echo = false;


      Serial.print("degres: ");
      Serial.println(data[1]);

      Serial.print("distance: ");
      Serial.println(data[2]);

      delay(1000);

//    if (data[1] != previous1){
//      
//      previous1 = data[1];
//      //sendData();
//      Serial.print("degres: ");
//      Serial.println(data[1]);
//      }   
//
//    if (data[2] != previous2){
//      
//      previous2 = data[2];
//      //sendData();
//      Serial.print("distance: ");
//      Serial.println(data[2]);
//      }   
      return true;
       }
    
    return false;
  }

};


Collision collision(COLLISION);
Radar radar(TRIG,ECHO);

void setup() {
Serial.begin(115200);
pinMode(COLLISION,INPUT);
pinMode(TRIG, OUTPUT); 
pinMode(ECHO, INPUT);
radar.Attach(SERVO);
posDegrees = 0;
//previousMicros = 0;
//previousDeg = 0;
}




void loop() {

  //currentMicros = micros();
  collision.Update();
  //radar.Update();


  
//for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
//       servo1.write(posDegrees);
//        Serial.println(posDegrees);
//          delay(5);
//    }
//
//    for(int posDegrees = 180; posDegrees > 0; posDegrees--) {
//        servo1.write(posDegrees);
//        Serial.println(posDegrees);
//          delay(5);
//    }

//    

//  
//Serial.print("posDegrees: ");
//      Serial.println(posDegrees);
//
if (posDegrees == 0){
  
  up = true;
  if (radar.Update(posDegrees)){
      posDegrees += 1;
    }
  }

if (posDegrees > 0 && posDegrees < 180 && up == true){
  if (radar.Update(posDegrees)){
      posDegrees += 1;
    }
  }

  if (posDegrees == 180){
  up = false;
  if (radar.Update(posDegrees)){
      posDegrees -= 1;
    }  
    }

  if (posDegrees > 0 && posDegrees < 180 && up == false){
  if (radar.Update(posDegrees)){
      posDegrees -= 1;
    }
  }
        //Serial.println(data)
}
