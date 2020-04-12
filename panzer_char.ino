#include <esp_now.h>
#include <WiFi.h>
#include <esp32-hal-ledc.h>

#define WIFI_CHANNEL 1

// pas d'inversion: les deux écoutent sur un canal et émettent sur un autre sans croisement...?

#define LED_PIN 2

uint8_t remoteMac[] = {0xB4,0xE6,0x2D,0x99,0x09,0x75};

esp_now_peer_info_t remote;
const esp_now_peer_info_t *remoteNode = &remote;
const byte maxDataFrameSize = 3;
uint8_t dataToSend[maxDataFrameSize]={0,0,0};
uint8_t dataRecved[maxDataFrameSize]={0,0,0};

int freq = 30000;
int ledChannelA = 0;
int ledChannelB = 1;
int resolution = 8; 

int servoChannel = 2;
int servofreq = 50;
int servoResolution = 16; 

int MIN_PULSE_WIDTH = 900;
int MAX_PULSE_WIDTH = 2500;

int posDegrees;
bool up;

bool sent;
int delaiServo = 50;
unsigned long previousMillisServo;

class MoteurG
{
  int pinEG;
  int pin1G;
  int pin2G;
   
  public:
  MoteurG(int pin1, int pin2, int pin3)
  {
  pinEG = pin1;
  pin1G = pin2;
  pin2G = pin3;

  pinMode(pin1G, OUTPUT);
  pinMode(pin2G, OUTPUT);

  ledcSetup(ledChannelB, freq, resolution);
  ledcAttachPin(pinEG, ledChannelB);

  digitalWrite(pin1G, LOW); 
  digitalWrite(pin2G, LOW);
  ledcWrite(ledChannelB, 0);
  }
 
  void Update()
  {
    
    if (dataRecved[0] == 0){
        digitalWrite(pin1G, LOW); 
        digitalWrite(pin2G, LOW);
        ledcWrite(ledChannelB, 0);
      }

        if (dataRecved[0] == 1){
        digitalWrite(pin1G, LOW); 
        digitalWrite(pin2G, HIGH);
        ledcWrite(ledChannelB, 255);
      }  

        if (dataRecved[0] == 2){
        digitalWrite(pin1G, LOW); 
        digitalWrite(pin2G, HIGH);
        ledcWrite(ledChannelB, 200);
      }  


        if (dataRecved[0] == 3){
        digitalWrite(pin1G, HIGH); 
        digitalWrite(pin2G, LOW);
        ledcWrite(ledChannelB, 190);
      }  

         if (dataRecved[0] == 4){
        digitalWrite(pin1G, HIGH); 
        digitalWrite(pin2G, LOW);
        ledcWrite(ledChannelB, 220);
      }  

              if (dataRecved[0] == 5){
        digitalWrite(pin1G, HIGH); 
        digitalWrite(pin2G, LOW);
        ledcWrite(ledChannelB, 255);
      }  
  }
};



class MoteurD
{
  int pinED;
  int pin1D;
  int pin2D;
   
  public:
  MoteurD(int pin1, int pin2, int pin3)
  {
  pinED = pin1;
  pin1D = pin2;
  pin2D = pin3;

  pinMode(pin1D, OUTPUT);
  pinMode(pin2D, OUTPUT);

  ledcSetup(ledChannelA, freq, resolution);
  ledcAttachPin(pinED, ledChannelA);

  digitalWrite(pin1D, LOW); 
  digitalWrite(pin2D, LOW);
  ledcWrite(ledChannelA, 0);
  }
 
  void Update()
  {
    
    if (dataRecved[1] == 0){
        digitalWrite(pin1D, LOW); 
        digitalWrite(pin2D, LOW);
        ledcWrite(ledChannelA, 0);
      }

        if (dataRecved[1] == 1){
        digitalWrite(pin1D, LOW); 
        digitalWrite(pin2D, HIGH);
        ledcWrite(ledChannelA, 255);
      }  

        if (dataRecved[1] == 2){
        digitalWrite(pin1D, LOW); 
        digitalWrite(pin2D, HIGH);
        ledcWrite(ledChannelA, 200);
      }  


        if (dataRecved[1] == 3){
        digitalWrite(pin1D, HIGH); 
        digitalWrite(pin2D, LOW);
        ledcWrite(ledChannelA, 190);
      }  

              if (dataRecved[1] == 4){
        digitalWrite(pin1D, HIGH); 
        digitalWrite(pin2D, LOW);
        ledcWrite(ledChannelA, 220);
      }  

              if (dataRecved[1] == 5){
        digitalWrite(pin1D, HIGH); 
        digitalWrite(pin2D, LOW);
        ledcWrite(ledChannelA, 255);
      }  
  }
};

class Klaxon
{
  int pinK;
   
  public:
  Klaxon(int pin)
  {
  pinK = pin;

  pinMode(pinK, OUTPUT);
  digitalWrite(pinK, LOW); 
  }
 
  void Update()
  {
//Serial.println(data2[2]);
    
    if (dataRecved[2] == 1){
        digitalWrite(pinK, HIGH); 
      }else{
                digitalWrite(pinK, LOW); 

        }
  }
};



class Collision
{
  int pinC;
  int previous;  
  unsigned long previousMillis;
  bool first;
  bool second; 
  int sensitivity;
  int duree;
  
  public:
  Collision(int pin)
  {
  pinC = pin;
  pinMode(pinC, INPUT);
  previous = 0; 
  first = false;
  second = false;
  previousMillis = 0;
  sensitivity = 50;
  duree = 1000;
  // si sensitivity > vitesse du servo pas besoin de previous;
  }
 
  void Update()
  {
    if (!first && !second){
    if(!digitalRead(pinC)){
      first = true;
      previousMillis = millis();
    }else{
        dataToSend[0]= 0;
    }
    
    }

    if (first && !second && millis() - previousMillis >= sensitivity){
      
      if(!digitalRead(pinC)){
      second = true;
      previousMillis = millis();
      }
      else{
    dataToSend[0]= 0;        
    first = false;
    }
    }

if (first && second && millis() - previousMillis < duree){
    dataToSend[0]= 1;
    Serial.println("collision");
  }

if (first && second && millis() - previousMillis >= duree){
    first = false;
    second = false;
    dataToSend[0]= 0;
}

 if (dataToSend[0] != previous){
      previous = dataToSend[0];
      Serial.print("Sending: ");
     for (int i = 0; i < 3; i++) {
        Serial.print(dataToSend[i]);
                    if (i<2){
            Serial.print(";");
            }
        }
        Serial.println();

    if(esp_now_send(remote.peer_addr, dataToSend, maxDataFrameSize) == ESP_OK){
    Serial.println("Success");
    }
    else
    {
    Serial.println("Failed!");
    }
      }  
      }

       
};


class Radar
{
  int pinS;
  int pinT;
  int pinE;
  int previous1;
  int previous2;
  long duration;
  int distance;
  bool trig;
  bool echo; 
  unsigned long previousMicros;
  unsigned long previousMillis;

  public:
  Radar(int pin1, int pin2, int pin3)
  {
  pinS = pin1;
  pinT = pin2;
  pinE = pin3;
  pinMode(pinT, OUTPUT); 
  pinMode(pinE, INPUT); 

  ledcSetup(servoChannel,servofreq,servoResolution);
  ledcAttachPin(pinS,servoChannel);
  
  trig = false;
  echo = false;
  previous1 = 0;
  previous2 = 0;
  previousMicros = 0;
  
  }


  int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * servofreq * 65536);
  return analog_value;
}

 
  bool Update(int degree)
  {
   
    if (trig == false && echo == false){
          ledcWrite(servoChannel, pulseWidth(degree)); 
          dataToSend[1] = degree;
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
          //duration = pulseIn(pinE, HIGH,14705UL); //2 x 2.5 m = 5 m
          duration = pulseIn(pinE, HIGH,5882UL);     //2 x 1 m = 2 m
          //duration = pulseIn(pinE, HIGH,5000UL);
          
          distance= duration*0.034/2;

          dataToSend[2] = distance;
          echo = false;
          
        if (dataToSend[1] != previous1 || dataToSend[2] != previous2 ){
          
          previous1 = dataToSend[1];
          previous2 = dataToSend[2];

        sent = false;
      
        Serial.print("Sending: ");
        for (int i = 0; i < 3; i++) {
            Serial.print(dataToSend[i]);
           if (i<2){
            Serial.print(";");
            }
            }
            Serial.println();
    
        if(esp_now_send(remote.peer_addr, dataToSend, maxDataFrameSize) == ESP_OK){
        Serial.println("Success");
          sent = true;
          previousMillisServo = millis();
        return true; //VOIR SI TRUE DANS TOUS LES CAS?
        }
        else
        {
        Serial.println("Failed!");
        }
          }   
           }
        
        return false;
    
      }
    };

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len){
  char macStr[18];
snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("\t\tLast Packet Recv from: "); Serial.println(macStr);
  Serial.print("\t\tLast Packet Recv Data: "); 
  for(int i = 0; i < 3; i++){
  Serial.print(data[i]);
  if (i<2){
            Serial.print(";");
            }
}
Serial.println();
  memcpy(dataRecved, data, sizeof data);
}

MoteurG moteurG(21,16,17);
MoteurD moteurD(14,26,27);
Klaxon klaxon(23);
Collision collision (34);
Radar radar(22,18,19);

void setup()
{
  Serial.begin(115200);
  Serial.println("Panzer");

  posDegrees = 0;
  previousMillisServo = millis();
  sent = true;
  
  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_STA); 
  Serial.print("STA MAC PANZER: "); Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  if(esp_now_init() == ESP_OK)
  {
    Serial.println("ESP NOW INIT!");
  }
  else
  {
    Serial.println("ESP NOW INIT FAILED....");
    ESP.restart();
  }

  memcpy( &remote.peer_addr, &remoteMac, 6 );
  remote.channel = WIFI_CHANNEL;
  remote.encrypt = 0;

  if( esp_now_add_peer(remoteNode) == ESP_OK)
  {
    Serial.println("Added Remote Node!");
  }
  else
  {
    Serial.println("Remote Node could not be added...");
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
}

void loop()
{

  moteurG.Update();
  moteurD.Update();
  klaxon.Update();


  collision.Update();

  //ajouter sent == true

  //CONTROLER SEULEMENTLES PULSE AVEC LE RADAR  

 if (sent == true && millis() - previousMillisServo >= delaiServo){
  
 //sent = false;
 
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

 // previousMillisServo = millis();
//  }
//  
//  else if(sent == false && millis() - previousMillisServo >= delaiServo){
//    if (!radar.Update(posDegrees)){
//      Serial.println("Radar work in progress...");
//    }
    }else{
    Serial.println("Waiting...");
    }
  //delay(1000);

  
    //delay(10); //A VOIR
}



