//MASTER

#include <esp_now.h>
#include <WiFi.h>


#include <TFT_eSPI.h>
#include <SPI.h>

#define WIFI_CHANNEL 1


// pas d'inversion: les deux écoutent sur un canal et émettent sur un autre sans croisement...?


esp_now_peer_info_t panzer;
TFT_eSPI tft = TFT_eSPI();

#define LED_PIN 2


uint8_t panzerMac[] = {0xB4,0xE6,0x2D,0x98,0xA6,0x11};
const uint8_t maxDataFrameSize=3;
const esp_now_peer_info_t *panzerNode = &panzer;
uint8_t dataToSend[maxDataFrameSize]={0,0,0};
uint8_t dataRecved[maxDataFrameSize]={0,0,0};

volatile bool pressed = false;


class ManetteG
{
  int pinG;
  int previous;
 
  public:
  ManetteG(int pin)
  {
  pinG = pin;
  adcAttachPin(pinG);
  analogSetPinAttenuation(pinG,ADC_11db);
  previous = 0; 
  
  }
 
  void Update()
  {

    if (analogRead(pinG) == 0){
    dataToSend[0]= 1;
    } 

    if (analogRead(pinG) > 0 && analogRead(pinG) <= 140){
    dataToSend[0]= 2;
    } 

    if (analogRead(pinG) > 140 && analogRead(pinG) <= 160){
    dataToSend[0]= 0;
    } 

    if (analogRead(pinG) > 160 && analogRead(pinG) <= 300){
    dataToSend[0]= 3;
    } 

    if (analogRead(pinG) > 300 && analogRead(pinG) <= 480){
    dataToSend[0]= 4;
    } 

    if (analogRead(pinG) > 480){
    dataToSend[0]= 5;
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


    if( esp_now_send(panzer.peer_addr, dataToSend, maxDataFrameSize) == ESP_OK){
    Serial.println("Success");
    }
    else
    {
    Serial.println("Failed!");
    }

      }    
  }
};


class ManetteD
{
  int pinD;
  int previous;
 
  public:
  ManetteD(int pin)
  {
  pinD = pin;
  adcAttachPin(pinD);
  analogSetPinAttenuation(pinD,ADC_11db);
  previous = 0;
  }
 
  void Update()
  {
    
if (analogRead(pinD) == 0){
    dataToSend[1]= 1;
    } 

    if (analogRead(pinD) > 0 && analogRead(pinD) <= 140){
    dataToSend[1]= 2;
    } 

    if (analogRead(pinD) > 140 && analogRead(pinD) <= 160){
    dataToSend[1]= 0;
    } 

    if (analogRead(pinD) > 160 && analogRead(pinD) <= 300){
    dataToSend[1]= 3;
    } 

    if (analogRead(pinD) > 300 && analogRead(pinD) <= 480){
    dataToSend[1]= 4;
    } 

    if (analogRead(pinD) > 480){
    dataToSend[1]= 5;
    } 

       if (dataToSend[1] != previous){
      
      previous = dataToSend[1];
      Serial.print("Sending: ");
            for (int i = 0; i < 3; i++) {
        Serial.print(dataToSend[i]);
                    if (i<2){
            Serial.print(";");
            }
        }
        Serial.println();


    if( esp_now_send(panzer.peer_addr, dataToSend, maxDataFrameSize) == ESP_OK){
    Serial.println("Success");
    }
    else
    {
    Serial.println("Failed!");
    }

      }
   }
};



class Klaxon
{
  int pinK;
  int previous;
 
  public:
  Klaxon(int pin)
  {
  pinK = pin;
  pinMode(pinK, INPUT_PULLDOWN);
  previous = 0;
  }
 
  void Update()
  {
    //Serial.println(pressed);
    if (digitalRead(pinK) != HIGH){
    dataToSend[2]= 1;
    } 
    else 
    {
      dataToSend[2]= 0;
      }

         if (dataToSend[2] != previous){
      previous = dataToSend[2];
    Serial.print("Sending: ");
          for (int i = 0; i < 3; i++) {
        Serial.print(dataToSend[i]);
                    if (i<2){
            Serial.print(";");
            }
        }
        Serial.println();


    if( esp_now_send(panzer.peer_addr, dataToSend, maxDataFrameSize) == ESP_OK){
    Serial.println("Success");
    }
    else
    {
    Serial.println("Failed!");
    }

   }
  }
};

class Screen
{

  public:
  Screen()
  {

  }

  void Update(uint8_t int1,uint8_t int2,uint8_t int3,uint8_t int4,uint8_t int5){

  
tft.setCursor(0, 0);
tft.setTextSize(3);
if (int1 == 1){
  tft.setTextColor(TFT_WHITE,TFT_RED);
  tft.print(" ALARM ");
  }
if (int1 == 0){
  tft.setTextColor(TFT_BLACK,TFT_BLACK);
  tft.print("             ");
  }

if (int3 == 0 && int3 >= 100){

}

if (int3 != 0){
  
  int distance = int(map(int3, 0, 100, 0, 64));

  int coox1 = 63 + distance*cos(radians(-int2));
  int cooy1 = 159 + distance*sin(radians(-int2));
  tft.drawLine(63, 159,coox1,cooy1, TFT_GREEN);

  int coox2 = 63 + 64*cos(radians(-int2));
  int cooy2 = 159 + 64*sin(radians(-int2));
  tft.drawLine(coox1,cooy1,coox2,cooy2, TFT_RED);
  
  }else{

    int coox = 63 + 64*cos(radians(-int2));
    int cooy = 159 + 64*sin(radians(-int2));

tft.drawLine(63, 159,coox,cooy, TFT_GREEN);
    }

//tft.setCursor(0,60 );
//tft.setTextSize(1);
//tft.setTextColor(TFT_WHITE,TFT_BLACK);
//tft.print("Distance = ");
//tft.print(int3);
//
//if (int3!=0){
//
//
//int coox2 = coox + distance *cos(radians(int2-180));
//int cooy2 = cooy + distance *sin(radians(int2-180));
//
//tft.drawLine(coox, cooy,coox2,cooy2, TFT_RED);
//  }
//










tft.setCursor(0,40 );
tft.setTextSize(3);
switch (int4) {
  case 0:
    tft.setTextColor(TFT_BLACK,TFT_WHITE);
    tft.print("N");
    break;
  case 1:
    tft.setTextColor(TFT_BLACK,TFT_RED);
    tft.print("2");
    break;
  case 2:
    tft.setTextColor(TFT_BLACK,TFT_RED);
    tft.print("1");
    break;
  case 3:
    tft.setTextColor(TFT_BLACK,TFT_GREEN);
    tft.print("1");
    break;
  case 4:
    tft.setTextColor(TFT_BLACK,TFT_GREEN);
    tft.print("2");
    break;
  case 5:
    tft.setTextColor(TFT_BLACK,TFT_GREEN);
    tft.print("3");
    break;
}

tft.setCursor(110, 40);
switch (int5) {
  case 0:
    tft.setTextColor(TFT_BLACK,TFT_WHITE);
    tft.print("N");
    break;
  case 1:
    tft.setTextColor(TFT_BLACK,TFT_RED);
    tft.print("2");
    break;
  case 2:
    tft.setTextColor(TFT_BLACK,TFT_RED);
    tft.print("1");
    break;
  case 3:
    tft.setTextColor(TFT_BLACK,TFT_GREEN);
    tft.print("1");
    break;
  case 4:
    tft.setTextColor(TFT_BLACK,TFT_GREEN);
    tft.print("2");
    break;
  case 5:
    tft.setTextColor(TFT_BLACK,TFT_GREEN);
    tft.print("3");
    break;
}



//  tft.setTextColor(TFT_WHITE);
//  tft.setTextWrap(true);
//  tft.print("STARTING...");
//
//  //tft.fillRect(0,0,50,50,TFT_BLUE);
//


//      Serial.print("collision: ");
//      Serial.println(int1);
//
//
//      Serial.print("degres: ");
//      Serial.println(int3);
//
//      Serial.print("distance: ");
//      Serial.println(int3);


  
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
        for (int i = 0; i < 3; i++) {
        Serial.print(data[i]);
          if (i<2){
            Serial.print(";");
            }
        }
        Serial.println();

  memcpy(dataRecved, data, sizeof data);
}


//void configDeviceAP() {
//  String Prefix = "ESPNOW:";
//  String Mac = WiFi.macAddress();
//  String SSID = Prefix + Mac;
//  String Password = "123456789";
//  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), WIFI_CHANNEL_PR, false,1);
//  if (!result) {
//    Serial.println("AP Config failed.");
//  } else {
//    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
//  }
//}






ManetteG manetteG(32);
ManetteD manetteD(33);
Klaxon klaxon(13);
Screen screen;



void setup()
{
  Serial.begin(115200);
  Serial.println("Telecommande");

  pinMode(LED_PIN, OUTPUT);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  //tft.drawLine(0, 90, 128, 90, TFT_WHITE);
  tft.drawCircle(63,159,64,TFT_GREEN);
  
  analogReadResolution(9);
  analogSetWidth(9); 

  WiFi.mode(WIFI_STA);
  //configDeviceAP();
  Serial.print("STA MAC REMOTE: "); Serial.println(WiFi.macAddress());


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
  
  
  memcpy( &panzer.peer_addr, &panzerMac, 6 );
  panzer.channel = WIFI_CHANNEL;
  panzer.encrypt = 0;

  
  if( esp_now_add_peer(panzerNode) == ESP_OK)
  {
    Serial.println("Added Panzer Node!");
  }
  else
  {
    Serial.println("Panzer Node could not be added...");
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
}

void loop()
{

manetteG.Update();
manetteD.Update();
klaxon.Update();
screen.Update(dataRecved[0],dataRecved[1],dataRecved[2],dataToSend[0],dataToSend[1]);
  
  delay(100);
}

