//Télécommande

#include <esp_now.h>
#include <WiFi.h>

#include <TFT_eSPI.h>
#include <SPI.h>

#define LED_PIN 2

esp_now_peer_info_t slave;
bool slaveFound = false;

#define CHANNEL_MASTER 3
#define CHANNEL_SLAVE 1

//INVERSION?

#define PRINTSCANRESULTS 0
#define DATASIZE 3


TFT_eSPI tft = TFT_eSPI();

void InitESPNow() {
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
   
  memset(&slave, 0, sizeof(slave));
  //memset(slave, 0, sizeof(slave));
  
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("ESPNOW") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];          }
        }
        slave.channel = CHANNEL_MASTER; // pick a channel
        slave.encrypt = 0; // no encryption
        slaveFound = true;
         break;
      }
    }
  }

  if (slaveFound) {
    Serial.println(" Slave found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slaveFound) {

    //deletePeer();

    if ( esp_now_is_peer_exist(slave.peer_addr)) {

      Serial.println("Already Paired");
      return true;
    }else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&slave);
        //esp_err_t addStatus = esp_now_add_peer(slave);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
          return true;
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
          return true;
        } else {
          Serial.println("Not sure what happened");
          return false;
        }
      }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

//void deletePeer() {
//  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
//  Serial.print("Slave Delete Status: ");
//  if (delStatus == ESP_OK) {
//    // Delete success
//    Serial.println("Success");
//  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
//    // How did we get so far!!
//    Serial.println("ESPNOW Not Init");
//  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
//    Serial.println("Invalid Argument");
//  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
//    Serial.println("Peer not found.");
//  } else {
//    Serial.println("Not sure what happened");
//  }
//}

uint8_t data1[DATASIZE]={0,0,0};

uint8_t data2[DATASIZE]={0,0,0};

// send data
//void sendDataInit() {
//  sprintf((char *)data1,"esp32 1 init");
//  for (int i = 0; i < SlaveCnt; i++) {
//    const uint8_t *peer_addr = slaves[i].peer_addr;
//    if (i == 0) { // print only for first slave
//      Serial.print("Sending: ");
//      Serial.println((char *)data1);
//    }
//    esp_err_t result = esp_now_send(peer_addr, data1, DATASIZE);
//    Serial.print("Send Status: ");
//    if (result == ESP_OK) {
//      Serial.println("Success");
//    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
//      // How did we get so far!!
//      Serial.println("ESPNOW not Init.");
//    } else if (result == ESP_ERR_ESPNOW_ARG) {
//      Serial.println("Invalid Argument");
//    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
//      Serial.println("Internal Error");
//    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
//      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
//    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
//      Serial.println("Peer not found.");
//    } else {
//      Serial.println("Not sure what happened");
//    }
//    delay(100);
//  }
//}

void sendData() {
  //sprintf((char *)data,"Send commandes");
    const uint8_t *peer_addr = slave.peer_addr;
    Serial.print("Sending: ");
    Serial.println((char *)data1);
    
    esp_err_t result = esp_now_send(peer_addr, data1, DATASIZE);
    //esp_err_t result = esp_now_send(peer_addr, &data1, DATASIZE);

    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];

if (data[0] == 0 && data[1] == 0 && data[2] == 0 ){
    digitalWrite(LED_PIN, LOW);
    }

   if (data[0] != 0 || data[1] != 0 || data[2] != 0 ){
    digitalWrite(LED_PIN, HIGH);
    } 


snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("\t\tLast Packet Recv from: "); Serial.println(macStr);
  Serial.print("\t\tLast Packet Recv Data: "); Serial.println((char *)data);
  memcpy(data2, data, sizeof data);  
}


// config AP SSID
void configDeviceAP() {
  String Prefix = "ESPNOW:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL_SLAVE, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}



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
    data1[0]= 1;
    } 

    if (analogRead(pinG) > 0 && analogRead(pinG) <= 140){
    data1[0]= 2;
    } 

    if (analogRead(pinG) > 140 && analogRead(pinG) <= 160){
    data1[0]= 0;
    } 

    if (analogRead(pinG) > 160 && analogRead(pinG) <= 300){
    data1[0]= 3;
    } 

    if (analogRead(pinG) > 300 && analogRead(pinG) <= 480){
    data1[0]= 4;
    } 

    if (analogRead(pinG) > 480){
    data1[0]= 5;
    }

    if (data1[0] != previous){
      
      previous = data1[0];
      sendData();
      Serial.println(data1[0]);

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
    data1[1]= 1;
    } 

    if (analogRead(pinD) > 0 && analogRead(pinD) <= 140){
    data1[1]= 2;
    } 

    if (analogRead(pinD) > 140 && analogRead(pinD) <= 160){
    data1[1]= 0;
    } 

    if (analogRead(pinD) > 160 && analogRead(pinD) <= 300){
    data1[1]= 3;
    } 

    if (analogRead(pinD) > 300 && analogRead(pinD) <= 480){
    data1[1]= 4;
    } 

    if (analogRead(pinD) > 480){
    data1[1]= 5;
    } 

       if (data1[1] != previous){
      
      previous = data1[1];
      sendData();
      Serial.println(data1[1]);

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
  pinMode(pinK, INPUT);
  previous = 0;
  }
 
  void Update()
  {
    if (digitalRead(pinK) != 1){
    data1[2]= 1;
    } 
    else 
    {
      data1[2]= 0;
      }

         if (data1[2] != previous){
      
      previous = data1[2];
      sendData();
//      Serial.print("klaxon");
//      Serial.println(data[2]);
      }

       }
};

class Screen
{
  int collision;
  int motorD;
  int motorG;
  int distance;
  int degres;
 
  public:
  Screen()
  {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  tft.drawLine(0, 65, 128, 65, TFT_WHITE);
  tft.drawCircle(64,0,64,TFT_GREEN);

  //tft.drawRect(0,0,50,50,TFT_RED);

  
  
  
  }
 
  void Update()
  {

//tft.setCursor(6, 0);
//  tft.setTextColor(TFT_WHITE);
//  tft.setTextWrap(true);
//  tft.print("STARTING...");
//
//  //tft.fillRect(0,0,50,50,TFT_BLUE);
//


      Serial.print("collision: ");
      Serial.println(data2[0]);


      Serial.print("degres: ");
      Serial.println(data2[1]);

      Serial.print("distance: ");
      Serial.println(data2[2]);


  
   }
};





//uint8_t MAC_array[6];
//char MAC_char[18];
//
//
//    WiFi.macAddress(MAC_array);
//    for (int i = 0; i < sizeof(MAC_array); ++i){
//      sprintf(MAC_char,"%s%02x:",MAC_char,MAC_array[i]);
//    }
// 
//    Serial.println(MAC_char);

//https://www.arduino.cc/en/Reference/WiFiMACAddress

//  WiFi.macAddress(mac);
//  Serial.print("MAC: ");
//  Serial.print(mac[5],HEX);
//  Serial.print(":");
//  Serial.print(mac[4],HEX);
//  Serial.print(":");
//  Serial.print(mac[3],HEX);
//  Serial.print(":");
//  Serial.print(mac[2],HEX);
//  Serial.print(":");
//  Serial.print(mac[1],HEX);
//  Serial.print(":");
//  Serial.println(mac[0],HEX);


ManetteG manetteG(34);
ManetteD manetteD(35);
Klaxon klaxon(32);
Screen screen;

void setup() {
  Serial.begin(115200);
  

  analogReadResolution(9);
  analogSetWidth(9); 

  pinMode(LED_PIN, OUTPUT);
  
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_MODE_APSTA);
  Serial.println("Telecommande");
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  ScanForSlave();

  if (slaveFound) { 

      if (manageSlave()) { 
      

    Serial.println("Pairing OK");
    
    // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    
  }else{
    Serial.println("Slave pair failed!");
    
    }

  } 
}

void loop() {

manetteG.Update();
manetteD.Update();
klaxon.Update();
screen.Update();
}
