// Char

#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 2

// Global copy of slave
#define NUMSLAVES 20
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;

#define CHANNEL_MASTER 3
#define CHANNEL_SLAVE 1
#define PRINTSCANRESULTS 0
#define DATASIZE 48
#define DATASIZE2 3

int freq = 20000;
int ledChannelA = 0;
int ledChannelB = 1;
int resolution = 8; 

// Init ESP Now with fallback
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
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
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
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slaves[SlaveCnt].channel = CHANNEL_MASTER; // pick a channel
        slaves[SlaveCnt].encrypt = 0; // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt); Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      const esp_now_peer_info_t *peer = &slaves[i];
      const uint8_t *peer_addr = slaves[i].peer_addr;
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(peer_addr);
      if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(peer);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}


uint8_t data[DATASIZE];
uint64_t pos=0;

uint8_t data2[DATASIZE2]={0,0,0};

// send data
void sendDataInit() {
  sprintf((char *)data,"esp32 2 init");
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) { // print only for first slave
      Serial.print("Sending: ");
      Serial.println((char *)data);
    }
    esp_err_t result = esp_now_send(peer_addr, data, DATASIZE);
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
    delay(100);
  }
}

void sendData() {
  pos++;
  sprintf((char *)data,"espnow %lld",pos);
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) { // print only for first slave
      Serial.print("Sending: ");
      Serial.println((char *)data);
    }
    esp_err_t result = esp_now_send(peer_addr, data, DATASIZE);
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
    //delay(100);
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
    digitalWrite(LED_PIN, 0);
    }

   if (data[0] == 1 || data[1] == 1 || data[2] == 1 ){
    digitalWrite(LED_PIN, 1);
    } 
 
      
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("\t\tLast Packet Recv from: "); Serial.println(macStr);
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

  pinMode(pinEG, OUTPUT);
  pinMode(pin1G, OUTPUT);
  pinMode(pin2G, OUTPUT);

  ledcSetup(ledChannelA, freq, resolution);
  ledcAttachPin(pinEG, ledChannelA);

  digitalWrite(pin1G, LOW); 
  digitalWrite(pin2G, LOW);
  ledcWrite(ledChannelA, 0);
  }
 
  void Update()
  {
      //Serial.println(data2[0]);
    
    if (data2[0] == 0){
        digitalWrite(pin1G, LOW); 
        digitalWrite(pin2G, LOW);
      }

        if (data2[0] == 1){
        digitalWrite(pin1G, LOW); 
        digitalWrite(pin2G, HIGH);
        ledcWrite(ledChannelA, 255);
      }  

        if (data2[0] == 2){
        digitalWrite(pin1G, LOW); 
        digitalWrite(pin2G, HIGH);
        ledcWrite(ledChannelA, 200);
      }  


        if (data2[0] == 3){
        digitalWrite(pin1G, HIGH); 
        digitalWrite(pin2G, LOW);
        ledcWrite(ledChannelA, 190);
      }  

         if (data2[0] == 4){
        digitalWrite(pin1G, HIGH); 
        digitalWrite(pin2G, LOW);
        ledcWrite(ledChannelA, 220);
      }  

              if (data2[0] == 5){
        digitalWrite(pin1G, HIGH); 
        digitalWrite(pin2G, LOW);
        ledcWrite(ledChannelA, 255);
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

  pinMode(pinED, OUTPUT);
  pinMode(pin1D, OUTPUT);
  pinMode(pin2D, OUTPUT);

  ledcSetup(ledChannelB, freq, resolution);
  ledcAttachPin(pinED, ledChannelB);

  digitalWrite(pin1D, LOW); 
  digitalWrite(pin2D, LOW);
  ledcWrite(ledChannelA, 0);
  }
 
  void Update()
  {
//Serial.println(data2[1]);
    
    if (data2[1] == 0){
        digitalWrite(pin1D, LOW); 
        digitalWrite(pin2D, LOW);
      }

        if (data2[1] == 1){
        digitalWrite(pin1D, LOW); 
        digitalWrite(pin2D, HIGH);
        ledcWrite(ledChannelB, 255);
      }  

        if (data2[1] == 2){
        digitalWrite(pin1D, LOW); 
        digitalWrite(pin2D, HIGH);
        ledcWrite(ledChannelB, 200);
      }  


        if (data2[1] == 3){
        digitalWrite(pin1D, HIGH); 
        digitalWrite(pin2D, LOW);
        ledcWrite(ledChannelB, 190);
      }  

              if (data2[1] == 4){
        digitalWrite(pin1D, HIGH); 
        digitalWrite(pin2D, LOW);
        ledcWrite(ledChannelB, 220);
      }  

              if (data2[1] == 5){
        digitalWrite(pin1D, HIGH); 
        digitalWrite(pin2D, LOW);
        ledcWrite(ledChannelB, 255);
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
    
    if (data2[2] == 1){
        digitalWrite(pinK, HIGH); 
      }else{
                digitalWrite(pinK, LOW); 

        }
  }
};



MoteurG moteurG(25,16,17);
MoteurD moteurD(21,4,15);
Klaxon klaxon(26);

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_MODE_APSTA);
  Serial.println("Char");
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

  if (SlaveCnt > 0) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    manageSlave();
    sendDataInit();

  }

  
}

void loop() {
    //sendData();

  //Serial.print("\t\tLast Packet Recv Data[0]: "); Serial.println(data2[0]);
  //Serial.print("\t\tLast Packet Recv Data[1]: "); Serial.println(data2[1]);
  //Serial.print("\t\tLast Packet Recv Data[2]: "); Serial.println(data2[2]);

  moteurG.Update();
  moteurD.Update();
  klaxon.Update();

  //A VOIR SI DELAI 
  delay(10); 
}
