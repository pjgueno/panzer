#include <WifiEspNowBroadcast.h>
#include <WiFi.h>
#include <Servo.h>
Servo servo;

#define LED_PIN 2

#define DATASIZE 3
#define COLLISION 32
#define SERVO 33
#define TRIG 34
#define ECHO 35


//changer valeurs ici

int freq = 20000;
int ledChannelA = 0;
int ledChannelB = 1;
int resolution = 8; 

//conflit avec channel?
//freq to high?

int posDegrees;
bool up;


void processRx(const uint8_t mac[6], const uint8_t* buf, size_t count, void* cbarg) {
  Serial.printf("Message from %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  for (int i = 0; i < count; ++i) {
    Serial.print(static_cast<char>(buf[i]));
  }
  Serial.println();

  digitalWrite(LED_PIN, HIGH);
 
}

uint8_t data1[DATASIZE]={0,0,0};
uint8_t data2[DATASIZE]={0,0,0};




void setup() {
  Serial.begin(115200);
  Serial.println();

servo.attach(
        SERVO, 
        Servo::CHANNEL_NOT_ATTACHED, 
        0,
        180
);



  WiFi.persistent(false);
  bool ok = WifiEspNowBroadcast.begin("ESPNOW", 3);
  if (!ok) {
    Serial.println("WifiEspNowBroadcast.begin() failed");
    ESP.restart();
  }

  WifiEspNowBroadcast.onReceive(processRx, nullptr);

  digitalWrite(LED_PIN, LOW);

  Serial.print("MAC address of this node is ");
  Serial.println(WiFi.softAPmacAddress());
}

void sendMessage() {
  WifiEspNowBroadcast.send(data1, DATASIZE);

  Serial.println("Sending message");
  Serial.println((char *)data1);
  Serial.print("Recipients:");
  const int MAX_PEERS = 20;
  WifiEspNowPeerInfo peer;
  Serial.printf(" %02X:%02X:%02X:%02X:%02X:%02X", peer.mac[0], peer.mac[1], peer.mac[2], peer.mac[3], peer.mac[4], peer.mac[5]);
  Serial.println();
}

void loop() {








  WifiEspNowBroadcast.loop();
  delay(10);
}
