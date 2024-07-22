#include <esp_now.h>
#include <WiFi.h>

// MAC Address of the receiver
uint8_t receiverAddress[] = {0x24, 0x58, 0x7C, 0xD8, 0x7F, 0xA4};

// Structure to send data
typedef struct struct_message {
  uint8_t LX;
  uint8_t LY;
  uint8_t RX;
  uint8_t RY;
} struct_message;

// Create a message to send
struct_message myData;

const int analogInPin1 = 34;  
const int analogInPin2 = 35;  
const int analogInPin3 = 39;  
const int analogInPin4 = 36;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  myData.LX = map(analogRead(analogInPin1),0,4096,0,256);
  myData.LY = map(analogRead(analogInPin2),0,4096,0,256);
  myData.RX = map(analogRead(analogInPin3),0,4096,0,256);
  myData.RY = map(analogRead(analogInPin4),0,4096,0,256);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  
  delay(100);
}
