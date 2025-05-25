#include <HX711.h>
#include "SPIFFS.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <time.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "FS.h"
#include <esp_wifi.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

// check these
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define TX_CHAR_UUID        "abcd1234-5678-1234-5678-abcdef012345"  // Notify characteristic (ESP32 -> app)
#define RX_CHAR_UUID        "fedc4321-9876-5432-1098-76543210dcba"  // Write characteristic (app -> ESP32)
#define TIME_CHAR_UUID      "87654321-4321-4321-4321-ba0987654321"  // Write: app -> ESP32

// Calibration factor - adjust this based on your load cell
float calibration_factor = 420.0;  // This value is an example — you'll need to find yours
float known_weight = 0.0;

//-------
// Time
unsigned long rebootTimestamp = 1747941600;  // from BLE
unsigned long bootMillis = 0;        // captured when time is set


// Detection (Just check the deviation value with regards)
const float MIN_BIRD_WEIGHT = 1000.0;
const float MAX_BIRD_WEIGHT = 1800.0;
const float RSD_THRESHOLD = 0.05;
const int BUFFER_SIZE = 20;
float weightBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int checker = 0;
bool sameBird = false;
bool birdDet = false;
bool datasaved = false;
BLECharacteristic *txCharacteristic;
BLECharacteristic *rxCharacteristic;
BLECharacteristic *timeCharacteristic;
BLE2902 *pBLE2902_2;

// --------------------------Node data--------------------------------------------
String currentNode;
uint8_t broadcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t esp2[] = {0x94, 0x6C, 0xC6, 0x99, 0xD9, 0x90};

// --------------------------Defining Functions-----------------------------------
float computeMean(float *data, int size);
float computeStdDev(float *data, int size, float mean);
void storeBirdData(float weight, unsigned long timestamp);
void sendStoredData();
void sendQueryToMesh(const String& target);
void sendDeleteToMesh();
void onESPNowReceive(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len);
void sendChunks(String data);
time_t getEstimatedTime();

// --------------------------ESP-NOW message structure-----------------------------
typedef struct message_t {
  char json[250];
} message_t;

// --------------------------BLE CallBack Functions--------------------------------

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String rxValue = pChar->getValue();
    Serial.print("Received via RX characteristic: ");
    Serial.println(rxValue);

    // check which node, if it is the gateway node send our file, otherwise send to ESPQuery function
    if (rxValue.startsWith("NODE:")) {
      String requestedNode = rxValue.substring(5);  // e.g., NODE:ESP_2
      currentNode = requestedNode;
      Serial.print("Requesting data from node: ");
      Serial.println(requestedNode);

      if(requestedNode == "ESP_1"){ 
        sendStoredData();
        Serial.println("Requested node is the gateway.");
      }
      else{
        sendQueryToMesh(requestedNode);
        Serial.println("Sent to ESP-NOW query.");
      } 
    }

    else if (rxValue == "RECEIVED") {
      if(currentNode == "ESP_1"){
        if (SPIFFS.exists("/bird_log.csv")) {
        SPIFFS.remove("/bird_log.csv");
        Serial.println("CSV file deleted after successful transfer.");
        }
      }
      else{
        //send delete query
        sendDeleteToMesh();
        Serial.println("Sending to delete data from target node.");
      }
    }

    else if (rxValue == "FAILED") {
      Serial.println("App reported failed transfer.");
    }
  }
};


class TimeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = characteristic->getValue(); // get std::string from BLECharacteristic
    //String value = String(valueStr.c_str());            // convert to Arduino String

    if (value.length() > 0) {
      rebootTimestamp = strtoul(value.c_str(), NULL, 10);
      bootMillis = millis();
    }
  }
};


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Central connected");
  }
  void onDisconnect(BLEServer* pServer) {
    Serial.println("Central disconnected");
  }
};

//-------------------------------------------------------------

// Initial setup, run only once
void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  scale.set_scale(); // initialize scale without calibration
  scale.tare();      // Reset the scale to 0
  SPIFFS.begin(true);

  Serial.println("Scale is ready.");

  bootMillis = millis();  // capture when fallback time starts

  //esp_log_level_set("task_wdt", ESP_LOG_NONE);
  // Init WiFi in STA mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  int channel = WiFi.channel();
  Serial.printf("WiFi channel: %d\n", channel);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0;    // Use current channel
  peerInfo.encrypt = false;

  uint8_t* macs[] = {esp2, broadcast};
  const char* labels[] = {"ESP_2", "Broadcast"};

  for (int i = 0; i < sizeof(macs) / sizeof(macs[0]); i++) {
    memcpy(peerInfo.peer_addr, macs[i], 6);
    esp_err_t result = esp_now_add_peer(&peerInfo);

    Serial.print("Adding peer ");
    Serial.print(labels[i]);
    if (result == ESP_OK) {
      Serial.println("...Success");
    } else if (result == ESP_ERR_ESPNOW_EXIST) {
      Serial.println("...Already exists");
    } else {
      Serial.printf("...Failed with error: %d\n", result);
    }
  }

  esp_now_register_recv_cb(onESPNowReceive);

 // BLE Setup
  BLEDevice::init("ESP_1"); //Mac address: 1C:69:20:95:B9:DC
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);
  server->setCallbacks(new MyServerCallbacks());

  // TX characteristic - Notify (ESP32 sends data)
  txCharacteristic = service->createCharacteristic(
    TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pBLE2902_2 = new BLE2902();
  txCharacteristic->addDescriptor(pBLE2902_2);

  // RX characteristic - Write (App sends commands)
  rxCharacteristic = service->createCharacteristic(
    RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_WRITE
  );

  rxCharacteristic->setCallbacks(new RxCallbacks());

  timeCharacteristic = service->createCharacteristic(
    TIME_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_WRITE
  );

  timeCharacteristic->setCallbacks(new TimeCallbacks());

  setenv("TZ", "SAST-2", 1); // South Africa Standard Time (UTC+2)
  tzset();

  service->start();
  //service->getAdvertising()->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("BLE advertising started");
}


//-----------------------------------------------------------------------------


// Repeated loop
void loop() { 

  scale.set_scale(calibration_factor);

  float weights[]= { 
 1388.53, 1388.53, 1477.38, 1437.60, 1377.00, 1426.59, 1377.29, 1377.18, 1411.86, 1306.25,
 1315.48, 1372.45, 1350.37, 1415.40, 1355.51, 1330.80, 1471.82, 1388.94, 1403.31, 1330.19,
 1373.33, 1405.44, 1343.60, 1418.41, 1370.57, 1385.71, 1370.52, 1490.76, 1399.34, 1348.17, 
 1440.30, 1340.18, 1410.23, 1303.98, 1334.92, 1409.65, 1459.21, 1275.11, 1774.43, 944.43,
 1388.53, 1388.53, 1510.38, 1437.60, 1377.00, 1426.59, 1550.29, 1377.18, 1411.86, 1306.25,
 1315.48, 1372.45, 1350.37, 1415.40, 1500.51, 1330.80, 1471.82, 1388.94, 1403.31, 1330.19};
  while(checker==0){

    for (int i=0; i<sizeof(weights)/sizeof(weights[0]) ;i++){

    float weight = weights[i];
    //float weight = scale.get_units(5);
    weightBuffer[bufferIndex] = weight;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    if (bufferIndex == 0) {
      float mean = computeMean(weightBuffer, BUFFER_SIZE);
      float stdDev = computeStdDev(weightBuffer, BUFFER_SIZE, mean);
      float rsd = stdDev / mean;

      Serial.print("Mean: "); Serial.print(mean);
      Serial.print(" g, RSD: "); Serial.println(rsd);
     
      if (mean >= MIN_BIRD_WEIGHT && mean <= MAX_BIRD_WEIGHT && rsd <= RSD_THRESHOLD && rsd>=0.005 && birdDet==false) {
        Serial.print("Mean: "); Serial.print(mean);
        Serial.print(" g, RSD: "); Serial.println(rsd); 
        storeBirdData(mean); // Adding data with the timestamp
        Serial.println("Bird detected and stored!");
        birdDet = true;
      }
      else if(mean >= MIN_BIRD_WEIGHT && mean <= MAX_BIRD_WEIGHT && rsd <= RSD_THRESHOLD && rsd>=0.005 && birdDet==true){
        sameBird = true;
        Serial.println("Same Bird.");
      } 
      else if(rsd > RSD_THRESHOLD && birdDet==true || rsd < 0.005  && birdDet==true){
        sameBird = false;
        birdDet = false;
        Serial.println("Previous bird left.");
      }
      }
      
    }
      checker = 1;
      
  }
}
  // ---------- Helper Functions ----------

  float computeMean(float *data, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) sum += data[i];
    return sum / size;
  }

  float computeStdDev(float *data, int size, float mean) {
    float variance = 0;
    for (int i = 0; i < size; i++) {
      float diff = data[i] - mean;
      variance += diff * diff;
    }
    return sqrt(variance / size);
  }


  // Storing the data in the SPIFFS with the timestamp
  void storeBirdData(float weight) {

    time_t currentTime = getEstimatedTime();

    if (currentTime == 0) {
      Serial.println("Time not set. Skipping log.");
      return;
    }

    struct tm timeinfo;
    localtime_r(&currentTime, &timeinfo);

    char timeStr[25];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);

    File file = SPIFFS.open("/bird_log.csv", FILE_APPEND);
    if (file) {
      file.printf("%s, %.2f\n", timeStr, weight);
      file.close();
    } else {
      Serial.println("Failed to open file for writing."); 
    }

    Serial.print("Logged: ");
    Serial.println(timeStr);
  }

  //Function facilitating the sending of data
  void sendStoredData() {
  File file = SPIFFS.open("/bird_log.csv", FILE_READ);
  if (!file) {
    Serial.println("No CSV file found to send");
    return;
  }

  String fileContent = "";
  while (file.available()) {
    fileContent += (char)file.read();
  }
  file.close();

  Serial.println("Starting CSV data transmission...");

  sendChunks(fileContent);

  Serial.println("CSV data transmission complete.");
}

time_t getEstimatedTime() {
  if (rebootTimestamp == 0) return 0;  // time not set

  unsigned long secondsSinceSync = (millis() - bootMillis) / 1000;
  return rebootTimestamp + secondsSinceSync;
}

// -----------------------ESP-NOW functions----------------------------------

void sendQueryToMesh(const String& target) {
  DynamicJsonDocument doc(256);
  doc["type"] = "QUERY";
  doc["origin"] = "ESP_1";
  doc["target"] = target;
  doc["ttl"] = 5;
  doc["rebootTimeStamp"] = rebootTimestamp;
  JsonArray path = doc.createNestedArray("hop_path");
  path.add("ESP_1");

  String jsonStr;  
  serializeJson(doc, jsonStr);  //Take the Json document and save the contents as a string

  message_t msg;
  jsonStr.toCharArray(msg.json, sizeof(msg.json)); //use message struct to write string into an array


  Serial.println("Broadcasting query message to peers.");
  // Broadcast to all peers

  esp_err_t result = esp_now_send(broadcast, (uint8_t *)&msg, sizeof(msg));
  if (result == ESP_OK) {
    Serial.println("Sent successfully");
  } else {
    Serial.printf("Send failed: %d\n", result);
  }
}

void sendDeleteToMesh() { 
  DynamicJsonDocument doc(256);
  doc["type"] = "DELETE";
  doc["origin"] = "ESP_1";
  doc["target"] = currentNode;
  doc["ttl"] = 5;
  JsonArray path = doc.createNestedArray("hop_path");
  path.add("ESP_1");

  String jsonStr;  
  serializeJson(doc, jsonStr);  //Take the Json document and save the contents as a string

  message_t msg;
  jsonStr.toCharArray(msg.json, sizeof(msg.json)); //use message struct to write string into an array

  Serial.println("Broadcasting delete message to peers.");
  // Broadcast to all peers
  esp_err_t result = esp_now_send(broadcast, (uint8_t *)&msg, sizeof(msg));
  if (result == ESP_OK) {
    Serial.println("Sent successfully");
  } else {
    Serial.printf("Send failed: %d\n", result);
  }
}

// Callback to handle incoming ESP-NOW messages (Might put this under call backs)
void onESPNowReceive(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, data, len);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  Serial.println("Received from mesh.");

  String type = doc["type"];
  if (type == "RESPONSE") {
    String csvData = doc["data"];
    JsonArray path = doc["hop_path"];

    Serial.println("Received data from node.");
    Serial.println("Path taken:"); //only to troubleshoot whether the closest is being taken and that there isn't a loop
    for (JsonVariant hop : path) {
      Serial.print(hop.as<String>() + " -> ");
    }
    Serial.println("ESP_1");

    sendChunks(csvData);
  }
  else if(type == "NODE_DOWN"){
    String downESP = doc["target"];
    JsonArray path = doc["hop_path"];

    Serial.println("The node " + String(downESP) + " is down.");
    Serial.println("Path taken:"); //only to troubleshoot whether the closest is being taken and that there isn't a loop
    for (JsonVariant hop : path) {
      Serial.print(hop.as<String>() + " -> ");
    }
    Serial.println("ESP_1");

    String downMsg = "Down: " + downESP;

    txCharacteristic->setValue(downMsg.c_str());
    txCharacteristic->notify();
  }
}

void sendChunks(String data){
  // Send file contents in chunks of ~100 bytes to avoid BLE payload limit (~512 bytes max)

  int totalLength = data.length();
  char buffer[181];  // chunkSize + 1 for null-terminator
  Serial.println("Sending chunks.");

  for (int i = 0; i < totalLength; i += 180) {
    int len = min(180, totalLength - i);  
    // Extract substring and convert to char array
    data.substring(i, i + len).toCharArray(buffer, len + 1); // +1 for '\0'
    
    // Send the chunk
    txCharacteristic->setValue((uint8_t*)buffer, len);
    txCharacteristic->notify();
    delay(50);  // Short delay to avoid BLE congestion
  }

  Serial.println("Done sending chunks");
  // Send EOF marker to indicate end of transmission
  const char* eofMarker = "END_OF_FILE";
  txCharacteristic->setValue((uint8_t*)eofMarker, strlen(eofMarker));
  txCharacteristic->notify();
}


