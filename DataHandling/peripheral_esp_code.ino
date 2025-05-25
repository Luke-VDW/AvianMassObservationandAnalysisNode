#include <WiFi.h>
#include <esp_now.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <HX711.h>
#include "FS.h"
#include <time.h>
#include <esp_wifi.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

 // Unique name for this ESP
const char* nodeName = "ESP_2"; //MAC Address: 94:3C:C6:99:D9:90

// This value is an example — you'll need to find yours
float calibration_factor = 420.0;  
float known_weight = 0.0;

// -------------------------------Time---------------------------------------------
unsigned long rebootTimestamp = 1747941600;  // from BLE
unsigned long bootMillis = 0;        // captured when time is set


//--------------------------------Detection----------------------------------------
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

uint8_t broadcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t esp1[] = {0x1C, 0x69, 0x20, 0x95, 0xB9, 0xDC};

//-------------------------Function Definitions-------------------------------------
void onESPNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void sendMessageBackThroughPath(DynamicJsonDocument &msg, JsonArray &hopPath);
float computeMean(float *data, int size);
float computeStdDev(float *data, int size, float mean);
void storeBirdData(float weight);

// --------------------------ESP-NOW message structure-----------------------------
typedef struct message_t {
  char json[250];
} message_t;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  scale.set_scale(); // initialize scale without calibration
  scale.tare();      // Reset the scale to 0
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  Serial.println("Scale is ready.");

  bootMillis = millis();  // capture when fallback time starts

  esp_log_level_set("task_wdt", ESP_LOG_NONE);

  WiFi.mode(WIFI_STA);

  WiFi.STA.begin();

  int channel = WiFi.channel();
  Serial.printf("WiFi channel: %d\n", channel);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0;    // Use current channel
  peerInfo.encrypt = false;

  uint8_t* macs[] = {esp1, broadcast};
  const char* labels[] = {"ESP_1", "Broadcast"};

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
  Serial.println("Peer node using JSON ready.");
}

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

time_t getEstimatedTime() {
  if (rebootTimestamp == 0) return 0;  // time not set

  unsigned long secondsSinceSync = (millis() - bootMillis) / 1000;
  return rebootTimestamp + secondsSinceSync;
}

// -------------------------------ESP-NOW Functions----------------------------------

void onESPNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, data, len);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  Serial.println("Received by " + String(nodeName));

  const char* type = doc["type"];
  Serial.println(type);
  const char* target = doc["target"];
  Serial.println(target);
  const char* origin = doc["origin"];
  int ttl = doc["ttl"];
  rebootTimestamp = doc["rebootTimeStamp"]; //Too frequent or???
  bootMillis = millis();
  JsonArray hopPath = doc["hop_path"];


  if (String(target) == "ESP_2") {
    Serial.println("It saw that it is the target.");
    if (String(type) == "QUERY") {
      Serial.println("Target matched. Preparing response...");

      File file = SPIFFS.open("/bird_log.csv");
      if (!file) {
        Serial.println("File open failed");
        return;
      }

      String content = file.readString();
      file.close();

      DynamicJsonDocument respDoc(512);
      respDoc["type"] = "RESPONSE";
      respDoc["origin"] = nodeName;
      respDoc["target"] = origin;       // Send it back to origin
      respDoc["ttl"] = ttl - 1;
      respDoc["data"] = content;

      JsonArray respPath = respDoc.createNestedArray("hop_path");
      for (JsonVariant v : hopPath) {
        respPath.add(v.as<String>());
      }
      respPath.add(nodeName);

      sendMessageBackThroughPath(respDoc, respPath, ttl);
    }
    else if (String(type) == "DELETE") {
      if (SPIFFS.exists("/bird_log.csv")) {
        SPIFFS.remove("/bird_log.csv");
        Serial.println("ESP_2 CSV deleted on request.");
      }
    }
  }
  else if ((ttl - 1) == 0) {
    // TTL expired and not the target
    Serial.println("TTL expired. Sending NODE_DOWN.");

    DynamicJsonDocument downDoc(256);
    downDoc["type"] = "NODE_DOWN";
    downDoc["origin"] = nodeName;
    downDoc["target"] = origin;

    JsonArray downPath = downDoc.createNestedArray("hop_path");
    for (JsonVariant v : hopPath) {
      downPath.add(v.as<String>());
    }
    downPath.add(nodeName);

    //String downJson;
    //serializeJson(downDoc, downJson);
    sendMessageBackThroughPath(downDoc, downPath, ttl);
  }
  else if (String(type) == "RESPONSE") {
    Serial.println("Entered Response.");
    sendMessageBackThroughPath(doc, hopPath, ttl);
    Serial.println("Sending back to gateway.");
  }
  else {
   Serial.println("Forwarding QUERY.");

    // Already visited?
    for (JsonVariant v : hopPath) {
      if (v.as<String>() == nodeName) {
        Serial.println("Loop detected. Dropping packet.");
      return;
      }
    }

    DynamicJsonDocument fwdDoc(512);
    fwdDoc["type"] = "QUERY";
    fwdDoc["origin"] = origin;
    fwdDoc["target"] = target;
    fwdDoc["ttl"] = ttl - 1;

    JsonArray fwdPath = fwdDoc.createNestedArray("hop_path");
    for (JsonVariant v : hopPath) {
      fwdPath.add(v.as<String>());
    }
    fwdPath.add(nodeName);

    String fwdJson;
    serializeJson(fwdDoc, fwdJson);

    message_t msg;
    fwdJson.toCharArray(msg.json, sizeof(msg.json)); //use message struct to write string into an array

    // Broadcast to all peers
    esp_err_t result = esp_now_send(broadcast, (uint8_t *)&msg, sizeof(msg));
    if (result == ESP_OK) {
      Serial.println("Sent successfully");
    } else {
      Serial.printf("Send failed: %d\n", result);
    }
  }
}

// ------------------------Supporting Functions------------------------------------

void sendMessageBackThroughPath(DynamicJsonDocument &msg, JsonArray &hopPath, int &ttl) {
  // Find the index of this node in the hop_path
    int thisIndex = -1;
    for (int i = 0; i < hopPath.size(); i++) {
      if (hopPath[i].as<String>() == nodeName) {
        thisIndex = i;
        break;
      }
    }

    if (thisIndex == -1) {
      Serial.println("This node not in hop_path. Dropping RESPONSE.");
      return;
    }

    // We are in the middle of the hop path — forward to previous node
    const char* nextTarget = hopPath[thisIndex - 1].as<const char*>();

    msg["target"] = nextTarget;
    msg["ttl"] = ttl - 1;

    // Convert to string
    String jsonStr;
    serializeJson(msg, jsonStr);

    message_t docMSG;
    jsonStr.toCharArray(docMSG.json, sizeof(docMSG.json)); //use message struct to write string into an array

    // Broadcast to peers
    esp_err_t result = esp_now_send(broadcast, (uint8_t *)&docMSG, sizeof(docMSG));
    if (result == ESP_OK) {
      Serial.println("Sent successfully");
    } else {
      Serial.printf("Send failed: %d\n", result);
    } 

    Serial.printf("Forwarding RESPONSE to %s\n", nextTarget);
    return;
}

