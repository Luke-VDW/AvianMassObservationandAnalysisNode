#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

// Calibration factor - adjust this based on your load cell
float calibration_factor = 420.0;  // This value is an example — you'll need to find yours
float known_weight = 0.0;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("HX711 Calibration and Weight Reading");
  Serial.println("Place the load cell on a stable surface...");
  
  while (!scale.is_ready()) {
    Serial.println("Waiting for the scale...");
    delay(500);
  }

  scale.set_scale(); // initialize scale without calibration
  scale.tare();      // Reset the scale to 0

  Serial.println("Scale is ready.");
  Serial.println("Send 't' to tare, 'c' to calibrate, 'r' to reset calibration factor.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == 't') {
      scale.tare();
      Serial.println("Tare done. Scale reset to 0.");
    }
    else if (cmd == 'r') {
      calibration_factor = 420.0;
      scale.set_scale(calibration_factor);
      Serial.println("Calibration factor reset.");
    }
    else if (cmd == 'c') {
      Serial.println("Place a known weight on the scale (in kg), then enter its weight:");
      while (!Serial.available());
      known_weight = Serial.parseFloat();
      long reading = scale.get_units(10); // take average of 10 readings
      calibration_factor = reading / known_weight;
      scale.set_scale(calibration_factor);
      Serial.print("New calibration factor set: ");
      Serial.println(calibration_factor);
    }
  }

  scale.set_scale(calibration_factor);
  float weight = scale.get_units(5); // average 5 readings
  Serial.print("Weight: ");
  Serial.print(weight, 3); // 3 decimal places
  Serial.println(" kg");

  delay(500);
}
