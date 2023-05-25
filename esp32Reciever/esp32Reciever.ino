#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  SerialBT.setPin("7957");
  SerialBT.begin("Vector 2.0 Upravljač");
  Serial.println("ESP32 je pokrenut!");
}


void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  /*
  S int //Brushless motor speed 50-90
  G int //Gripper Servo Position
  M int //Mid Servo Position
  B int //Bottom Servo Position

  L int // Left Stepper Speed
  R int // Right Stepper Speed
  */
}
