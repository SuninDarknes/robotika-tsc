#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.setPin("7957");
  SerialBT.begin("ESP32test");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}


void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  /*
  BMS int //Brushless motor speed 50-90
  GSP int //Gripper Servo Position
  MSP int //Mid Servo Position
  BSP int //Bottom Servo Position

  LSMS int // Left Stepper Max Speed
  RSMS int // Right Stepper Max Speed
  */
}
