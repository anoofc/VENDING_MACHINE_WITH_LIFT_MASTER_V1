#define DEBUG       0

#define TRAY1       33
#define TRAY2       25
#define TRAY3       26
#define TRAY4       27
#define TRAY5       14
#define TRAY6       13

#define SLOT1       23
#define SLOT2       22
#define SLOT3       21
#define SLOT4       19
#define SLOT5       18
#define SLOT6       4

#define IR_SENSOR   32

#define DISPENCE_TIMEOUT  20000
#define MIN_TIMEOUT       900

#define SENSOR_DETECT  digitalRead(IR_SENSOR) == 1

#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

uint8_t trayPins[] = {TRAY1, TRAY2, TRAY3, TRAY4, TRAY5, TRAY6};
uint8_t slotPins[] = {SLOT1, SLOT2, SLOT3, SLOT4, SLOT5, SLOT6};

uint32_t disUpdateMillis = 0;

bool     disActive = false;

void debugSensorPin(){
  Serial.println("IR Sensor: " + String(digitalRead(IR_SENSOR))); 
}

void testAll () {
  for (int i = 0; i < 6; i++){
    digitalWrite(trayPins[i], HIGH);
    digitalWrite(slotPins[i], LOW);
    delay(1000);
    digitalWrite(trayPins[i], LOW);
    digitalWrite(slotPins[i], HIGH);
  }
}

void testTray(int tray){
  digitalWrite(trayPins[tray], HIGH);
  delay(1000);
  digitalWrite(trayPins[tray], LOW);
}

void testSlot(int slot){
  digitalWrite(slotPins[slot], LOW);
  delay(1000);
  digitalWrite(slotPins[slot], HIGH);
}

void haltDispence(){
  for (int i = 0; i < 6; i++){
    digitalWrite(trayPins[i], LOW);
    digitalWrite(slotPins[i], HIGH);
  }
  disActive = false;
  if (DEBUG){ Serial.println("Dispence Halted!!"); }
}

void checkSensor(){
  // if (disActive && SENSOR_DETECT && millis() - disUpdateMillis < MIN_TIMEOUT ){ Serial.println(); }
  if (disActive && SENSOR_DETECT && millis() - disUpdateMillis > MIN_TIMEOUT ){ Serial2.println("%Z#"); haltDispence(); }
}

void checkDispenceTimeout(){
  if (disActive && millis() - disUpdateMillis > DISPENCE_TIMEOUT){ Serial.println(); Serial.println("*ERROR#");  Serial2.println("%Z#"); haltDispence(); }
}

void dispence(uint8_t tray, uint8_t slot){
  digitalWrite(trayPins[tray], HIGH);
  digitalWrite(slotPins[slot], LOW);
  disActive = true;
  disUpdateMillis = millis();
}


void checkData (String data){
  if (data == "A1"){ dispence(0, 0); dispence(0, 1);  } 
  else if (data == "A2"){ dispence(0, 2); dispence(0, 3); }
  else if (data == "A3"){ dispence(0, 4); dispence(0, 5); } 
  else if (data == "B1"){ dispence(1, 0); dispence(1, 1); } 
  else if (data == "B2"){ dispence(1, 2); } 
  else if (data == "B3"){ dispence(1, 3); } 
  else if (data == "B4"){ dispence(1, 4); dispence(1, 5); } 
  else if (data == "C1"){ dispence(2, 0); } 
  else if (data == "C2"){ dispence(2, 1); } 
  else if (data == "C3"){ dispence(2, 2); dispence(2, 3); }
  else if (data == "C4"){ dispence(2, 4); } 
  else if (data == "C5"){ dispence(2, 5); } 
  else if (data == "D1"){ dispence(3, 0); dispence(3, 1); } 
  else if (data == "D2"){ dispence(3, 2); dispence(3, 3); } 
  else if (data == "D3"){ dispence(3, 4); }
  else if (data == "D4"){ dispence(3, 5); } 
  else if (data == "E1"){ dispence(5, 0); } 
  else if (data == "E2"){ dispence(5, 1); } 
  else if (data == "E3"){ dispence(5, 2); } 
  else if (data == "E4"){ dispence(5, 3); }
  else if (data == "E5"){ dispence(5, 4); }
  else if (data == "E6"){ dispence(5, 5); }
}

void processData(String data){
  if (data.startsWith("*") && data.endsWith("#")){
    Serial2.println(data);
    if (DEBUG){ Serial.println("Waiting for Lift..."); }
  } else if (data.startsWith("%") && data.endsWith("#")){
    if (data.substring(1) == "Z"){ haltDispence(); }
  }
}

void processBTData(String data){
  if (data.startsWith("*") && data.endsWith("#")){
    char trayChar = data.charAt(1);
    uint8_t tray = trayChar - 'A';                // Convert 'A' to 0, 'B' to 1, etc.
    uint8_t slot = data.substring(2).toInt() - 1; // Convert '1' to 0, '2' to 1, etc.
    SerialBT.println("Tray: "+ String(tray) + " Slot: " + String(slot));
    dispence(tray, slot);
  } else if (data.startsWith("%") && data.endsWith("#")){
    if (data.substring(1) == "Z"){ haltDispence(); }
  }
}

void processSerial2Data(String data){
  if (data == "$X#"){ Serial.println(); Serial.println("*SUCCESS#"); }
  if (data == "*ERROR#"){ Serial.println(); Serial.println("*ERROR#");  } 
  else if (data.startsWith("*") && data.endsWith("#")){
    String disData = data.substring(1, data.length() - 1);
    if (DEBUG){ Serial.println("Dispencing: " + disData); }
    checkData(disData);
  } else if (data.startsWith("%") && data.endsWith("#")){
    if (data.substring(1) == "Z"){ haltDispence(); }
  }
}

void readSerial2(){
  if (Serial2.available()){
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim();
    if (DEBUG){ Serial.print("Serial2: "); Serial.println(incoming); }
    processSerial2Data(incoming);
  }
}

void readSerialBT(){
  if (SerialBT.available()){
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();
    if (DEBUG){ Serial.print("SerialBT: "); Serial.println(incoming); }
    processBTData(incoming);
  }
}

void readSerial(){
  if (Serial.available()){
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();
    processData(incoming);
    if (DEBUG){ Serial.print("PC Serial: "); Serial.println(incoming); }
  }
}

void io_init(){
  pinMode(IR_SENSOR, INPUT_PULLUP);

  for (int i = 0; i < 6; i++){
    pinMode(trayPins[i], OUTPUT);
    pinMode(slotPins[i], OUTPUT);
    digitalWrite(trayPins[i], LOW);
    digitalWrite(slotPins[i], HIGH);
  }
}


void setup() {
  io_init();
  Serial.begin(9600);
  Serial2.begin(115200);
  SerialBT.begin("ITS_VENDING_1");
  

}

void loop() {
  readSerial2();
  readSerialBT();
  readSerial();
  checkSensor();
  checkDispenceTimeout();
  // debugSensorPin();
  // testSerial2();
}