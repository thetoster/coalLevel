#include <Arduino.h>
#include <UltraDistSensor.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

const size_t VALUES_COUNT = 4;

//pins D2-D12 -> LED Bar
//pin A1 - distance trigger
//pin A2 - distance echo
//pin 13 - connect to BT TX
//pin A5 - connect to BT RX

constexpr int btRx = 13;
constexpr int btTx = A5;

UltraDistSensor distSensor;
float readValues[VALUES_COUNT] = { 0 };
size_t index = 0;
SoftwareSerial hmSerial(btRx, btTx);
int level = 0;

struct Limits {
  float minVal;
  float maxVal;
  uint16_t checksum;
};

Limits limits;

void loadLimits();
void initHM();

void setup() {
  Serial.begin(9600);
  distSensor.attach(A1, A2); //Trigger pin , Echo pin
  distSensor.changeTimeout(23200);
  loadLimits();
  //D3-D12
  for(int t = 3; t <= 12; t++) {
    pinMode(t, OUTPUT);
  }
  initHM();
}

void execHMCmd(String cmd) {
  Serial.print("HM:");
  Serial.print(cmd);
  hmSerial.write(cmd.c_str(), cmd.length());
  Serial.print(" >");
  delay(400);

  //consume Response
  int c;
  while(true) {
    c = hmSerial.read();
    if (c < 0) {
      break;
    }
    Serial.print(static_cast<char>(c));
  }
  Serial.println("<\n");
}

void initHM() {
  pinMode(btRx, INPUT);
  pinMode(btTx, OUTPUT);
  hmSerial.begin(9600);
  hmSerial.setTimeout(1300);
  delay(500);
  execHMCmd("AT");
  execHMCmd("AT+RENEW");
  execHMCmd("AT+NAMECOAL");
  execHMCmd("AT+RESET");
  delay(500);
  execHMCmd("AT");
  execHMCmd("AT+NAME?");
  execHMCmd("AT+ADDR?");
}

uint16_t calcChecksum(float minVal, float maxVal) {
  float h = minVal * 31 + maxVal * 61;
  uint16_t* iH1 = reinterpret_cast<uint16_t*>(&h);
  uint16_t* iH2 = iH1 + 1;
  uint16_t checksum = (*iH1) ^ (*iH2);
  return checksum;
}

void saveLimits() {
  limits.checksum = calcChecksum(limits.minVal, limits.maxVal);
  EEPROM.put(0, limits);
}

void setDefaultLimits() {
  limits.minVal = 10.0f;
  limits.maxVal = 90.0f;
  saveLimits();
}

void loadLimits() {
  EEPROM.get(0, limits);
  if (((limits.minVal < 0.01) and (limits.maxVal < 0.01))
      or (limits.checksum != calcChecksum(limits.minVal, limits.maxVal))) {
    setDefaultLimits();
  }
}

int convertToLevel(float reading) {
  if (reading < limits.minVal) {
    return 10;
  }
  if (reading > limits.maxVal) {
    return 0;
  }
  reading = reading - limits.minVal;
  float diff = (limits.maxVal - limits.minVal) / 10;
  diff = diff <= 0 ? 0.1f : diff;

  //lower level value means empties container
  return 10 - static_cast<int>(roundf(reading / diff));
}

void updateLeds(int level) {
  int state = HIGH;
  if (level <= 2) {
    //alarm level
    state = ((millis() % 2000) < 1000) ? HIGH : LOW;
  }

  level += 2; // shift to match pin numbers
  //D2-D12
  for(int t = 3; t <= 12; t++) {
    digitalWrite(t, t <= level ? state : LOW);
  }
}

void updateBT(int level) {
  hmSerial.write(level);
}

void processReadings() {
  float reading = distSensor.distanceInCm();
  if (reading > 0.1f) {
    readValues[index] = reading;
    index++;
    if (index >= VALUES_COUNT) {
      index = 0;
    }

    //calc mean
    reading = 0;
    for (int t = 0; t < 4; t++) {
      reading += readValues[t];
    }
    reading /= VALUES_COUNT;

    //apply limits
    level = convertToLevel(reading);

    //set bluetooth
    updateBT(level);

    //debug dump
    Serial.print(reading);
    Serial.print(" CM -> ");
    Serial.println(level);
  }
}

float tryGetValue(String& str, float defVal) {
  str.remove(0, 1);
  int val = static_cast<int>(str.toInt());
  if ((val > 150) or (val < 10)) {
    return defVal;
  }
  return val;
}

void processBTCommand() {
  if (hmSerial.available() <= 0) {
    return;
  }
  String cmd = hmSerial.readString();
  if (cmd[0] == 'U' or cmd[0] == 'u') {
    limits.maxVal = tryGetValue(cmd, limits.maxVal);
    Serial.print("New max dist:");
    Serial.println(limits.maxVal);
    saveLimits();

  } else if (cmd[0] == 'L' or cmd[0] == 'l') {
    limits.minVal = tryGetValue(cmd, limits.minVal);
    Serial.print("New min dist:");
    Serial.println(limits.minVal);
    saveLimits();
  }
}

void loop() {
  processReadings();
  processBTCommand();
  updateLeds(level);
  delay(1000);
}
