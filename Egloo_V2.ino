#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Adafruit_NeoPixel.h>

#define PIN 14 // Pin where NeoPixels are connected
#define PIXELCOUNT 200
Adafruit_NeoPixel strip(PIXELCOUNT, PIN, NEO_GRB + NEO_KHZ800);

#define NANO_33_BLE_SERVICE_UUID(val) ("d91cb6ee-" val "-11ea-87d0-0242ac130004")

BLEService            service                 (NANO_33_BLE_SERVICE_UUID("d174"));
BLECharacteristic     InputCharacteristic     (NANO_33_BLE_SERVICE_UUID("5001"), BLENotify, 5 * sizeof(uint16_t));
BLECharacteristic     OutputCharacteristic    (NANO_33_BLE_SERVICE_UUID("5002"), BLEWrite, 5 * sizeof(uint8_t));
BLECharacteristic     NeopixCharacteristic    (NANO_33_BLE_SERVICE_UUID("5003"), BLEWrite, 3 * sizeof(uint8_t));

const int Input[5] = {15, 16, 17, 20, 21};
const int Output[5] = {6, 7, 8, 9, 10};
const uint8_t RGB[3] = {50, 50, 50};
int inputConf[5] = {1, 1, 1, 0, 0};
uint16_t currentInput[5] = {0, 0, 0, 0, 0};
uint16_t lastInput[5] = {1, 1, 1, 1, 1};
uint8_t color[3] = {50, 50, 50};
uint16_t BLEinput[5] = {0, 0, 0, 0, 0};
uint16_t lastBLEinput[5] = {1, 1, 1, 1, 1};
bool bleConnected = false;

void writeOutput(uint8_t out, uint8_t val){
  if (val == 0) {
    analogWrite(Output[out], 256);
  } else {
    analogWrite(Output[out], 255 - val);
  }
}

// Configure Input as needed. 0 = Off, 1 = active/HIGH, 2 = active/LOW with Pullup, 3 = Analog Input 0-1024
// please match pinMode in setup with your configuration
void configureInput(int input, uint8_t mode){
  Serial.print("In");
  Serial.print(input);
  Serial.print("\t");
  Serial.println(mode);
  switch(mode){
    case 0: 
      pinMode(Input[input], INPUT);
      break;
    case 1: 
      pinMode(Input[input], INPUT_PULLDOWN);
      break;
    case 2: 
      pinMode(Input[input], INPUT_PULLUP);
      break;
    case 3: 
      pinMode(Input[input], INPUT);
      break;
    default:
      break;
  }
}

bool readInput() {
  bool hasChanged = false;
  // Serial.print("Func\t");
  for(int i = 0; i <= 4; i++) {
    switch(inputConf[i]){
      case 0:
        break;
      case 1:
        currentInput[i] = (uint16_t)digitalRead(Input[i]);
        break;
      case 2:
        currentInput[i] = (uint16_t)!digitalRead(Input[i]);
        break;
      case 3:
        currentInput[i] = (uint16_t)analogRead(Input[i]);
        break;
      default:
        break;
    }
    if (currentInput[i] != lastInput[i]) {
      hasChanged = true;
      lastInput[i] = currentInput[i];
    }
    // Serial.print(currentInput[i]);
    // Serial.print(F("\t"));
  }
  //Serial.println();
  return hasChanged;
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

uint16_t readInput(uint8_t input) {
  switch(inputConf[input]){
    case 1:
      return (uint16_t)digitalRead(Input[input]);
      break;
    case 2:
      return (uint16_t)!digitalRead(Input[input]);
      break;
    case 3:
      return (uint16_t)analogRead(Input[input]);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  //while(!Serial) delay(200);
  delay(500);
  // set Input Pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LED_PWR, HIGH);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);
  pinMode(Input[0], INPUT_PULLDOWN);
  delay(100);
  pinMode(Input[1], INPUT_PULLDOWN);
  delay(100);
  pinMode(Input[2], INPUT_PULLDOWN);
  delay(100);
  pinMode(Input[3], INPUT_PULLDOWN);
  delay(100);
  pinMode(Input[4], INPUT_PULLDOWN);
  delay(100);
  pinMode(Output[4], OUTPUT);
  // setup and test Neopixels
  strip.begin();
  strip.setBrightness(127);
  strip.fill(strip.Color(NeopixCharacteristic[0], NeopixCharacteristic[1], NeopixCharacteristic[2]), 0, PIXELCOUNT);
  strip.show();
  rainbowCycle(1);
  strip.clear();
  strip.show();
  // setup BLE
  if (!BLE.begin()) {
    Serial.println(F("starting BLE failed!"));
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    while (true);
  } else {
    Serial.println(F("starting BLE OK"));
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
  }
  Serial.print(F("Peripheral address: "));
  Serial.println(BLE.address());
  Serial.println(service);
  BLE.setLocalName("Egloo");
  BLE.setDeviceName("Egloo");
  uint8_t data[5] = {'E', 'G', 'L', 'O', 'O'};
  BLE.setManufacturerData(data, sizeof(data));
  BLE.setAdvertisedService(service);
  service.addCharacteristic(InputCharacteristic);
  service.addCharacteristic(OutputCharacteristic);
  service.addCharacteristic(NeopixCharacteristic);
  BLE.addService(service);
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    if (bleConnected == false) {
      Serial.println(F("BLE CONNECTED"));
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, LOW);
      bleConnected = true;
    }
    if (InputCharacteristic.subscribed() && readInput()){
      InputCharacteristic.writeValue(currentInput, sizeof(currentInput));
    }
    if (OutputCharacteristic.written()) {
      /*Serial.print(F("OUT received :"));
      Serial.print(F("\t"));
      Serial.print(OutputCharacteristic[0]);
      Serial.print(F("\t"));
      Serial.print(OutputCharacteristic[1]);
      Serial.print(F("\t"));
      Serial.print(OutputCharacteristic[2]);
      Serial.print(F("\t"));
      Serial.print(OutputCharacteristic[3]);
      Serial.print(F("\t"));
      Serial.println(OutputCharacteristic[4]);*/

      // Output 1-4 are analog
      analogWrite(Output[0], OutputCharacteristic[0]);
      analogWrite(Output[1], OutputCharacteristic[1]);
      analogWrite(Output[2], OutputCharacteristic[2]);
      analogWrite(Output[3], OutputCharacteristic[3]);

      // Output 5 is digital
      bool out4;
      if (OutputCharacteristic[4] >= 1){
        out4 = true;
      } else {
        out4 = false;
      }
      digitalWrite(Output[4], out4);
    }

    // write color to neopixels
    if (NeopixCharacteristic.written()) {
      Serial.print("LED writen :\t");
      Serial.print(NeopixCharacteristic[0]);
      Serial.print("\t");
      Serial.print(NeopixCharacteristic[1]);
      Serial.print("\t");
      Serial.println(NeopixCharacteristic[2]);
      strip.fill(strip.Color(NeopixCharacteristic[0], NeopixCharacteristic[1], NeopixCharacteristic[2]), 0, PIXELCOUNT);
      strip.show();
      delay(10);
    }
    // update strip even if nothing has changed to prevent comunication problems
    //strip.show();
    //delay(30);
  }

  // set onboard led blue if connection is lost
  if (!central && bleConnected) {
    bleConnected = 0;
    Serial.println(F("Disconnected !"));
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
  }
}
