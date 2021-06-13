#include <Wire.h>
#include <Adafruit_GFX.h>     // https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h> // https://github.com/adafruit/Adafruit_SSD1306
#include <ESP32Encoder.h>     // https://github.com/madhephaestus/ESP32Encoder/
#include <Keypad.h>           // https://github.com/Chris--A/Keypad
#include <BleGamepad.h>       // https://github.com/lemmingDev/ESP32-BLE-Gamepad



//////////////////////////////////////////////////
// Battery Polling Intervals
//////////////////////////////////////////////////
long batteryUpdateInterval = 300000; // every 5 minutes
// long batteryUpdateInterval = 1000; // 1 Hz (testing/fun)
long prevBatteryUpdate = 295000; // 5 seconds from goal



//////////////////////////////////////////////////
// OLED Screen Setup
//////////////////////////////////////////////////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
#define CUSTOM_I2C_SDA 25
#define CUSTOM_I2C_SCL 26
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



//////////////////////////////////////////////////
// BLE Gamepad Setup
//////////////////////////////////////////////////
BleGamepad bleGamepad("BLE Sim Buttons", "Arduino");



//////////////////////////////////////////////////
// Custom Keypad Matrix Setup
//////////////////////////////////////////////////
#define ROWS 5
#define COLS 4
uint8_t rowPins[ROWS] = {15, 32, 14, 22, 23};
uint8_t colPins[COLS] = {4, 12, 27, 33};
byte keymap[ROWS][COLS] = { // buttons
  { 0, 1, 2, 3},
  { 5, 6, 7, 8},
  { 9,10,11,12},
  {13,14,15,16},
  {17,18,19,20}
};
Keypad customKeypad = Keypad( makeKeymap(keymap), rowPins, colPins, ROWS, COLS); 



//////////////////////////////////////////////////
// John Main "Robust Rotary encoder" Setup
//////////////////////////////////////////////////
// The `PEC11H-4020F-S0016` encoders I used from the BOM are
// very noisy and cause issues and double counts when used
// with the ESP32Encoder library. John's code works well!
#define ENCODER_COUNT 2
uint8_t uppPin[ENCODER_COUNT] = {16, 18}; // A pins
uint8_t dwnPin[ENCODER_COUNT] = {19, 5};  // B pins
uint8_t encCount[ENCODER_COUNT] = {0, 0};
uint8_t encValue[ENCODER_COUNT] = {0, 0};
uint8_t encPrevNextCode[ENCODER_COUNT] = {0, 0};
uint16_t encStore[ENCODER_COUNT]= {0, 0};
uint8_t encoderUpp[ENCODER_COUNT] = {23,25}; // Up buttons
uint8_t encoderDwn[ENCODER_COUNT] = {24,26}; // Down buttons
static int8_t rotEncTable[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};



//////////////////////////////////////////////////
// ESP32Encoder Setup
//////////////////////////////////////////////////
// The `RKJXT1F42001` encoder works really well with the
// ESP32Encoder library, and does not work very well with
// the implemented "Robust Rotary encoder" solution.
unsigned long funkyEncoderHoldoff = 0;
int32_t funkyEncoderPrevCenter = 0;
uint8_t funkyEncoderUppPin = 21; // A pin
uint8_t funkyEncoderDwnPin = 17; // B pin
uint8_t funkyEncoderUpp = 22; // Up button
uint8_t funkyEncoderDwn = 21; // Down button
#define FUNKY_HOLDOFF_TIME 30
ESP32Encoder funkyEncoder;



//////////////////////////////////////////////////
// Funky Switch Direction Logic Setup
//////////////////////////////////////////////////
#define FUNKY_DIR_COUNT 4
#define FUNKY_DIR_HOLDOFF_TIME 300 // 0.3sec
unsigned long funkyDirectionLastHoldoff[4] = {0,0,0,0};
char funkyCenterCode = 0;
char funkyDirectionCodes[FUNKY_DIR_COUNT] = {13, 9, 5, 17};
bool handlingFunkySwitch = false;



void setup() {
  Serial.begin(115200);
  
  // Custom i2c pins and address for Adafruit OLED library
  Wire.begin(CUSTOM_I2C_SDA, CUSTOM_I2C_SCL, SCREEN_ADDRESS);

  // Initialize the OLED display
  display.begin();

  // Setup encoder logic for "John Main" encoders:
  // https://esp32.com/viewtopic.php?f=19&t=2881&start=30;
  for (uint8_t i=0; i<ENCODER_COUNT; i++) {
    pinMode(uppPin[i], INPUT);
    pinMode(uppPin[i], INPUT_PULLUP);
    pinMode(dwnPin[i], INPUT);
    pinMode(dwnPin[i], INPUT_PULLUP);
  }

  // Attach ESP32Encoder pins for "funky switch" (RKJXT1F42001) encoder
  funkyEncoder.attachHalfQuad(funkyEncoderUppPin, funkyEncoderDwnPin);
  funkyEncoder.clearCount();

  // Initialize Keypad Matrix & Options
  customKeypad.setHoldTime(7000); // 7 seconds is considered holding
  
  // Initialize BLE Gamepad
  bleGamepad.begin();

  Serial.println("Booted!");

  // Cheesy OLED boot message
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Ignition,");
  display.println("Race Mode");
  display.display();
  delay(3000);
  display.clearDisplay();
}


void loop() {
  unsigned long now = millis();

  // Loop through "encoder table" encoders
  for (uint8_t i=0; i < ENCODER_COUNT; i++) {
    if(encValue[i] = readRotary(uppPin[i], dwnPin[i], i)) {
      encCount[i] +=encValue[i];
      Serial.print(encCount[i]);Serial.print(" ");

      if ( encPrevNextCode[i]==0x0b) {
          sendKey(encoderUpp[i]);
          Serial.print("UP ");
          Serial.println(encStore[i],HEX);
      }

      if ( encPrevNextCode[i]==0x07) {
          sendKey(encoderDwn[i]);
          Serial.print("DOWN ");
          Serial.println(encStore[i],HEX);
      }
    }
  }

  // Funky encoder is using ESP32Encoder routine
  int32_t cntr = funkyEncoder.getCount();
  if (cntr!=funkyEncoderPrevCenter) {
    if (!funkyEncoderHoldoff) {
      if (cntr>funkyEncoderPrevCenter) { sendKey(funkyEncoderUpp); }
      if (cntr<funkyEncoderPrevCenter) { sendKey(funkyEncoderDwn); }
      funkyEncoderHoldoff = now;
      // SAFEGUARD WRAP AROUND OF millis() (WHICH IS TO 0) SINCE funkyEncoderHoldoff==0 HAS A SPECIAL MEANING ABOVE
      if (funkyEncoderHoldoff==0) funkyEncoderHoldoff = 1;
    }
    else if (now - funkyEncoderHoldoff > FUNKY_HOLDOFF_TIME) {
      funkyEncoderPrevCenter = funkyEncoder.getCount();
      funkyEncoderHoldoff = 0;
    }
  }

  if (customKeypad.getKeys()) {
    // Loops through the funky switch directions and checks to see if the push is being triggered
    // along with another direction. If it is, we will flag it for skipping the normal press
    // routine. We also press the _intended_ direction only.
    for (int i = 0; i < FUNKY_DIR_COUNT; i++) {
      // if center not handling press, try directions.
      if(customKeypad.findInList((char) funkyCenterCode) != -1 && customKeypad.findInList((char) funkyDirectionCodes[i]) != -1) {
        if(now - funkyDirectionLastHoldoff[i] > FUNKY_DIR_HOLDOFF_TIME) {
          pressKey((char) funkyDirectionCodes[i]);
          funkyDirectionLastHoldoff[i] = now; // last press send
        }
        if (funkyDirectionLastHoldoff[i]==0) funkyDirectionLastHoldoff[i] = 1;   // SAFEGUARD WRAP AROUND OF millis() (WHICH IS TO 0) SINCE holdoff[i]==0 HAS A SPECIAL MEANING ABOVE
        handlingFunkySwitch = true;
      }
    }

    // Scan the whole key list and find changed keys.
    for (int i=0; i < LIST_MAX; i++) {
      if (customKeypad.key[i].stateChanged) { // Only find keys that have changed state.
        switch (customKeypad.key[i].kstate) { // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
          case PRESSED:
            if(handlingFunkySwitch == false){
              pressKey(customKeypad.key[i].kchar);
            }
          break;
          case HOLD:
            if(handlingFunkySwitch == false){
              holdKey(customKeypad.key[i].kchar);
            }
          break;
          case RELEASED:
          case IDLE:
            releaseKey(customKeypad.key[i].kchar);
          break;
        }

      }
    }
  }

  // Send battery level to host periodically
  if(now - prevBatteryUpdate > batteryUpdateInterval) {
    prevBatteryUpdate = now;
    int batteryLevel = getBatteryPercent();
    float batteryVoltage = getBatteryVoltage();

    Serial.print("battery: \t");
    Serial.println((String)batteryLevel);
    
    if(bleGamepad.isConnected()) {
      bleGamepad.setBatteryLevel(batteryLevel);
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println((String)batteryVoltage + "V");
    display.println((String)batteryLevel + "%");
    display.display();
  }

  // No longer looking for strange funky switch logic
  handlingFunkySwitch = false;
}

void sendKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key); // CONVERT TO THE BINARY MAPPING GAMEPAD KEYS USE
    Serial.print("pulse\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.press(gamepadbutton);
      delay(100);
      bleGamepad.release(gamepadbutton);
    }
}

void pressKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);
    Serial.print("press\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.press(gamepadbutton);
    }
}

// Something clever should happen here eventually
void holdKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);
    Serial.print("hold\t");
    Serial.println(key);
    // if(bleGamepad.isConnected()) {
    //   bleGamepad.press(gamepadbutton);
    // }
}

void releaseKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);
    Serial.print("release\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.release(gamepadbutton);
    }
}

// https://www.best-microcontroller-projects.com/rotary-encoder.html
// A vald CW or  CCW move returns 1, invalid returns 0.
int8_t readRotary(uint8_t DATA_PIN, uint8_t CLK_PIN, uint8_t i) {
  static int8_t rotEncTable[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  encPrevNextCode[i] <<= 2;
  if (digitalRead(DATA_PIN)) encPrevNextCode[i] |= 0x02;
  if (digitalRead(CLK_PIN)) encPrevNextCode[i] |= 0x01;
  encPrevNextCode[i] &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rotEncTable[encPrevNextCode[i]] ) {
      encStore[i] <<= 4;
      encStore[i] |= encPrevNextCode[i];
      //if (encStore[i]==0xd42b) return 1;
      //if (encStore[i]==0xe817) return -1;
      if ((encStore[i]&0xff)==0x2b) return -1;
      if ((encStore[i]&0xff)==0x17) return 1;
   }
   return 0;
}

int getBatteryPercent() {
  const int maxDisplayed = 100; // 100% (higher values are constrained)
  const int minDisplayed = 0; // 0% or flat (we'll probably never get there)
  const float maxBatteryVoltage = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2
  const float minBatteryVoltage = 3.3; // cutoff voltage (3.2, 3.3 to be safer)
  float voltageLevel = getBatteryVoltage();
  float usablePercent = ((voltageLevel - minBatteryVoltage) / (maxBatteryVoltage - minBatteryVoltage)) * ((maxDisplayed - minDisplayed) + minDisplayed);
  return constrain((int)usablePercent, minDisplayed, maxDisplayed);
}

// If you read voltage values higher than 4.2, and are seeing a difference
// between the value on your multimeter, adjust the vRef. Something between
// 1.0 and 1.1 should be where you're aiming for.
// You can go as deep as you want here:
// https://esp32.com/viewtopic.php?f=19&t=2881&start=30;
// the simple fix below works for me though.
float getBatteryVoltage() {
  const float vRef = 1.048; // should be 1.1V but may need to be calibrated
  const float maxAnalogVal = 4095.0; // defines the range of the ADC calculation
  return (analogRead(35) / maxAnalogVal) * 2 * vRef * 3.3; // calculate voltage level
}
