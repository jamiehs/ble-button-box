#include <ESP32Encoder.h>     // https://github.com/madhephaestus/ESP32Encoder/
#include <Keypad.h>           // https://github.com/Chris--A/Keypad
#include <BleGamepad.h>       // https://github.com/lemmingDev/ESP32-BLE-Gamepad

int getBatteryLevel() {
  const int maxDisplayed = 100;
  const int minDisplayed = 0;
  const float maxAnalogVal = 4095.0;
  const float maxBatteryVoltage = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2
  const float minBatteryVoltage = 3.2; // Huzzah cutoff voltage
  float voltageLevel = (analogRead(35) / maxAnalogVal) * 2 * 1.1 * 3.3; // calculate voltage level
  float usablePercent = ((voltageLevel - minBatteryVoltage) / (maxBatteryVoltage - minBatteryVoltage)) * ((maxDisplayed - minDisplayed) + minDisplayed);
  // Serial.print("raw: ");
  // Serial.print((String)analogRead(35));
  // Serial.print("\t");
  // Serial.print("voltage: ");
  // Serial.print((String)voltageLevel);
  // Serial.print("\t");
  // Serial.print("usable: ");
  // Serial.print((String)usablePercent);
  // Serial.print("\t");
  // Serial.print("percent: \t");
  // Serial.println((String)constrain((int)usablePercent, minDisplayed, maxDisplayed));
  return constrain((int)usablePercent, minDisplayed, maxDisplayed);
}

// Set the battery level first or Windows will just read the default of 100
BleGamepad bleGamepad("BLE Sim Buttons", "Arduino", getBatteryLevel());

long batteryUpdateInterval = 300000; // every 5 minutes
long prevBatteryUpdate = 295000; // 5 seconds from goal

////////////////////// BUTTON MATRIX //////////////////////
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

//////////// ROTARY ENCODERS ////////////
#define MAXENC 2
uint8_t uppPin[MAXENC] = {16, 18};
uint8_t dwnPin[MAXENC] = {19, 5};
uint8_t encCount[MAXENC] = {0, 0};
uint8_t encValue[MAXENC] = {0, 0};
uint8_t encPrevNextCode[MAXENC] = {0, 0};
uint16_t encStore[MAXENC]= {0, 0};
static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

uint8_t encoderUpp[MAXENC] = {23,25};
uint8_t encoderDwn[MAXENC] = {24,26};

ESP32Encoder funkyEncoder;
unsigned long funkyEncoderHoldoff = 0;
int32_t funkyEncoderPrevCenter = 0;
uint8_t funkyEncoderUppPin = 21; // pin number
uint8_t funkyEncoderDwnPin = 17; // pin number
uint8_t funkyEncoderUpp = 22; // button number
uint8_t funkyEncoderDwn = 21; // button number

#define FUNKY_DIR_COUNT 4
char funkyPress = 0;
char funkyDirections[FUNKY_DIR_COUNT] = {13, 9, 5, 17};
bool handlingFunkySwitch = false;
unsigned long funkyHoldoff[4] = {0,0,0,0};
#define FUNKY_HOLDOFFTIME 300 // 0.3sec

#define HOLDOFFTIME 30   // TO PREVENT MULTIPLE ROTATE "CLICKS"

void setup() {
  Serial.begin(115200);

  // setup encoder logic for "encoder table" encoders
  for (uint8_t i=0; i<MAXENC; i++) {
    pinMode(uppPin[i], INPUT);
    pinMode(uppPin[i], INPUT_PULLUP);
    pinMode(dwnPin[i], INPUT);
    pinMode(dwnPin[i], INPUT_PULLUP);
  }

  // different encoder logic for funky encoder
  funkyEncoder.attachHalfQuad(funkyEncoderUppPin, funkyEncoderDwnPin);
  funkyEncoder.clearCount();

  // start button & BLE routines/settings
  customKeypad.setHoldTime(7000); // 7 seconds is considered holding
  bleGamepad.begin();
  Serial.println("Booted!");
}


void loop() {

  unsigned long now = millis();

  // loop through "encoder table" encoders
  for (uint8_t i=0; i<MAXENC; i++) {
    if( encValue[i]=read_rotary(uppPin[i], dwnPin[i], i) ) {
      encCount[i] +=encValue[i];
      Serial.print(encCount[i]);Serial.print(" ");

      if ( encPrevNextCode[i]==0x0b) {
          sendKey(encoderUpp[i]);
          Serial.print("eleven ");
          Serial.println(encStore[i],HEX);
      }

      if ( encPrevNextCode[i]==0x07) {
          sendKey(encoderDwn[i]);
          Serial.print("seven ");
          Serial.println(encStore[i],HEX);
      }
    }
  }

  // funky encoder is using ESP32Encoder routine
  int32_t cntr = funkyEncoder.getCount();
  if (cntr!=funkyEncoderPrevCenter) {
    if (!funkyEncoderHoldoff) {
      if (cntr>funkyEncoderPrevCenter) { sendKey(funkyEncoderUpp); }
      if (cntr<funkyEncoderPrevCenter) { sendKey(funkyEncoderDwn); }
      funkyEncoderHoldoff = now;
      if (funkyEncoderHoldoff==0) funkyEncoderHoldoff = 1;  // SAFEGUARD WRAP AROUND OF millis() (WHICH IS TO 0) SINCE funkyEncoderHoldoff==0 HAS A SPECIAL MEANING ABOVE
    }
    else if (now - funkyEncoderHoldoff > HOLDOFFTIME) {
      funkyEncoderPrevCenter = funkyEncoder.getCount();
      funkyEncoderHoldoff = 0;
    }
  }


  if (customKeypad.getKeys()) {
    /*
    Loops through the funky switch directions and checks to see if the push is being triggered
    along with another direction. If it is, we will flag it for skipping the normal press
    routine. We also press the _intended_ direction only.
    */
    for (int i = 0; i < FUNKY_DIR_COUNT; i++) {
      // if center not handling press, try directions.
      if(customKeypad.findInList((char) funkyPress) != -1 && customKeypad.findInList((char) funkyDirections[i]) != -1) {
        if(now - funkyHoldoff[i] > FUNKY_HOLDOFFTIME) {
          pressKey((char) funkyDirections[i]);
          funkyHoldoff[i] = now; // last press send
        }
        if (funkyHoldoff[i]==0) funkyHoldoff[i] = 1;   // SAFEGUARD WRAP AROUND OF millis() (WHICH IS TO 0) SINCE holdoff[i]==0 HAS A SPECIAL MEANING ABOVE
        handlingFunkySwitch = true;
      }
    }

    for (int i=0; i<LIST_MAX; i++) {   // Scan the whole key list.
      if (customKeypad.key[i].stateChanged) {   // Only find keys that have changed state.
        switch (customKeypad.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
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

  // send battery level periodically
  if(now - prevBatteryUpdate > batteryUpdateInterval) {
    prevBatteryUpdate = now;
    int batteryLevel = getBatteryLevel();

    Serial.print("battery: \t");
    Serial.println((String)batteryLevel);
    
    bleGamepad.setBatteryLevel(batteryLevel);
  }

  handlingFunkySwitch = false;
}

void sendKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);      // CONVERT TO THE BINARY MAPPING GAMEPAD KEYS USE
    Serial.print("pulse\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.press(gamepadbutton);
      delay(100);
      bleGamepad.release(gamepadbutton);
    }
}

void pressKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);      // CONVERT TO THE BINARY MAPPING GAMEPAD KEYS USE
    Serial.print("press\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.press(gamepadbutton);
    }
}

void holdKey(uint8_t key) {
    // uint32_t gamepadbutton = pow(2,key);      // CONVERT TO THE BINARY MAPPING GAMEPAD KEYS USE
    // Serial.print("hold\t");
    // Serial.println(key);
    // if(bleGamepad.isConnected()) {
    //   bleGamepad.press(gamepadbutton);
    // }
}

void releaseKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);      // CONVERT TO THE BINARY MAPPING GAMEPAD KEYS USE
    Serial.print("release\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.release(gamepadbutton);
    }
}

// https://www.best-microcontroller-projects.com/rotary-encoder.html
// A vald CW or  CCW move returns 1, invalid returns 0.
int8_t read_rotary(uint8_t DATA_PIN, uint8_t CLK_PIN, uint8_t i) {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  encPrevNextCode[i] <<= 2;
  if (digitalRead(DATA_PIN)) encPrevNextCode[i] |= 0x02;
  if (digitalRead(CLK_PIN)) encPrevNextCode[i] |= 0x01;
  encPrevNextCode[i] &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[encPrevNextCode[i]] ) {
      encStore[i] <<= 4;
      encStore[i] |= encPrevNextCode[i];
      //if (encStore[i]==0xd42b) return 1;
      //if (encStore[i]==0xe817) return -1;
      if ((encStore[i]&0xff)==0x2b) return -1;
      if ((encStore[i]&0xff)==0x17) return 1;
   }
   return 0;
}
