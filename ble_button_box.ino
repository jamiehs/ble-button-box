#include <ESP32Encoder.h>     // https://github.com/madhephaestus/ESP32Encoder/
#include <Keypad.h>           // https://github.com/Chris--A/Keypad
#include <BleGamepad.h>       // https://github.com/lemmingDev/ESP32-BLE-Gamepad

int batteryLevel = map(analogRead(35), 2297.0f, 2507.0f, 0, 100);
BleGamepad bleGamepad("BLE Sim Buttons", "Arduino", batteryLevel);

////////////////////// BUTTON MATRIX //////////////////////
#define ROWS 5
#define COLS 4
uint8_t rowPins[ROWS] = {15, 32, 14, 22, 23};
uint8_t colPins[COLS] = {4, 12, 27, 33};
byte keymap[ROWS][COLS] = {
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

uint8_t encoderUpp[MAXENC] = {21,23,25};
uint8_t encoderDwn[MAXENC] = {22,24,26};

ESP32Encoder funkyEncoder;
unsigned long funkyEncoderHoldoff = 0;
int32_t funkyEncoderPrevCenter = 0;

#define FUNKY_DIR_COUNT 4
char funkyPress = 0;
char funkyDirections[FUNKY_DIR_COUNT] = {13, 9, 5, 17};
bool handlingFunkySwitch = false;
unsigned long funkyHoldoff[4] = {0,0,0,0};
#define FUNKY_HOLDOFFTIME 300 // 0.3sec

#define HOLDOFFTIME 30   // TO PREVENT MULTIPLE ROTATE "CLICKS"

void setup() {
  Serial.begin(115200);

  pinMode(CLK, INPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DATA, INPUT);
  pinMode(DATA, INPUT_PULLUP);

  funkyEncoder[i].clearCount();
  funkyEncoder[i].attachHalfQuad(21, 17);

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

        batteryLevel = map(analogRead(35), 2297.0f, 2607.0f, 0, 100);
        Serial.println(batteryLevel);
        bleGamepad.setBatteryLevel(batteryLevel);
      }
    }
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
