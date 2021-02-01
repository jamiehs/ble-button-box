#include <ESP32Encoder.h>     // https://github.com/madhephaestus/ESP32Encoder/
#include <Keypad.h>           // https://github.com/Chris--A/Keypad
#include <BleGamepad.h>       // https://github.com/MagnusThome/ESP32-BLE-Gamepad

BleGamepad bleGamepad("BLE Sim Buttons", "Arduino", 100);

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

//////////// ROTARY ENCODERS (WITH PUSH SWITCHES) ////////////
#define MAXENC 3
uint8_t uppPin[MAXENC] = {21, 16, 18};
uint8_t dwnPin[MAXENC] = {17, 19, 5};

uint8_t pressed[5];

uint8_t encoderUpp[MAXENC] = {21,23,25};
uint8_t encoderDwn[MAXENC] = {22,24,26};

ESP32Encoder encoder[MAXENC];
unsigned long holdoff[MAXENC] = {0,0,0};
int32_t prevenccntr[MAXENC] = {0,0,0};
#define HOLDOFFTIME 50   // TO PREVENT MULTIPLE ROTATE "CLICKS" WITH CHEAP ENCODERS WHEN ONLY ONE CLICK IS INTENDED

bool handlingFunkySwitch = false;

void setup() {
  Serial.begin(115200);

  for (uint8_t i=0; i<MAXENC; i++) {
    encoder[i].clearCount();
    encoder[i].attachHalfQuad(dwnPin[i], uppPin[i]);
  }
  customKeypad.addEventListener(keypadEvent);
  customKeypad.setHoldTime(7000); // 5 seconds is considered holding
  bleGamepad.begin();
  Serial.println("Booted!");
}

void loop() {

  unsigned long now = millis();

  for (uint8_t i=0; i<MAXENC; i++) {
    int32_t cntr = encoder[i].getCount();
    if (cntr!=prevenccntr[i]) {
      if (!holdoff[i]) {
        if (cntr>prevenccntr[i]) { sendKey(encoderUpp[i]); }
        if (cntr<prevenccntr[i]) { sendKey(encoderDwn[i]); }
        holdoff[i] = now;
        if (holdoff[i]==0) holdoff[i] = 1;  // SAFEGUARD WRAP AROUND OF millis() (WHICH IS TO 0) SINCE holdoff[i]==0 HAS A SPECIAL MEANING ABOVE
      }
      else if (now-holdoff[i] > HOLDOFFTIME) {
        prevenccntr[i] = encoder[i].getCount();
        holdoff[i] = 0;
      }
    }
  }


  if (customKeypad.getKeys()) {

    if(customKeypad.findInList((char) 0) != -1 && customKeypad.findInList((char) 13) != -1) {
      pressKey((char) 13);
      handlingFunkySwitch = true;
    }

    if(customKeypad.findInList((char) 0) != -1 && customKeypad.findInList((char) 9) != -1) {
      pressKey((char) 9);
      handlingFunkySwitch = true;
    }

    if(customKeypad.findInList((char) 0) != -1 && customKeypad.findInList((char) 5) != -1) {
      pressKey((char) 5);
      handlingFunkySwitch = true;
    }

    if(customKeypad.findInList((char) 0) != -1 && customKeypad.findInList((char) 17) != -1) {
      pressKey((char) 17);
      handlingFunkySwitch = true;
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
        Serial.println((int) map(analogRead(35), 0.0f, 4095.0f, 0, 100));
        bleGamepad.setBatteryLevel((int) map(analogRead(35), 0.0f, 4095.0f, 0, 100));
      }
    }
  }

  handlingFunkySwitch = false;

  delay(10);
}




////////////////////////////////////////////////////////////////////////////////////////

void keypadEvent(KeypadEvent key){
  // uint8_t keystate = customKeypad.getState();
  // if (keystate==PRESSED)  { pressKey(key); }
  // if (keystate==RELEASED) { releaseKey(key); }
}


////////////////////////////////////////////////////////////////////////////////////////

void sendKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);      // CONVERT TO THE BINARY MAPPING GAMEPAD KEYS USE
    Serial.print("pulse\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.press(gamepadbutton);
      delay(150);
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



////////////////////////////////////////////////////////////////////////////////////////
