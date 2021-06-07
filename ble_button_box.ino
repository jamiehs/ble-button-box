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
#define MAXENC 3
uint8_t uppPin[MAXENC] = {21, 16, 18};
uint8_t dwnPin[MAXENC] = {17, 19, 5};

uint8_t encoderUpp[MAXENC] = {21,23,25};
uint8_t encoderDwn[MAXENC] = {22,24,26};

// ESP32Encoder encoder[MAXENC];
// unsigned long holdoff[MAXENC] = {0,0,0};
// int32_t prevenccntr[MAXENC] = {0,0,0};

#define FUNKY_DIR_COUNT 4
char funkyPress = 0;
char funkyDirections[FUNKY_DIR_COUNT] = {13, 9, 5, 17};
bool handlingFunkySwitch = false;
unsigned long funkyHoldoff[4] = {0,0,0,0};
#define FUNKY_HOLDOFFTIME 300 // 0.3sec

#define HOLDOFFTIME 30   // TO PREVENT MULTIPLE ROTATE "CLICKS"

#define CLK  18
#define DATA 5

void setup() {
  Serial.begin(115200);

  pinMode(CLK, INPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DATA, INPUT);
  pinMode(DATA, INPUT_PULLUP);

  // for (uint8_t i=0; i<MAXENC; i++) {
  //   encoder[i].clearCount();
  //   encoder[i].attachHalfQuad(dwnPin[i], uppPin[i]);
  // }

  customKeypad.setHoldTime(7000); // 7 seconds is considered holding
  bleGamepad.begin();
  Serial.println("Booted!");
}


static uint8_t prevNextCode = 0;
static uint16_t store=0;

// A vald CW or  CCW move returns 1, invalid returns 0.
int8_t read_rotary() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(DATA)) prevNextCode |= 0x02;
  if (digitalRead(CLK)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   return 0;
}

void loop() {

  static int8_t c,val;

  if( val=read_rotary() ) {
    c +=val;
    Serial.print(c);Serial.print(" ");

    if ( prevNextCode==0x0b) {
        sendKey(25);
        Serial.print("eleven ");
        Serial.println(store,HEX);
    }

    if ( prevNextCode==0x07) {
        sendKey(26);
        Serial.print("seven ");
        Serial.println(store,HEX);
    }
  }


  unsigned long now = millis();

  // for (uint8_t i=0; i<MAXENC; i++) {
  //   int32_t cntr = encoder[i].getCount();
  //   if (cntr!=prevenccntr[i]) {
  //     if (!holdoff[i]) {
  //       if (cntr>prevenccntr[i]) { sendKey(encoderUpp[i]); }
  //       if (cntr<prevenccntr[i]) { sendKey(encoderDwn[i]); }
  //       holdoff[i] = now;
  //       if (holdoff[i]==0) holdoff[i] = 1;  // SAFEGUARD WRAP AROUND OF millis() (WHICH IS TO 0) SINCE holdoff[i]==0 HAS A SPECIAL MEANING ABOVE
  //     }
  //     else if (now - holdoff[i] > HOLDOFFTIME) {
  //       prevenccntr[i] = encoder[i].getCount();
  //       holdoff[i] = 0;
  //     }
  //   }
  // }


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

  // delay(10);
}

void sendKey(uint8_t key) {
    uint32_t gamepadbutton = pow(2,key);      // CONVERT TO THE BINARY MAPPING GAMEPAD KEYS USE
    Serial.print("pulse\t");
    Serial.println(key);
    if(bleGamepad.isConnected()) {
      bleGamepad.press(gamepadbutton);
      delay(75);
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
