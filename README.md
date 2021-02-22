# Bluetooth Button Box

## Parts List

* Micro controller [Adafruit HUZZAH32 – ESP32 Feather Board](https://www.adafruit.com/product/3405) (other ESP32 boards should work fine too)
* Ball spring detent encoders: [PEC11H-4020F-S0016](https://www.mouser.com/ProductDetail/652-PEC11H4020FS0016/) (most 6mm encoders will fit fine)
* Multi direction switch [RKJXT1F42001](https://www.mouser.com/ProductDetail/688-RKJXT1F42001/)  (funky switch)
* 12MM Tactile Switch 260GF 7.3MM [653-B3F-4155](https://www.mouser.com/ProductDetail/653-B3F-4155/) (I can't find the button caps)
* 3D printed parts in the [STL directory](./stl).


## Setup

This setup is for using the code with the Huzzah 32 from Adafruit. It has a built in LiPo charger and works really well.


### Prerequisites

* Install the Arduino IDE
* Install the board definitions for the ESP32
* Install the needed libraries (listed below)

```cpp
#include <ESP32Encoder.h>     // https://github.com/madhephaestus/ESP32Encoder/
#include <Keypad.h>           // https://github.com/Chris--A/Keypad
#include <BleGamepad.h>       // https://github.com/lemmingDev/ESP32-BLE-Gamepad
```


### Board Settings

These are my VS Code settings for interfacing with the board (your port will be different):
```json
{
    "port": "COM6",
    "board": "esp32:esp32:featheresp32",
    "configuration": "FlashFreq=80,UploadSpeed=921600,DebugLevel=none,PartitionScheme=default",
    "sketch": "ble_button_box.ino",
    "output": "../build"
}
```

### Button Matrix

I'm not using any diodes, but you can implement them if you need to. I am using a 5x4 matrix for a total of 20 buttons:

```cpp
#define ROWS 5
#define COLS 4
uint8_t rowPins[ROWS] = {15, 32, 14, 22, 23};
uint8_t colPins[COLS] = {4, 12, 27, 33};
```

* 5 buttons for the funky switch (4 directions and push)
* 2 buttons for shifters (microswitches)
* 2 buttons for the encoders (push)
* 11 face buttons (push)

You need to define the HID button numbers you want the matrix buttons to identify as. In my example I am just counting up from `1..20` (you can tell that I counted incorrectly and skipped `4` while starting from `0`; no matter).

```cpp
byte keymap[ROWS][COLS] = {
  { 0, 1, 2, 3},
  { 5, 6, 7, 8},
  { 9,10,11,12},
  {13,14,15,16},
  {17,18,19,20}
};
```

### Funky Switch Issues

A note about the funky switch; you may notice that I'm doing some strange stuff here:

```cpp
#define FUNKY_DIR_COUNT 4
char funkyPress = 0;
char funkyDirections[FUNKY_DIR_COUNT] = {13, 9, 5, 17};
bool handlingFunkySwitch = false;
```

The Alps funky switch presses the center push for every direction press; it's hardwired that way. In order to code around this, I am checking to see if one of the four directions is pressed (the funky switch ended up being the first column [`0`, `5`, `9`, `13`, `17`] in my wiring). The array defines the `up`, `left`, `right`, `down` directions and the code iterates through it all later on.

If one of the cardinal directions is being pressed, we send _only_ that button code and stop processing the matrix.

```cpp
/*
Loops through the funky switch directions and checks to see if the push is being triggered
along with another direction. If it is, we will flag it for skipping the normal press
routine. We also press the _intended_ direction only.
*/
for (int i = 0; i < FUNKY_DIR_COUNT; i++) {
    if(customKeypad.findInList((char) funkyPress) != -1 && customKeypad.findInList((char) funkyDirections[i]) != -1) {
    pressKey((char) funkyDirections[i]);
    handlingFunkySwitch = true;
    }
}
```

I'm sure there's a more elegant way to handle this, but I'm just not seeing it.


### Encoders

You will need to assign pins and "buttons" for your encoders too!

```cpp
//////////// ROTARY ENCODERS ////////////
#define MAXENC 3
uint8_t uppPin[MAXENC] = {21, 16, 18};
uint8_t dwnPin[MAXENC] = {17, 19, 5};

uint8_t encoderUpp[MAXENC] = {21,23,25};
uint8_t encoderDwn[MAXENC] = {22,24,26};
...
```

Here the encoders are listed as columns. In this example, `21`, `17` are one encoder, and `16`, `19` are another, etc.

Likewise, when the encoder attached to `21`, `17` is rolled up or down, it'll press the gamepad buttons: `21`, `22`. These can be assigned to whatever you need them to be.


### Battery Level

All the support is there, but this doesn't seem to work the way it should at the moment. It's likely something that I'm overlooking with the ADC conversion and maybe a Windows 10 bug, but alas it doesn't work for me.