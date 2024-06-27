#include <Arduino.h>
#include <EEPROM.h>
#include <LedControl.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>

#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

int DFplayerIN = 2;  // 'D6'
int DFPlayerOUT = 3; // 'D7'
// Matrix
byte devices = 4;
LedControl lc = LedControl(4, 5, 6, devices);


// RTC
#define DS3231_I2C_ADDRESS 0x68                                // I2C Address of DS3231 RTC
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year; // global variables

// Timing
unsigned long presentTime;
unsigned long displayTime; // Drawing

// IO
#define BTN1 7
#define BTN2 8
#define BTN3 9
#define BTN4 10
#define BTN5 11
#define BTN6 12
#define BTN7 13

bool lastInput1;
bool lastInput2;
bool presentInput1;
bool presentInput2;
bool alarmBTNState;

// SystemState
byte systemState;
// chars
byte znaky[95][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // U+0020 (space)
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, // U+0021 (!)
    {0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // U+0022 (")
    {0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00}, // U+0023 (#)
    {0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00}, // U+0024 ($)
    {0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00}, // U+0025 (%)
    {0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00}, // U+0026 (&)
    {0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}, // U+0027 (')
    {0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00}, // U+0028 (()
    {0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00}, // U+0029 ())
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00}, // U+002A (*)
    {0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00}, // U+002B (+)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06}, // U+002C (,)
    {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00}, // U+002D (-)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00}, // U+002E (.)
    {0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00}, // U+002F (/)
    {0x3e, 0x63, 0x63, 0x63, 0x63, 0x63, 0x3e, 0x00}, // U+0030 (0)
    {0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00}, // U+0031 (1)
    {0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00}, // U+0032 (2)
    {0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00}, // U+0033 (3)
    {0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00}, // U+0034 (4)
    {0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00}, // U+0035 (5)
    {0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00}, // U+0036 (6)
    {0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00}, // U+0037 (7)
    {0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00}, // U+0038 (8)
    {0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00}, // U+0039 (9)
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00}, // U+003A (:)
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x06}, // U+003B (//)
    {0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00}, // U+003C (<)
    {0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00}, // U+003D (=)
    {0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00}, // U+003E (>)
    {0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00}, // U+003F (?)
    {0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00}, // U+0040 (@)
    {0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00}, // U+0041 (A)
    {0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00}, // U+0042 (B)
    {0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00}, // U+0043 (C)
    {0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00}, // U+0044 (D)
    {0x7e, 0x06, 0x06, 0x1e, 0x06, 0x06, 0x7e, 0x00}, // U+0045 (E)
    {0x7e, 0x06, 0x06, 0x1e, 0x06, 0x06, 0x06, 0x00}, // U+0046 (F)
    {0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00}, // U+0047 (G)
    {0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00}, // U+0048 (H)
    {0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, // U+0049 (I)
    {0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00}, // U+004A (J)
    {0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00}, // U+004B (K)
    {0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00}, // U+004C (L)
    {0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00}, // U+004D (M)
    {0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00}, // U+004E (N)
    {0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00}, // U+004F (O)
    {0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00}, // U+0050 (P)
    {0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00}, // U+0051 (Q)
    {0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00}, // U+0052 (R)
    {0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00}, // U+0053 (S)
    {0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, // U+0054 (T)
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00}, // U+0055 (U)
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00}, // U+0056 (V)
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00}, // U+0057 (W)
    {0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00}, // U+0058 (X)
    {0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00}, // U+0059 (Y)
    {0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00}, // U+005A (Z)
    {0x1E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1E, 0x00}, // U+005B ([)
    {0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00}, // U+005C (\)
    {0x1E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00}, // U+005D (])
    {0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00}, // U+005E (^)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF}, // U+005F (_)
    {0x0C, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00}, // U+0060 (`)
    {0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00}, // U+0061 (a)
    {0x07, 0x06, 0x06, 0x3E, 0x66, 0x66, 0x3B, 0x00}, // U+0062 (b)
    {0x00, 0x00, 0x1E, 0x33, 0x03, 0x33, 0x1E, 0x00}, // U+0063 (c)
    {0x38, 0x30, 0x30, 0x3e, 0x33, 0x33, 0x6E, 0x00}, // U+0064 (d)
    {0x00, 0x00, 0x1E, 0x33, 0x3f, 0x03, 0x1E, 0x00}, // U+0065 (e)
    {0x1C, 0x36, 0x06, 0x0f, 0x06, 0x06, 0x0F, 0x00}, // U+0066 (f)
    {0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x1F}, // U+0067 (g)
    {0x07, 0x06, 0x36, 0x6E, 0x66, 0x66, 0x67, 0x00}, // U+0068 (h)
    {0x0C, 0x00, 0x0E, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, // U+0069 (i)
    {0x30, 0x00, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E}, // U+006A (j)
    {0x07, 0x06, 0x66, 0x36, 0x1E, 0x36, 0x67, 0x00}, // U+006B (k)
    {0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, // U+006C (l)
    {0x00, 0x00, 0x33, 0x7F, 0x7F, 0x6B, 0x63, 0x00}, // U+006D (m)
    {0x00, 0x00, 0x1F, 0x33, 0x33, 0x33, 0x33, 0x00}, // U+006E (n)
    {0x00, 0x00, 0x1E, 0x33, 0x33, 0x33, 0x1E, 0x00}, // U+006F (o)
    {0x00, 0x00, 0x3B, 0x66, 0x66, 0x3E, 0x06, 0x0F}, // U+0070 (p)
    {0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x78}, // U+0071 (q)
    {0x00, 0x00, 0x3B, 0x6E, 0x66, 0x06, 0x0F, 0x00}, // U+0072 (r)
    {0x00, 0x00, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x00}, // U+0073 (s)
    {0x08, 0x0C, 0x3E, 0x0C, 0x0C, 0x2C, 0x18, 0x00}, // U+0074 (t)
    {0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x6E, 0x00}, // U+0075 (u)
    {0x00, 0x00, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00}, // U+0076 (v)
    {0x00, 0x00, 0x63, 0x6B, 0x7F, 0x7F, 0x36, 0x00}, // U+0077 (w)
    {0x00, 0x00, 0x63, 0x36, 0x1C, 0x36, 0x63, 0x00}, // U+0078 (x)
    {0x00, 0x00, 0x33, 0x33, 0x33, 0x3E, 0x30, 0x1F}, // U+0079 (y)
    {0x00, 0x00, 0x3F, 0x19, 0x0C, 0x26, 0x3F, 0x00}, // U+007A (z)
    {0x38, 0x0C, 0x0C, 0x07, 0x0C, 0x0C, 0x38, 0x00}, // U+007B ({)
    {0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00}, // U+007C (|)
    {0x07, 0x0C, 0x0C, 0x38, 0x0C, 0x0C, 0x07, 0x00}, // U+007D (})
    {0x6E, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // U+007E (~)
};

byte mode = 0;
byte bright = 0;

byte volume = 0;
bool showDots;

// Alarm
byte alarmHour = 00;
byte alarmMinute = 00;
bool alarmState = false;

bool displayState = false;
bool lastButtonState = HIGH;
bool newLastButtonState = HIGH;
bool currentButtonState;
bool newButtonState;
bool checkBtn;

SoftwareSerial mySoftwareSerial(DFplayerIN, DFPlayerOUT); // RX, TX
DFRobotDFPlayerMini player;

const int addrL = 0; // first LED matrix - Left robot eye
const int addrR = 3; // second LED matrix - Right robot eye
byte eye[8];

void displayEmotion(byte left[8], byte right[8])
{
  lc.clearDisplay(addrL);
  lc.clearDisplay(addrR);
  for (int row = 0; row < 8; row++)
  {
    lc.setRow(addrL, row, left[row]);
    lc.setRow(addrR, row, right[row]);
  }
}
void drawMouth(byte left[8], byte right[8])
{
  for (int row = 0; row < 8; row++)
  {
    lc.setRow(2, row, left[row]);
    lc.setRow(1, row, right[row]);
  }
}
void neutral()
{
  byte left_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01111110,
      0b01100110,
      0b01100110,
      0b01111110,
      0b00111100,
      0b00000000};
  byte right_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01111110,
      0b01100110,
      0b01100110,
      0b01111110,
      0b00111100,
      0b00000000};
  displayEmotion(left_Eye, right_Eye);
}
void anger_2() {
  byte right_Eye[8] = {
    0b00000110,
  0b00001010,
  0b00010010,
  0b00100010,
  0b01110010,
  0b01110010,
  0b01111110,
  0b00000000
  };
  byte left_Eye[8] = {
    0b00000000,
  0b01110000,
  0b01001000,
  0b01000100,
  0b01000110,
  0b01000111,
  0b01111111,
  0b00000000

  };
  
  displayEmotion(left_Eye, right_Eye);
}
void heart(){
  byte left_eye[8] = {
    
  0b00000000,
  0b01101100,
  0b11111110,
  0b01111100,
  0b00111000,
  0b00010000,
  0b00000000,
  0b00000000
  };

  byte right_eye[8] = {
    
  0b00000000,
  0b01101100,
  0b11111110,
  0b01111100,
  0b00111000,
  0b00010000,
  0b00000000,
  0b00000000
  };

  displayEmotion(left_eye, right_eye);
}
void look_right()
{
  byte left_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01111110,
      0b01111110,
      0b01110010,
      0b01110010,
      0b00111100,
      0b00000000};
  byte right_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01111110,
      0b01111110,
      0b01110010,
      0b01110010,
      0b00111100,
      0b00000000};

  displayEmotion(left_Eye, right_Eye);
}

void look_left()
{
  byte left_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01111110,
      0b01111110,
      0b01001110,
      0b01001110,
      0b00111100,
      0b00000000};
  byte right_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01111110,
      0b01111110,
      0b01001110,
      0b01001110,
      0b00111100,
      0b00000000};

  displayEmotion(left_Eye, right_Eye);
}

void look_up_right()
{
  byte left_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01110010,
      0b01110010,
      0b01111110,
      0b01111110,
      0b00111100,
      0b00000000};
  byte right_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01110010,
      0b01110010,
      0b01111110,
      0b01111110,
      0b00111100,
      0b00000000};

  displayEmotion(left_Eye, right_Eye);
}

void look_up_left()
{
  byte left_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01001110,
      0b01001110,
      0b01111110,
      0b01111110,
      0b00111100,
      0b00000000};
  byte right_Eye[8] = {
      0b00000000,
      0b00111100,
      0b01001110,
      0b01001110,
      0b01111110,
      0b01111110,
      0b00111100,
      0b00000000};

  displayEmotion(left_Eye, right_Eye);
}

void anger()
{
  byte right_Eye[8] = {
      0b00000000,
      0b01100000,
      0b01010000,
      0b01001000,
      0b01110100,
      0b01110010,
      0b01111110,
      0b00000000};
  byte left_Eye[8] = {
      0b00000000,
      0b00000110,
      0b00001010,
      0b00010010,
      0b00101110,
      0b01001110,
      0b01111110,
      0b00000000};

  displayEmotion(left_Eye, right_Eye);
}

void blink_1()
{
  byte blink_eye[8] = {
      0b00000000,
      0b00000000,
      0b01111110,
      0b01100110,
      0b01100110,
      0b01111110,
      0b00111100,
      0b00000000};
  displayEmotion(blink_eye, blink_eye);
}
void blink_2()
{
  byte blink_eye[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b01111110,
      0b01100110,
      0b01111110,
      0b00111100,
      0b00000000};
  displayEmotion(blink_eye, blink_eye);
}

void blink_3()
{
  byte blink_eye[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b01111110,
      0b01111110,
      0b00111100,
      0b00000000};
  displayEmotion(blink_eye, blink_eye);
}
void sleep()
{
  byte sleep_Eye[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b01111110,
      0b00111100,
      0b00000000};
  displayEmotion(sleep_Eye, sleep_Eye);
}

void mouth_smile()
{
  byte left_mouth[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00010000,
      0b00001111,
      0b00000000};
  byte right_mouth[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00001000,
      0b11110000,
      0b00000000};

  drawMouth(left_mouth, right_mouth);
}

void mouth_anger()
{
  byte left_mouth[8] = {
      0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00011111,
  0b00100000,
  0b00000000};
  byte right_mouth[8] = {
     0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111000,
  0b00000100,
  0b00000000};

  drawMouth(left_mouth, right_mouth);
}
void mouth_closing()
{
  byte left_mouth[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00011111,
      0b00001111,
      0b00000000};
  byte right_mouth[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b11111000,
      0b11110000,
      0b00000000};
  drawMouth(left_mouth, right_mouth);
}
void mouth_open_full()
{
  byte left_mouth[8] = {

      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00001111,
      0b00010000,
      0b00001111,
      0b00000000};
  byte right_mouth[8] = {
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b11110000,
      0b00001000,
      0b11110000,
      0b00000000};

  drawMouth(left_mouth, right_mouth);
}

void animateEyes()
{
  neutral();
  delay(700);
  look_right();
  delay(1000);
  look_up_left();
  delay(800);
  look_left();
  delay(950);
  neutral();
  delay(700);
  look_up_right();
  delay(1000);
  neutral();
  blink_1();
  delay(250);
  blink_2();
  delay(250);
  blink_3();
  delay(250);
  sleep();
  delay(350);
  
  heart();
  delay(1000);
  anger();
  mouth_anger();
  delay(1000);
}

void animateMouth()
{
  mouth_smile();
  delay(500);
  mouth_closing();
  delay(350);
  mouth_open_full();
  delay(350);
  mouth_closing();
  delay(350);
  mouth_smile();
}

void setup()
{
  systemState = 0;
  // Communication

  Wire.begin();       // Start the I2C interface
  Serial.begin(9600); // Start the serial interface

  mySoftwareSerial.begin(9600);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!player.begin(mySoftwareSerial))
  { // Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true)
    {
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));


  // IO
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);
  pinMode(BTN5, INPUT_PULLUP);
  pinMode(BTN6, INPUT_PULLUP);
  pinMode(BTN7, INPUT_PULLUP);

  bright = EEPROM.read(0);
  alarmHour = EEPROM.read(1);
  alarmMinute = EEPROM.read(2);
  volume = EEPROM.read(3);
  delay(10);

  // Set All Displays

  for (byte address = 0; address < devices; address++)
  {
    lc.shutdown(address, false);
    lc.setIntensity(address, bright);
    lc.clearDisplay(address);
  }
}
// conversion Dec to BCD
byte decToBcd(byte val)
{
  return ((val / 10 * 16) + (val % 10));
}

// conversion BCD to Dec
byte bcdToDec(byte val)
{
  return ((val / 16 * 10) + (val % 16));
}

void GetRtc()
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // Set the register pointer to (0x00)
  Wire.endTransmission();

  Wire.requestFrom(DS3231_I2C_ADDRESS, 7); // Request 7 bytes of data

  // A few of these need masks because certain bits are control bits
  second = bcdToDec(Wire.read() & 0x7f);
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0x3f); // Need to change this if 12 hour am/pm
  dayOfWeek = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read());
}
byte ByteRevers(byte in)
{
  // font turning
  byte out;
  out = 0;
  if (in & 0x01)
    out |= 0x80;
  if (in & 0x02)
    out |= 0x40;
  if (in & 0x04)
    out |= 0x20;
  if (in & 0x08)
    out |= 0x10;
  if (in & 0x10)
    out |= 0x08;
  if (in & 0x20)
    out |= 0x04;
  if (in & 0x40)
    out |= 0x02;
  if (in & 0x80)
    out |= 0x01;
  return (out);
}
void DrawSymbol(byte adr, byte symbol, byte offset)
{
  // draw symbol
  // offset move the symbol to right side

  for (int i = 0; i <= 7; i++)
  {
    byte dataRow = znaky[symbol][i];
    dataRow = ByteRevers(dataRow) >> offset;
    lc.setRow(adr, i, dataRow);
  }
}

void WriteTime()
{
  // write time to matrix display
  if (systemState == 0 || systemState == 2 || systemState == 6)
  {
    // Directly use 'hour' for 24-hour display
    uint8_t hour24 = hour; // Already in 24-hour format

    // For drawing, separate into tens and units
    DrawSymbol(2, (hour24 % 10) + 16, 0); // Draw units digit of the hour
    DrawSymbol(3, (hour24 / 10) + 16, 0); // Draw tens digit of the hour
  }

  if (systemState == 0 || systemState == 3)
  {
    DrawSymbol(0, (minute % 10) + 16, 1);
    DrawSymbol(1, (minute / 10) + 16, 1);
  }

  // blinking dots on display
  lc.setLed(2, 1, 7, showDots); // addr, row, column
  lc.setLed(2, 2, 7, showDots);
  lc.setLed(2, 5, 7, showDots);
  lc.setLed(2, 6, 7, showDots);

  showDots = !showDots;
}

void WriteAlarmTime()
{

  // write time to matrix display
  if (systemState == 6)
  {
    // Directly use the alarm hour in 24-hour format
    uint8_t alarmHour24 = alarmHour;

    // Draw the units digit of the hour
    DrawSymbol(2, (alarmHour24 % 10) + 16, 0);

    // Draw the tens digit of the hour
    DrawSymbol(3, (alarmHour24 / 10) + 16, 0);
  }

  if (systemState == 7)
  {
    DrawSymbol(0, (alarmMinute % 10) + 16, 1);
    DrawSymbol(1, (alarmMinute / 10) + 16, 1);
  }

  // blinking dots on display
  lc.setLed(2, 1, 7, showDots); // addr, row, column
  lc.setLed(2, 2, 7, showDots);
  lc.setLed(2, 5, 7, showDots);
  lc.setLed(2, 6, 7, showDots);

  showDots = !showDots;
}

// Set RTC
void SetRtc(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set 0 to first register

  Wire.write(decToBcd(second));     // set second
  Wire.write(decToBcd(minute));     // set minutes
  Wire.write(decToBcd(hour));       // set hours
  Wire.write(decToBcd(dayOfWeek));  // set day of week (1=su, 2=mo, 3=tu)
  Wire.write(decToBcd(dayOfMonth)); // set day of month
  Wire.write(decToBcd(month));      // set month
  Wire.write(decToBcd(year));       // set year
  Wire.endTransmission();
}

// serial communication with PC
// set time via PC
void SerialComm()
{
  // first letter - type of data
  // second letter - data (space = 0, ! = 1, etc. see ASCII table)

  if (Serial.available() > 0)
  {
    byte receivedCommand;
    receivedCommand = Serial.read(); // read first letter

    if (receivedCommand < 90)
    {
      // received data is less than 90 (letter Z)
      delay(10); // wait for second letter

      byte receivedData;
      receivedData = Serial.read();
      receivedData -= 32; // shift - 32 -> 32 = space

      switch (receivedCommand)
      {
      case 65:
        // year 65 = A
        year = receivedData;
        lc.setLed(3, 7, 0, true); // show setting dot
        break;
      case 66:
        // month 66 = B
        month = receivedData;
        lc.setLed(3, 7, 1, true); // show setting dot
        break;
      case 67:
        // dayOfMonth 67 = C
        dayOfMonth = receivedData;
        lc.setLed(3, 7, 2, true); // show setting dot
        break;
      case 68:
        // hour 68 = D
        hour = receivedData;
        lc.setLed(3, 7, 3, true); // show setting dot
        break;
      case 69:
        // minute 69 = E
        minute = receivedData;
        lc.setLed(3, 7, 4, true); // show setting dot
        break;
      case 70:
        // second 70 = F
        second = receivedData;
        lc.setLed(3, 7, 5, true); // show setting dot
        break;
      case 71:
        // dayofWeek 71 = G
        dayOfWeek = receivedData;
        lc.setLed(3, 7, 6, true); // show setting dot
        break;
      }
      SetRtc(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
    }

    // spl chnout buffer do hajzlu
    Serial.flush();
    
  }
}

void alarm()
{
  if (alarmState == 1)
  {
    Serial.println("Alarm on");
    if ((alarmHour == hour && alarmMinute == minute) || (alarmHour == hour && alarmMinute == minute + 1) || (alarmHour == hour && alarmMinute == minute + 2))
    {
      player.play(1);
      player.volume(15);
    }
  }
}
bool sleepMode = false;
void loop()
{
  // if(hour <= 22)
  // {
  //   sleepMode = true; 
  // }

  // if(hour == 6 && minute == 0)
  // {
  //   sleepMode = false;
  // }
  // while (sleepMode)
  // {
  //   sleep();
  // }

  presentInput1 = digitalRead(BTN1);
  presentInput2 = digitalRead(BTN2);

  alarm();

  if(digitalRead(BTN4) == LOW)
  {
    
      DrawSymbol(3, 77 - 32, 0); // M
      DrawSymbol(2, 0, 0);       // Space
      DrawSymbol(1, 79 - 32, 0); // O
      DrawSymbol(0, 78 - 32, 0); // N
      player.randomAll();
      player.volume(volume);
      
  }
  if(digitalRead(BTN5) == LOW)
  {
    
      DrawSymbol(3, 77 - 32, 0); // M
      DrawSymbol(2, 79 - 32, 0); // O
      DrawSymbol(1, 70 - 32, 0); // F
      DrawSymbol(0, 70 - 32, 0); // F
      player.stop();
  }

  currentButtonState = digitalRead(BTN3);

  // Check if the button is pressed and it was not pressed previously
  if (currentButtonState == LOW && lastButtonState == HIGH)
  {
    // Toggle the alarm state
    alarmState = !alarmState;

    if (alarmState)
    {
      Serial.println("Alarm on");

      DrawSymbol(3, 65 - 32, 0); // A
      DrawSymbol(2, 0, 0);       // Space
      DrawSymbol(1, 79 - 32, 0); // O
      DrawSymbol(0, 78 - 32, 0); // N
      delay(1000);
    }
    else
    {
      Serial.println("Alarm off");
      DrawSymbol(3, 0, 0);       // Space
      DrawSymbol(2, 79 - 32, 0); // O
      DrawSymbol(1, 70 - 32, 0); // F
      DrawSymbol(0, 70 - 32, 0); // F
      player.stop();
      delay(1000);
    }
  }

  lastButtonState = currentButtonState;

  if(digitalRead(BTN6) == LOW)
  {
   systemState = 11;
    mouth_smile();   
    animateEyes();

  
  }
  if (digitalRead(BTN7) == LOW)
  {
    WriteTime();
    systemState = 0;
  }
  
  delay(100);

  switch (systemState)
  {
  case 0:

    presentTime = millis();

    if (presentTime - displayTime > 500)
    {
      displayTime = presentTime;
      GetRtc();
      WriteTime();
    }

    if (!presentInput1 && !presentInput2)
    {
      delay(20);
      systemState = 1;
    }

    break;
  case 1:
    if (presentInput1 && presentInput2)
    {
      systemState = 2;

      DrawSymbol(1, 0, 0);       // Space
      DrawSymbol(0, 72 - 32, 0); // H
    }
    break;

  case 2:
    // Menu 1
    //  Set Hours
    WriteTime();

    if (presentInput1 != lastInput1)
    {
      // Change detected in BTN1
      delay(20);
      if (presentInput1)
      {
        delay(20);

        systemState = 3;

        DrawSymbol(3, 77 - 32, 0); // M
        DrawSymbol(2, 0, 0);       // Space
      }
    }
    if (presentInput2 != lastInput2)
    {
      // change detected BTN2
      delay(20);
      if (presentInput2)
      {
        // rising edge detected
        // add hour
        hour++;
        if (hour > 23)
        {
          hour = 0;
        }
      }
    }
    break;

  case 3:
    // Menu 2
    // Set Minutes
    WriteTime();

    if (presentInput1 != lastInput1)
    {
      // change detected BTN1
      delay(20);
      if (presentInput1)
      {
        // rising edge detected
        systemState = 4;

        DrawSymbol(3, 66 - 32, 0);            // B
        DrawSymbol(2, 0, 0);                  // space
        DrawSymbol(0, (bright % 10) + 16, 1); // actual light intensity
        DrawSymbol(1, (bright / 10) + 16, 1); // actual light intensity
      }
    }

    if (presentInput2 != lastInput2)
    {
      delay(20);
      // change detected BTN2
      if (presentInput2)
      {
        // rising edge detected
        // add hour
        minute++;
        if (minute > 59)
        {
          minute = 0;
        }
      }
    }
    break;

  case 4:
    // menu 3
    // set brightnes
    if (presentInput1 != lastInput1)
    {
      // change detected BTN1
      delay(20);
      if (presentInput1)
      {
        // rising edge detected
        systemState = 5;
      }
    }

    if (presentInput2 != lastInput2)
    {
      // change detected BTN2
      delay(20);
      if (presentInput2)
      {
        // rising edge detected
        // add hour
        bright++;
        if (bright > 15)
        {
          bright = 0;
        }

        DrawSymbol(0, (bright % 10) + 16, 1); // actual light intensity
        DrawSymbol(1, (bright / 10) + 16, 1); // actual light intensity

        for (byte address = 0; address < devices; address++)
        {
          lc.setIntensity(address, bright); // set light intensity 0 - min, 15 - max
        }
      }
    }
    break;

  case 5:
    if (presentInput1 && presentInput2)
    {
      systemState = 6;

      DrawSymbol(1, 0, 0);       // Space
      DrawSymbol(0, 65 - 32, 0); // H
    }
    break;

  case 6:
    // Menu 1
    //  Set Hours
    WriteAlarmTime();

    if (presentInput1 != lastInput1)
    {
      // Change detected in BTN1
      delay(20);
      if (presentInput1)
      {
        delay(20);

        systemState = 7;

        DrawSymbol(3, 65 - 32, 0); // M
        DrawSymbol(2, 0, 0);       // Space
      }
    }
    if (presentInput2 != lastInput2)
    {
      // change detected BTN2
      delay(20);
      if (presentInput2)
      {
        // rising edge detected
        // add hour
        alarmHour++;
        if (alarmHour > 23)
        {
          alarmHour = 0;
        }
      }
    }
    break;

  case 7:
    // Menu 2
    // Set Minutes
    WriteAlarmTime();

    if (presentInput1 != lastInput1)
    {
      // change detected BTN1
      delay(20);
      if (presentInput1)
      {
        // rising edge detected
        systemState = 8;
      }
    }

    if (presentInput2 != lastInput2)
    {
      delay(20);
      // change detected BTN2
      if (presentInput2)
      {
        // rising edge detected
        // add hour
        alarmMinute++;
        if (alarmMinute > 59)
        {
          alarmMinute = 0;
        }
      }
    }
    break;



    case 8:

    if (presentInput1 != lastInput1)
    {
      // change detected BTN1
      delay(20);
      if (presentInput1)
      {
        // rising edge detected
        systemState = 9;
        
        
        DrawSymbol(3, 86 - 32, 0);            // V
        DrawSymbol(2, 0, 0);                  // space
        DrawSymbol(0, (volume % 10) + 16, 1); // actual light intensity
        DrawSymbol(1, (volume / 10) + 16, 1); // actual light intensity

      }
    }
    break;

  case 9:
    // menu 3
    // set brightnes
    if (presentInput1 != lastInput1)
    {
      // change detected BTN1
      delay(20);
      if (presentInput1)
      {
        // rising edge detected
        systemState = 10;

        
        DrawSymbol(3, 83 - 32, 0); // S
        DrawSymbol(2, 65 - 32, 0); // a
        DrawSymbol(1, 86 - 32, 0); // v
        DrawSymbol(0, 69 - 32, 0); // e
      }
    }

    if (presentInput2 != lastInput2)
    {
      // change detected BTN2
      delay(20);
      if (presentInput2)
      {
        // rising edge detected
        // add hour
        volume++;
        if (volume > 30)
        {
          volume = 1;
        }

        DrawSymbol(0, (volume % 10) + 16, 1); // actual volume
        DrawSymbol(1, (volume / 10) + 16, 1); // actual volume
      }
    }
    break;

  case 10:
    // menu 4

    if (presentInput1 != lastInput1)
    {
      // change detected BTN1
      delay(20);
      if (presentInput1)
      {
        // rising edge detected
        SetRtc(0, minute, hour, dayOfWeek, dayOfMonth, month, year); // set time and zero second
        EEPROM.write(0, bright);                                     // store actual light intensity to addr 0
        EEPROM.write(1, alarmHour);
        EEPROM.write(2, alarmMinute);
        EEPROM.write(3, volume);

        systemState = 0;
      }
    }

    break;
    case 11: 
      animateEyes();
      mouth_smile();
    break;
  }
  lastInput1 = presentInput1;
  lastInput2 = presentInput2;

  if(digitalRead(BTN6) == LOW){
    Serial.println("state1");
    systemState = 11;
  }
  if(digitalRead(BTN7) == LOW){
    systemState = 0;
    Serial.println("state0");
  }
  SerialComm();
}