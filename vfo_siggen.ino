//
//      _          VFO / Signal Generator       _     _ _     _
//   __| | __ _  ___ _ __ ___   ___  _ __   ___| |__ (_) | __| |
//  / _` |/ _` |/ _ \ '_ ` _ \ / _ \| '_ \ / __| '_ \| | |/ _` |
// | (_| | (_| |  __/ | | | | | (_) | | | | (__| | | | | | (_| |
//  \__,_|\__,_|\___|_| |_| |_|\___/|_| |_|\___|_| |_|_|_|\__,_|
//
//  Author: Tom Rowan, 2W0KKR
//

// Import Library Code

#include <Wire.h>
#include <SPI.h>
#include <si5351.h>
#include "Rotary.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// Debug Mode?
#define DEBUG 1

// Rotary Encoders
#define DIAL1_CCW   2   // Dial 1, CCW
#define DIAL1_CW    3   // Dial 1, CW
#define DIAL1_PUSH  4   // Dial 1, Push Button
#define DIAL2_CW    5   // Dial 2, CCW
#define DIAL2_CCW   6   // Dial 2, CW
#define DIAL2_PUSH  7   // Dial 2, Push Button

// Buzzer
#define BUZZER_PIN  9   // PE Buzzer

// Status LEDs
#define LED_1       10   // Active when VFO active
#define LED_2       11   // Active when VFO active
#define LED_3       12   // Active when VFO active

// OLED Display
#define SCREEN_WIDTH    128   // in Pixels
#define SCREEN_HEIGHT   32    // in Pixels
#define OLED_RESET      -1    // Share Reset PIN with Arduino
#define SCREEN_ADDRESS  0x3C 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Project Specific Constants
#define MHZ       1000000
#define KHZ       1000
#define FREQ_MULT 100LLU  // Allows us to work in Hz, saving memory

// Supported Frequency Range
#define MIN_VFO 8   * KHZ     // VFO min. frequency 
#define MAX_VFO 160 * MHZ     // VFO max. frequency 

// Menu Modes
#define MODE_STEP       0
#define MODE_ENABLE     1
#define MODE_MODE       2
#define MODE_CALIBRATE  3
#define MODE_SAVE       4

// Data Structure for Step Data
typedef struct
{
  char *name; // Display text
  long value; // Frequency value in Hz
} Step;

// Step data
Step step[] = {
  {(char *) " 10  Hz", 1 },
  {(char *) "100  Hz", 100 },
  {(char *) "  1  KHz", 1 * KHZ },
  {(char *) "  5  KHz", 5 * KHZ },
  {(char *) " 10  KHz", 10 * KHZ },
  {(char *) "12.5 KHz", 12.5 * KHZ },
  {(char *) "100  KHz", 100000 }, 
  {(char *) "500  KHz", 500000 }, 
  {(char *) "  1  Mhz", 1 * MHZ },
  {(char *) " 10  Mhz", 10 * MHZ }
}; 

// Calculate the index of last position of step[] array
const int lastStepVFO = (sizeof step / sizeof(Step)) - 1;
int stepIndex = 7;

#define CHANNEL_A   0
#define CHANNEL_B   1
#define CHANNEL_C   2

// My Variables - Config
unsigned long correction_factor = 0;
byte channel = CHANNEL_A;
byte mode = 0;

byte setup_check = 1;

// Data Structure for Config 

#define MODE_SG   1
#define MODE_VFO  2

typedef struct
{
  char *name;     // Display text
  long freq;      // Frequency value 
  int enabled;    // Status
  int  mode;      // Mode (VFO or Signal Generator)
  long IF;        // Intermediate Frequency value (VFO mode)
} ChanConfig;

// Step Data - default config
ChanConfig chanConfig[] = {
  {(char *) "A", 7.1 * MHZ, 1, MODE_SG, 10 * MHZ},   
  {(char *) "B", 3.5 * MHZ, 0, MODE_SG, 9 * MHZ},   
  {(char *) "C", 8.0 * KHZ, 0, MODE_SG, 400 * KHZ},  
}; 

#define CMD_STEP 0
#define CMD_MODE 1
#define CMD_ENABLED 2
#define CMD_SAVE 3
#define CMD_CAL 4
#define CMD_LAST CMD_CAL
byte currentCommand = CMD_STEP;

// Encoder controller
Rotary encoder = Rotary(DIAL1_CW, DIAL1_CCW);

// Si5351 Signal Generator
Si5351 si5351;

bool isFreqChanged = true;
bool clearDisplay = false;

// uint64_t vfoFreq;
uint64_t vfoLastValue;

// Encoder control variables
volatile int encoderCount = 0;


void setup()
{
  if (DEBUG) 
    Serial.begin(9600);

  // Load Config
  //fake_load();
  //eeprom_save();
  eeprom_load();

  // Configure Pin Modes

  // Rotary Encoders
  pinMode(DIAL1_CW, INPUT_PULLUP);
  pinMode(DIAL1_CCW, INPUT_PULLUP);
  pinMode(DIAL1_PUSH, INPUT_PULLUP);
  pinMode(DIAL2_CW,  INPUT_PULLUP);
  pinMode(DIAL2_CCW, INPUT_PULLUP);
  pinMode(DIAL1_PUSH, INPUT_PULLUP);

  // Buzzer
  pinMode(9, OUTPUT);
  beep(5000,50);

  // Status LEDs
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

  // LED Check
  flash (300,LED_1);
  flash (300,LED_2);
  flash (300,LED_3);

  // OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.cp437(true);
  display.setTextColor(SSD1306_WHITE);

  // Signal Generator 
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(correction_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  //si5351.set_freq(chanConfig[channel].freq, SI5351_CLK0); // Start CLK0 (VFO)
  update_clock_status();

  if (DEBUG)
    Serial.println(correction_factor);

  delay(500);

  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(DIAL1_CW), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DIAL1_CCW), rotaryEncoder, CHANGE);

  delay(1000);
  updateDisplay();

}

// Use Rotary.h and  Rotary.cpp implementation to process encoder via interrupt
void rotaryEncoder()
{ // rotary encoder events
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus)
  {
    if (encoderStatus == DIR_CW)
    {
      encoderCount = 1;
      beep(5000,50);
    }
    else
    {
      encoderCount = -1;
      beep(5000,50);
    }
  }
}

// EEPROM Functions

void eeprom_save() {

  if (DEBUG)
    Serial.println("EEPROM Write");

  int eeAddr = 0;
  EEPROM.put(eeAddr, channel);
  eeAddr += sizeof(channel);
  EEPROM.put(eeAddr, correction_factor);
  eeAddr += sizeof(correction_factor);
  EEPROM.put(eeAddr, chanConfig[0]);
  eeAddr += sizeof(chanConfig[0]);
  EEPROM.put(eeAddr, chanConfig[1]);
  eeAddr += sizeof(chanConfig[1]);
  EEPROM.put(eeAddr, chanConfig[2]);
  eeAddr += sizeof(chanConfig[2]);

}

void fake_load() {
  
  chanConfig[0] = {(char *) "A", 10.0 * KHZ, 1, MODE_SG, 0.0};   // Load from EEPROM
  chanConfig[1] = {(char *) "B", 5.0 * MHZ, 0, MODE_SG, 0.0};  // Load from EEPROM
  chanConfig[2] = {(char *) "C", 14.1 * MHZ, 0, MODE_SG, 0.0};  // Load from EEPROM

  correction_factor = 130000; // Load from EEPROM
  channel = CHANNEL_A; // Load from EEPROM

}

void eeprom_load() {

   if (DEBUG)
    Serial.println("EEPROM Read");

  int eeAddr = 0;
  EEPROM.get(eeAddr, channel);
  eeAddr += sizeof(channel);
  EEPROM.get(eeAddr, correction_factor);
  eeAddr += sizeof(correction_factor);
  EEPROM.get(eeAddr, chanConfig[0]);
  eeAddr += sizeof(chanConfig[0]);
  EEPROM.get(eeAddr, chanConfig[1]);
  eeAddr += sizeof(chanConfig[1]);
  EEPROM.get(eeAddr, chanConfig[2]);
  eeAddr += sizeof(chanConfig[2]);

}

// Pad a number string (0 -> 000)
String padNumberString(String theString) {

 switch (theString.length()) {
    case 0:
      theString = "000";
      break;
    case 1:
      theString = "00" + theString;  
      break;
    case 2:
      theString = "0" + theString;   
      break;
  }

  return theString;

}

void message() {
 

  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.print("VFO / Signal Generator");
  display.setCursor(15, 10);
  display.print("Loading...");
  display.display(); 
}

// 
void updateDisplay()
{

  // Extract VFO components for display
  int dispHzInt = (chanConfig[channel].freq % KHZ);
  String dispHz =  padNumberString(String (dispHzInt));
  String dispKhz = padNumberString(String((chanConfig[channel].freq % MHZ)/KHZ));
  String dispMhz = padNumberString(String(int(chanConfig[channel].freq/MHZ)));

  // Put together as: mhz.khz.hz, eg: 120.500.005
  String freqText = dispMhz + "." + dispKhz + "." + dispHz;

  // Update Display
  display.clearDisplay();

  // Header Grid
  display.drawLine(0,8,127,8,WHITE);
  display.drawLine(30,0,30,8,WHITE);

  display.setCursor(0, 0);
  display.print("Ch " + String(chanConfig[channel].name)); //Display Current Channel

  // Display Frequency
  display.setCursor(38, 0);
  display.print(freqText);
  display.setCursor(108, 0);
  display.print("Mhz");

  // Depends on mode now:

  display.setCursor(0, 10);
  display.print("Step: ");
  display.print(step[stepIndex].name);
  display.setCursor(0, 20);
  display.print("Mode: ");
  display.print((currentCommand == CMD_STEP) ? "STEP" : "MORE");

  display.setCursor(122, 24);
  display.print (String(chanConfig[channel].enabled));

  display.display();

  update_leds();

}

// Update the Channel Status LEDs
void update_leds() {

    digitalWrite(LED_1, chanConfig[CHANNEL_A].enabled);
    digitalWrite(LED_2, chanConfig[CHANNEL_B].enabled);
    digitalWrite(LED_3, chanConfig[CHANNEL_C].enabled);

}

// Update the Clocks
void update_clock_status() {

  si5351.output_enable(SI5351_CLK0, chanConfig[CHANNEL_A].enabled);
  si5351.output_enable(SI5351_CLK1, chanConfig[CHANNEL_B].enabled);
  si5351.output_enable(SI5351_CLK2, chanConfig[CHANNEL_C].enabled);
  si5351.update_status();
  isFreqChanged = true;

}

// Change the frequency (increment or decrement)
// direction parameter is 1 (clockwise) or -1 (counter-clockwise)
void changeFreq(int direction)
{
  chanConfig[channel].freq += step[stepIndex].value * direction; // stepIndex * direction;
  // Check the VFO limits
  if (chanConfig[channel].freq > MAX_VFO )
    chanConfig[channel].freq = MAX_VFO;
  else if (chanConfig[channel].freq < MIN_VFO)
    chanConfig[channel].freq = MIN_VFO;

  isFreqChanged = true;
}

void doCommandUp() {
  if ( currentCommand == CMD_STEP )
    stepIndex = (stepIndex < lastStepVFO) ? (stepIndex + 1) : 0;
  else {
    isFreqChanged = true;
  }

  delay(200);
}

void doCommandDown() {
  if ( currentCommand == CMD_STEP )
    stepIndex = (stepIndex < lastStepVFO) ? (stepIndex -1) : 0;
  else {
    isFreqChanged = true;
  }
  delay(200);
}

void loop()
{

  // Check if the encoder has moved.
  if (encoderCount != 0)
  {
    if (encoderCount == 1)
      changeFreq(1);
    else
      changeFreq(-1);
    encoderCount = 0;
  }

  // Switch the current command control (step or favorite frequencies)
  if (digitalRead(DIAL1_PUSH) == LOW) {
    Serial.println (channel);
    channel = channel + 1;
    if (channel > 2) {

      channel = 0;

    }
    Serial.println (channel);
    beep(7000,10);
    updateDisplay();
    delay(200);
  } else if (digitalRead(DIAL2_PUSH) == LOW) {
    currentCommand = currentCommand + 1;
      if (currentCommand > CMD_LAST)
        currentCommand = CMD_STEP;
    beep(7000,10);
    updateDisplay();
    delay(200);  
  } else if (digitalRead(DIAL2_CW) == LOW) {
    doCommandUp();
    beep(7000,10);
    updateDisplay();
  } else if (digitalRead(DIAL2_CCW) == LOW) {
    doCommandDown();
    beep(7000,10);
    updateDisplay();
  }

  if (isFreqChanged)
  {
    if (DEBUG) 
      Serial.println("Set freq: " + String(chanConfig[channel].freq) + " on channel [" + channel + "]");
    
    si5351.set_freq(chanConfig[channel].freq * FREQ_MULT, channel); //SI5351_CLK0, SI5351_CLK1, SI5351_CLK2
    updateDisplay();
    isFreqChanged = false;
  }
  delay(50);


}



void flash(unsigned int delayms, int led) {

    digitalWrite(led,HIGH);
    delay(delayms); 
    digitalWrite(led,LOW);
    delay(delayms);

}

void beep(int freq ,unsigned char delayms){

  tone(BUZZER_PIN, freq, delayms);                               
  delay(delayms);          
  
} 
