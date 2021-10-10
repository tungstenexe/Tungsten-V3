/********************************************************************************************************
* Nerf Rapidstrike - Tungsten V3-II 
*
* Description
* program for Nerf Rapidstrike with single shot, burst mode and full auto.
* Ammo counter. Combo mode
*
* created  18 Jun 2018
* modified 01 Aug 2018
* by TungstenEXE
*
* For non commercial use
* 
* If you find my code useful, do support me by subscribing my YouTube Channel, thanks.
*
* My YouTube Channel Link - Nerf related
* https://www.youtube.com/tungstenexe
* 
* Board used      - Arduino Nano
* Pusher Motor    - MTB Rhino Motor 
* FlyWheel Motors - MTB 180 Neo Hellcat Motor
* ESC used        - Hobbywing Quicrun 60A 2S-3S Waterproof Brushed ESC for 1/10
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce-Arduino-Wiring
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Bounce2.h>
#include <Servo.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the PWM library, to change the PWM Frequency for pin controlling the flywheel, found here :
// https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
// Note: unzip the files to the library folder, you might need to rename the folder name
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <PWM.h>

#include <SPI.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Adafruit-GFX library found here :
// https://github.com/adafruit/Adafruit-GFX-Library
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GFX.h>
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Adafruit_SSD1306 library found here :
// https://github.com/adafruit/Adafruit_SSD1306
// you might need to comment away the line "#define SSD1306_128_32" and uncomment the line
// "#define SSD1306_128_64" so that the display works for OLED screen size 128 by 64
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>

/*///////////////////////////////////////////////////////////////////////////////////////////////////////
 * The following are the setting for the ESC used, for other ESC, just chnage setting according to
 * ESC Spec
 *///////////////////////////////////////////////////////////////////////////////////////////////////////
#define THROTTLE_PUSHER_MIN         1500
#define THROTTLE_PUSHER_MAX         2000
#define THROTTLE_PUSHER_BAKE        1000
#define THROTTLE_PUSHER_REDUCE      1600
// End of Pusher ESC setting ///////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// PIN Assigment
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// The colors for for my wiring reference only you can use your own color

#define PIN_DARTCOUNT               2    // (Brown)  PIN listening to dart count 
#define PIN_FLYWHEEL_MOSFET         3    // (Orange) PIN to control DC Flywheel MOSFET 
#define PIN_OLED_RESET              4    //          for OLED
#define PIN_REV                     5    // (White)  PIN listening to change in the Nerf Rev Button 
#define PIN_DARTTRIGGER             6    // (Purple) PIN listening to trigger pull event
#define PIN_HALL_ONE                7    // (Yellow) Hall Sensor left
#define PIN_HALL_TWO                8    // (blue)   Hall Sensor right
#define PIN_PUSHERESC               9    // (Grey)   PIN to control Pusher ESC, normally the white wire from ESC 
#define PIN_SELECTOR_ONE            10   // (Grey)   Selector switch one
#define PIN_SELECTOR_TWO            11   // (Green)  Selector switch two
#define PIN_DARTRESET               12   // (Brown)  PIN listening to reset counter event 
  
#define PIN_SAFETY                  A0   // (Orange) PIN Listening to safety switch
#define PIN_ENCODER_BTN             A1   // (Purple) PIN listening to button press of rotary encoder
#define PIN_ENCODER_S1              A2   // (Grey)   PIN listening to change of state of rotary encoder
#define PIN_ENCODER_S2              A3   // (White)  PIN listening to change of state of rotary encoder
// Note                             A4      (Blue)   are used by the OLED SDA (Blue)
// Note                             A5      (Yellow) are used by the OLED SCL (Yellow)
#define PIN_VOLTREAD                A6   // (Yellow) PIN to receive voltage reading from battery 

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// End Of PIN Assigment
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AMMO_UPPER_LIMIT            35   // Maxmimum ammo configurable
#define AMMO_LOWER_LIMIT            6    // Minimum  ammo configurable

#define MODE_SINGLE                 0    // Integer constant to indicate firing single shot
#define MODE_BURST                  1    // Integer constant to indicate firing burst
#define MODE_AUTO                   2    // Integer constant to indicate firing full auto
#define NUM_OF_MODE                 3    // Number of mode available

#define MODE_ROF_LOW                0    // Integer constant to indicate low rate of fire 
#define MODE_ROF_STANDARD           2    // Integer constant to indicate standard rate of fire 
#define MODE_ROF_HIGH               9    // Integer constant to indicate highest rate of fire 
#define NUM_OF_MODE_ROF             10   // Number of ROF available
                                         
#define REV_UP_DELAY                150  // Increase/decrease this to control the flywheel rev-up time (in milliseconds) 

#define CONFIG_NONE                 0    // Indicate not in configuration mode
#define CONFIG_ROF                  1    // Indicate in fine configuration mode for Rate of Fire
#define CONFIG_FW_SPEED             2    // Indicate in fine configuration mode for flywheel speed
#define NUM_OF_CONFIG               3    // Total number of configuration available
#define NUM_OF_FW_SPEED             10   // Number of Fly wheel speed available

#define BATTERY_MIN                 10.8 // Minimum voltage of battery for rev to operate
#define BATTERY_MIN_3DIGIT          108  // Minimum voltage of battery for rev to operate
#define BATTERY_MAX_3DIGIT          123  // Maximun, any value above 12.3 volts is consider to be full battery life 

int     rofSingleShot             = 40;  // number are in percentage
int     rofLimitArr    []         = {30, 35, 40, 45, 50, 55, 60, 65, 70, 75};  // number are in percentage
int     modeROFSelected           = MODE_ROF_STANDARD;     // track the ROF selected
int     pusherThrottle            = THROTTLE_PUSHER_BAKE;  

int     fwLimitArr     []         = {55, 60, 65, 70, 75, 80, 85, 90, 95, 100}; // number are in percentage
int     fwSpeedSelected           = 9; // last in the array, which is 100%
int     modeOfConfig              = CONFIG_NONE;

int     ammoLimitArrSize          = 3;                 
int     ammoLimitArr      []      = {22, 15, 12}; // Type of mag capacity
int     ammoLimit                 = 18;           // default set as 18 dart mag
int     burstLimit                = 3;            // default set as 3 darts per burst

int     dartToBeFire              = 0;            // track amount of dart(s) to fire when trigger pulled, 0 by default
int     dartLeft                  = ammoLimit;    // track amount of dart in mag, same default value as of ammoLimit
float   currentVoltage            = 99.0;

boolean isModeFullAuto            = false;        // track is on Full Auto
boolean batteryLow                = false;        // track battery status
boolean isFiring                  = false;        // track if blaster firing         
boolean magOut                    = false;        // track is there a mag 
boolean safetyOn                  = false;        // track is safetyOn 
boolean isMagDetecting            = false;        // track is detecting mag capacity 

unsigned long timerMagDetectStart = 0;             
const    long timerMagDeThreshold = 1000;         // time allowed in millis to detect mag capactity

int     limitSelect_state_S1;  // track rotary encoder pin status
int     limitSelect_state_S2;  // track rotary encoder pin status

int32_t frequency     = 10000; //frequency (in Hz) for PWM controlling Flywheel motors

Servo   pusherESC;

Adafruit_SSD1306 display(PIN_OLED_RESET);

// Declare and Instantiate Bounce objects
Bounce btnDartCount      = Bounce(); 
Bounce btnRev            = Bounce(); 
Bounce sensorHallOne     = Bounce();
Bounce sensorHallTwo     = Bounce();
Bounce btnTrigger        = Bounce(); 
Bounce switchSelectorOne = Bounce();
Bounce switchSelectorTwo = Bounce();
Bounce btnDartReset      = Bounce();
Bounce btnEncoder        = Bounce(); 
Bounce btnSafety         = Bounce();

int voltagePinAnalogValue;

void setup() { // initilze  

  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();

  //sets the frequency for the specified pin
  bool success = SetPinFrequencySafe(PIN_FLYWHEEL_MOSFET, frequency);
  
  // if the pin frequency was set successfully, turn pin 13 on, a visual check
  // can be commented away in final upload to arduino board
  if(success) {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);    
  }
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // INPUT PINs setup
  // Note: Most input pins will be using internal pull-up resistor. A fall in signal indicate button pressed.
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
   
  pinMode(PIN_DARTCOUNT, INPUT_PULLUP);       // PULLUP
  btnDartCount.attach(PIN_DARTCOUNT);  
  btnDartCount.interval(5);

  pinMode(PIN_REV,INPUT_PULLUP);              // PULLUP
  btnRev.attach(PIN_REV);
  btnRev.interval(5);
  
  pinMode(PIN_HALL_ONE,INPUT_PULLUP);         // PULLUP
  sensorHallOne.attach(PIN_HALL_ONE);
  sensorHallOne.interval(5);

  pinMode(PIN_HALL_TWO,INPUT_PULLUP);         // PULLUP
  sensorHallTwo.attach(PIN_HALL_TWO);
  sensorHallTwo.interval(5);
  
  pinMode(PIN_DARTTRIGGER,INPUT_PULLUP);      // PULLUP
  btnTrigger.attach(PIN_DARTTRIGGER);
  btnTrigger.interval(5);

  pinMode(PIN_SELECTOR_ONE,INPUT_PULLUP);     // PULLUP
  switchSelectorOne.attach(PIN_SELECTOR_ONE);
  switchSelectorOne.interval(5);

  pinMode(PIN_SELECTOR_TWO,INPUT_PULLUP);     // PULLUP
  switchSelectorTwo.attach(PIN_SELECTOR_TWO);
  switchSelectorTwo.interval(5);

  pinMode(PIN_DARTRESET, INPUT_PULLUP);       // PULLUP
  btnDartReset.attach(PIN_DARTRESET);
  btnDartReset.interval(5);

  pinMode(PIN_SAFETY, INPUT_PULLUP);          // PULLUP
  btnSafety.attach(PIN_SAFETY);  
  btnSafety.interval(5);

  //pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);   // PULLUP
  btnEncoder.attach(PIN_ENCODER_BTN);
  btnEncoder.interval(5);

  pinMode(PIN_ENCODER_S1, INPUT);             // Not using PULLUP
  pinMode(PIN_ENCODER_S2, INPUT);             // Not using PULLUP

  pinMode(PIN_VOLTREAD, INPUT);               // Not using PULLUP analog read 0 to 1023

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // OUTPUT PINs setup
  ///////////////////////////////////////////////////////////////////////////////////////////////////////  
  
  pinMode (PIN_FLYWHEEL_MOSFET, OUTPUT);
  pwmWrite(PIN_FLYWHEEL_MOSFET, 0);  
  
  magOut     = (digitalRead(PIN_DARTRESET) == HIGH);
  safetyOn   = (digitalRead(PIN_SAFETY) == LOW);
  burstLimit = ((digitalRead(PIN_SELECTOR_TWO) * 2) + (digitalRead(PIN_SELECTOR_ONE) * 1)) + 1;

  int ammoSelectIdx = (digitalRead(PIN_HALL_ONE) * 2) + (digitalRead(PIN_HALL_TWO) * 1); // 0, 1, 2, or 3
  if (ammoSelectIdx != 3 && dartLeft != ammoLimitArr[ammoSelectIdx]) {
    dartLeft = ammoLimitArr[ammoSelectIdx];
  }

  pusherESC.attach(PIN_PUSHERESC);
  pusherESC.writeMicroseconds(THROTTLE_PUSHER_MIN);
  delay(2000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();
  if (magOut) {
    dartLeft     = 0;
    updateMagOutDisplay(); 
  } else if (safetyOn) {
    updateSafetyDisplay();
  } else {
    updateSettingDisplay();
  }
}

void loop() { // Main Loop  
  if (!batteryLow) {  
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Update all buttons
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
    btnDartCount.update();
    btnRev.update();
    sensorHallOne.update();
    sensorHallTwo.update();
    btnTrigger.update();
    switchSelectorOne.update();
    switchSelectorTwo.update();    
    btnDartReset.update();    
    btnEncoder.update();
    btnSafety.update();

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Encoder Button
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (safetyOn && btnEncoder.fell()) { // pressed
       modeOfConfig = ++modeOfConfig % NUM_OF_CONFIG;

       if (modeOfConfig != CONFIG_NONE) {
        limitSelect_state_S1 = digitalRead(PIN_ENCODER_S1);
        limitSelect_state_S2 = digitalRead(PIN_ENCODER_S2);

        updateEncoderDisplay();
       } else {
        magOut     = (digitalRead(PIN_DARTRESET) == HIGH);
        safetyOn   = (digitalRead(PIN_SAFETY) == LOW);
        burstLimit = ((digitalRead(PIN_SELECTOR_TWO) * 2) + (digitalRead(PIN_SELECTOR_ONE) * 1)) + 1;

        if (magOut) {
          updateMagOutDisplay();
        } else if (safetyOn) {
          updateSafetyDisplay();
        } else {
          updateSettingDisplay();      
        }        
       }
    }

    if (modeOfConfig != CONFIG_NONE) { // IN Setting Mode            
      int updateValue = (modeOfConfig == CONFIG_ROF) ? getLimitUpdateValue(modeROFSelected, (NUM_OF_MODE_ROF - 1), 0) : getLimitUpdateValue(fwSpeedSelected, (NUM_OF_FW_SPEED - 1), 0) ;

      if (updateValue != 0) {
        if (modeOfConfig == CONFIG_ROF) {
          modeROFSelected = (modeROFSelected + updateValue) % NUM_OF_MODE_ROF;
        } else {
          fwSpeedSelected = (fwSpeedSelected + updateValue) % NUM_OF_FW_SPEED;
        }        
        updateEncoderDisplay();
      }

      // For testing, Revving flywheel by pressing the Rev button during FW speed config
      // Can be commented away during final upload to arduino board if you do not want this function
      if (btnRev.fell()) {        // press 
        pwmWrite(PIN_FLYWHEEL_MOSFET, map(fwLimitArr[fwSpeedSelected] , 0, 100, 0, 255));
      } else if (btnRev.rose()) { // released
        pwmWrite(PIN_FLYWHEEL_MOSFET, 0);
      }
      // END of testing Rev
                  
    } else { // NOT in Setting Mode              
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // Listen to Mag Out
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      if (btnDartReset.fell()) { // pressed, there is a Mag in the blaster
        dartLeft = ammoLimit;
        magOut   = false;      
  
        isMagDetecting      = true;
        timerMagDetectStart = millis();
        
        if (safetyOn) {
          updateSafetyDisplay();
        } else {
          updateSettingDisplay();      
        }
      } else if (btnDartReset.rose()) { // No Mag in the blaster      
        shutdownSys();
        magOut = true;
        updateMagOutDisplay();
      }
  
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // Detecting to Mag Capacity
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      if (isMagDetecting) {
        if ((millis() - timerMagDetectStart) > timerMagDeThreshold ) {
          isMagDetecting = false;
          timerMagDetectStart = 0;               
        } else {
          int ammoSelectIdx = (digitalRead(PIN_HALL_ONE) * 2) + (digitalRead(PIN_HALL_TWO) * 1); // 0, 1, 2, or 3
          if (ammoSelectIdx != 3 && dartLeft != ammoLimitArr[ammoSelectIdx]) {
            dartLeft = ammoLimitArr[ammoSelectIdx];
            if (safetyOn) {
              updateSafetyDisplay();
            } else {
              updateSettingDisplay();      
            }
          }        
        }      
      }
  
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // Listen to Safety
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      if (btnSafety.fell()) {               // Safety on
        safetyOn = true;
        shutdownSys(); 
        
        if (!magOut) {
          updateSafetyDisplay();
        }
      } else if (btnSafety.rose()) {        // Safety off
        safetyOn     = false;
        modeOfConfig = CONFIG_NONE;
        if (!magOut) {
          updateSettingDisplay();
        }
      }
  
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // Listen to Rev Press/Release
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      if (btnRev.fell()) {                   // press
        if (magOut) {
          if (ammoLimit > AMMO_LOWER_LIMIT) {
            ammoLimit--;      
            updateMagOutDisplay();
          }
        } else if (safetyOn) {
          if (digitalRead(PIN_DARTTRIGGER) == LOW ) {           
            switch(modeROFSelected) {
              case MODE_ROF_LOW:
                  modeROFSelected = MODE_ROF_STANDARD;
                break;
              case MODE_ROF_STANDARD:
                  modeROFSelected = MODE_ROF_HIGH;
                break;
              case MODE_ROF_HIGH:
                  modeROFSelected = MODE_ROF_LOW;
                break;
              default:
                  modeROFSelected = MODE_ROF_STANDARD;
            }
            
            updateSafetyDisplay();
          }      
        } else { // V2 mode      
          if (digitalRead(PIN_DARTTRIGGER) == HIGH) {
            triggerPressedHandle(MODE_BURST);
            isModeFullAuto = false;
          } else {
            triggerPressedHandle(MODE_AUTO);
            isModeFullAuto = true;
          }              
        }
      } else if (btnRev.rose()) {        // released
        triggerReleasedHandle();
      }
    
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // Listen to Trigger Pull/Release
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      if (btnTrigger.fell()) {               // pull
        if (magOut) {
          if (ammoLimit < AMMO_UPPER_LIMIT) {
            ammoLimit++;      
            updateMagOutDisplay();
          }
        } else if (safetyOn) { // do nothing
          
        } else { // V2 mode
          if (digitalRead(PIN_REV) == HIGH) {
            triggerPressedHandle(MODE_SINGLE);
            isModeFullAuto = false;
          } else {
            triggerPressedHandle(MODE_AUTO);
            isModeFullAuto = true;
          }           
        }        
      } else if (btnTrigger.rose()) {        // released
        triggerReleasedHandle();
      }
    
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // Listen to Dart Count switch
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      if (btnDartCount.rose()) {        // counter switch released, pusher is extending towards dart 
        shotFiredHandle();              // for firing
      } else if (btnDartCount.fell()) { // pusher is returning from firing dart
        shotFiredReturnHandle();
      }
    
    
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // Listen to Firing Mode change: Single Shot, Burst, Full Auto
      /////////////////////////////////////////////////////////////////////////////////////////////////////
  
      if (switchSelectorOne.fell() || switchSelectorOne.rose() || switchSelectorTwo.fell() || switchSelectorTwo.rose() ) {  
          burstLimit = ((digitalRead(PIN_SELECTOR_TWO) * 2) + (digitalRead(PIN_SELECTOR_ONE) * 1)) + 1; // 2, 3 or 4    
  
        if (magOut) {
          updateMagOutDisplay();
        } else if (safetyOn) {
          updateSafetyDisplay();
        } else {
          updateSettingDisplay();
        }
      }
    }
  } else {
    // Battery is Low
    // Stop all Motors just in case.
    shutdownSys();    
    updateBatteryLowDisplay();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shotFiredHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shotFiredHandle() {
  if (dartLeft > 0) {
    // to refresh the whole display is too time consuming
    // therefore just overwrite the old dart count by switching to background color
    // write the current count on the screen, this remove it from the screen
    display.setTextColor(BLACK);
    display.setCursor(90,18);
    display.print(dartLeft);
    
    dartLeft--;     // decrease dart count
    dartToBeFire--;    

    switch (dartToBeFire) {
      case 1: 
          pusherESC.writeMicroseconds(THROTTLE_PUSHER_REDUCE);
        break;
      case 2: 
        if (rofLimitArr[modeROFSelected] > 50) {
          pusherESC.writeMicroseconds(map(rofSingleShot, 0, 100, THROTTLE_PUSHER_MIN, THROTTLE_PUSHER_MAX));
        }
        break;
    }

    // switch back to white text and write the new count
    display.setTextColor(WHITE);
    display.setCursor(90,18);
    display.print(dartLeft);
    display.display();    
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shotFiredReturnHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shotFiredReturnHandle() {
    if (dartLeft <= 0 || dartToBeFire <= 0) {   
      pusherESC.writeMicroseconds(THROTTLE_PUSHER_BAKE); // make sure pusher motor stops
      isFiring = false;

      pwmWrite(PIN_FLYWHEEL_MOSFET, 0);
          
      dartToBeFire = 0;
      updateSettingDisplay();       
    } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerPressedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerPressedHandle(int caseModeFire) {  
  if (dartLeft > 0){
    switch(caseModeFire) {
      case MODE_SINGLE: dartToBeFire++; break;
      case MODE_BURST : dartToBeFire += burstLimit; 
          if (dartToBeFire > dartLeft) {
            dartToBeFire = dartLeft;
          }
        break;
      case MODE_AUTO  : dartToBeFire = dartLeft;
    }    

    if (dartToBeFire < 2) {
      pusherThrottle = map(rofSingleShot, 0, 100, THROTTLE_PUSHER_MIN, THROTTLE_PUSHER_MAX);
    } else {
      pusherThrottle = map(rofLimitArr[modeROFSelected] , 0, 100, THROTTLE_PUSHER_MIN, THROTTLE_PUSHER_MAX);
    }

    // Start Firing
    display.setTextSize(3);

    pwmWrite(PIN_FLYWHEEL_MOSFET, map(fwLimitArr[fwSpeedSelected] , 0, 100, 0, 255));    
    delay(REV_UP_DELAY);

    // start pusher
    pusherESC.writeMicroseconds(pusherThrottle);    
    isFiring = true;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerReleasedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerReleasedHandle() {  
  if (isModeFullAuto && isFiring) {
    if (dartToBeFire > 1) {
      pusherESC.writeMicroseconds(THROTTLE_PUSHER_REDUCE);
      dartToBeFire = 2;    // fire off last shot
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: getLimitUpdateValue
// For getting value from rotary encoder          
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int getLimitUpdateValue(int currentLimit, int upperLimit, int lowerLimit) {
  int updateValue  = 0;

  int newLimitSelect_stateS2 = digitalRead(PIN_ENCODER_S2);
  int newLimitSelect_stateS1 = digitalRead(PIN_ENCODER_S1);
  
  if ((newLimitSelect_stateS2 != limitSelect_state_S2) || (newLimitSelect_stateS1 != limitSelect_state_S1)) {
    if (newLimitSelect_stateS2 == LOW && limitSelect_state_S2 == HIGH) { 
      updateValue = (limitSelect_state_S1 * 2) - 1;                     
      if (((currentLimit == upperLimit) && (updateValue == 1)) || 
          ((currentLimit == lowerLimit) && (updateValue == -1))) {
        updateValue = 0;
      }
    }
    limitSelect_state_S1 = newLimitSelect_stateS1;
    limitSelect_state_S2 = newLimitSelect_stateS2;
  }
  
  return updateValue;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: readVoltage
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void readVoltage() {
  voltagePinAnalogValue = analogRead(PIN_VOLTREAD);
    
  int   voltagePinValue = (int) (voltagePinAnalogValue / 0.4048 );    // 0.4281
  float newVoltage      = (voltagePinValue / 100.0);                  // 10.0

  if (!batteryLow) {
    currentVoltage = (newVoltage < currentVoltage) ? newVoltage : currentVoltage;      
  } else {
    currentVoltage = (newVoltage > BATTERY_MIN) ? newVoltage : currentVoltage;  
  }
  batteryLow = (currentVoltage <= BATTERY_MIN);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateSettingDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateSettingDisplay() {
  readVoltage();
  int numOfCircle = burstLimit;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
  
  display.setCursor(0,19);
  display.println("Ammo Count");  
  
  display.setCursor(0,32);
  display.println("TEXE-V3-II");  
    
  display.setCursor(4,57);

  int numOfSpace = 9 - modeROFSelected;
  int numOfArrow = 1 + modeROFSelected;
  for(int i=0; i<numOfSpace; i++) {
    display.print(" ");
  }
  for(int i=0; i<numOfArrow; i++) {
      display.print("<");
  }
  for(int i=0; i<numOfArrow; i++) {
      display.print(">");
  }
  
  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(dartLeft);  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateMagOutDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateMagOutDisplay() {
  readVoltage();
  
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
  
  display.setCursor(0,19);
  display.println("MAG OUT - SET");  
      
  display.setCursor(4,57);
  display.println("<*-*-*-*-*^*-*-*-*-*>");  

  display.setTextSize(2);
  display.setCursor(0,32);
  display.println("-AMMO-");  
  
  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(ammoLimit);  

  display.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: Update Safety Display
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateSafetyDisplay() {
  readVoltage();
  int numOfCircle = burstLimit;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
      
  display.setCursor(4,57);
  int numOfSpace = 9 - modeROFSelected;
  int numOfArrow = 1 + modeROFSelected;
  for(int i=0; i<numOfSpace; i++) {
    display.print(" ");
  }
  for(int i=0; i<numOfArrow; i++) {
      display.print("<");
  }
  for(int i=0; i<numOfArrow; i++) {
      display.print(">");
  }

  display.setTextSize(2);
  display.setCursor(0,21);
  display.println("SAFETY");  

  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(dartLeft);  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: Update Encoder Display
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateEncoderDisplay() {
  readVoltage();
  int numOfCircle = burstLimit;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
      
  display.setCursor(4,57);

  int arrowValue = (modeOfConfig == CONFIG_ROF) ? modeROFSelected : fwSpeedSelected;
  int numOfSpace = 9 - arrowValue;
  int numOfArrow = 1 + arrowValue;
  for(int i=0; i<numOfSpace; i++) {
    display.print(" ");
  }
  for(int i=0; i<numOfArrow; i++) {
      display.print("<");
  }
  for(int i=0; i<numOfArrow; i++) {
      display.print(">");
  }

  display.setTextSize(2);
  display.setCursor(0,21);

  if (modeOfConfig == CONFIG_ROF) {
    display.println("R.O.F");  
  } else {
    display.println("FWSPEED");  
  }

  display.setCursor(90,18);
  display.setTextSize(3);
  display.println((arrowValue + 1));  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateBatteryLowDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateBatteryLowDisplay() {
  readVoltage();
  if (batteryLow) {
    display.clearDisplay();
        
    display.setTextSize(2);
    display.setCursor(0,14);
    display.println("+-------+");  
    display.println("|BattLow|=");  
    display.println("+-------+");  
  
    display.display();
  } else {
    if (magOut) {
      updateMagOutDisplay();
    } else if (safetyOn) {
      updateSafetyDisplay();
    } else {
      updateSettingDisplay();
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shutdown
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shutdownSys() {
  dartToBeFire = 0;
  pusherESC.writeMicroseconds(THROTTLE_PUSHER_BAKE);
  pwmWrite(PIN_FLYWHEEL_MOSFET, 0);
  isFiring     = false;
}
