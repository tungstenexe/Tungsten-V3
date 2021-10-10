/********************************************************************************************************
* Nerf Rapidstrike - Tungsten V3 Traditional
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
#define PIN_ENCODER_S1              A2   // (Grey)  PIN listening to change of state of rotary encoder
#define PIN_ENCODER_S2              A3   // (White)   PIN listening to change of state of rotary encoder
// Note                             A4      (Blue)   are used by the OLED SDA (Blue)
// Note                             A5      (Yellow) are used by the OLED SCL (Yellow)
#define PIN_VOLTREAD                A6   // (Yellow) PIN to receive voltage reading from battery 

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// End Of PIN Assigment
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AMMO_UPPER_LIMIT            35   // Maxmimum ammo configurable
#define AMMO_LOWER_LIMIT            6    // Minimum  ammo configurable

#define BURST_UPPER_LIMIT           4    // Maxmimum burst configurable
#define BURST_LOWER_LIMIT           2    // Minimum burst configurable

#define MODE_SINGLE                 0    // Integer constant to indicate firing single shot
#define MODE_BURST                  1    // Integer constant to indicate firing burst
#define MODE_AUTO                   2    // Integer constant to indicate firing full auto
#define NUM_OF_MODE                 3    // Number of mode available

#define MODE_ROF_LOW                0    // Integer constant to indicate low rate of fire 
#define MODE_ROF_STANDARD           1    // Integer constant to indicate standard rate of fire 
#define MODE_ROF_HIGH               2    // Integer constant to indicate highest rate of fire 
#define NUM_OF_MODE_ROF             3    // Number of ROF available
                                         
#define REV_UP_DELAY                150  // Increase/decrease this to control the flywheel rev-up time (in milliseconds) 

#define BATTERY_MIN                 10.8 // Minimum voltage of battery for rev to operate
#define BATTERY_MIN_3DIGIT          108  // Minimum voltage of battery for rev to operate
#define BATTERY_MAX_3DIGIT          123  // Maximun voltage above 12.3 is consider to be full

int     rofSingleShot             = 40;                  // number are in percentage
int     rofLimitArr    []         = {30, 40, 75};        // number are in percentage
String  rofLimitStrArr []         = {"        <<<>>>", "     <<<<<<>>>>>>", "<<<<<<<<<<<>>>>>>>>>>>"}; 
int     modeROFSetted             = MODE_ROF_STANDARD;   // track the ROF setted
int     modeROFSelected           = MODE_ROF_STANDARD;   // track the ROF selected
int     pusherThrottle            = THROTTLE_PUSHER_BAKE;

int     ammoLimitArrSize          = 3;                 
int     ammoLimitArr      []      = {22, 15, 12}; 
int     ammoLimit                 = 18;                  // default set as 18 dart mag
int     burstLimit                = 3;                   // default set as 3 darts per burst

int     modeFire                  = MODE_SINGLE;         // track the mode of fire, Single, Burst or Auto, Single by default
int     dartToBeFire              = 0;                   // track amount of dart(s) to fire when trigger pulled, 0 by default
int     dartLeft                  = ammoLimit;           // track amount of dart in mag, same default value as of ammoLimit
float   currentVoltage            = 99.0;

boolean batteryLow                = false;       // track battery status
boolean isRevving                 = false;       // track if blaster firing         
boolean isFiring                  = false;       // track if blaster firing         
boolean magOut                    = false;       // track is there a mag 
boolean safetyOn                  = false;       // track is safetyOn 
boolean isMagDetecting            = false;       // track is detecting mag capacity on

unsigned long timerMagDetectStart = 0;
const    long timerMagDeThreshold = 1000;

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

void setup() { // initilze  
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

  //pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);     // PULLUP
  btnEncoder.attach(PIN_ENCODER_BTN);
  btnEncoder.interval(5);

  pinMode(PIN_SAFETY, INPUT_PULLUP);          // PULLUP
  btnSafety.attach(PIN_SAFETY);  
  btnSafety.interval(5);

  pinMode(PIN_VOLTREAD, INPUT);               // Not using PULLUP analog read 0 to 1023

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // OUTPUT PINs setup
  ///////////////////////////////////////////////////////////////////////////////////////////////////////  

  digitalWrite(PIN_FLYWHEEL_MOSFET,LOW);  
  pinMode(PIN_FLYWHEEL_MOSFET, OUTPUT);
  
  magOut   = (digitalRead(PIN_DARTRESET) == HIGH);
  safetyOn = (digitalRead(PIN_SAFETY) == LOW);
  modeFire = 3 - ((digitalRead(PIN_SELECTOR_TWO) * 2) + (digitalRead(PIN_SELECTOR_ONE) * 1)); // 0, 1 or 2        
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
      safetyOn = false;
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
          modeROFSelected = ++modeROFSelected % NUM_OF_MODE_ROF;
          modeROFSetted   = modeROFSelected;          
          updateSafetyDisplay();
        }      
      } else {      
        isRevving = true;
        digitalWrite(PIN_FLYWHEEL_MOSFET, HIGH); // start flywheels
      }
    } else if (btnRev.rose()) {        // released
      isRevving = false;
      if (!isFiring) {        
        digitalWrite(PIN_FLYWHEEL_MOSFET, LOW); // stop flywheels
      }
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
      } else if (safetyOn) {
        if (digitalRead(PIN_REV) == LOW ) {
          burstLimit = (burstLimit == BURST_UPPER_LIMIT) ? BURST_LOWER_LIMIT : burstLimit + 1;
          updateSafetyDisplay();
        }
      } else {
        triggerPressedHandle(modeFire);
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
        modeFire = 3 - ((digitalRead(PIN_SELECTOR_TWO) * 2) + (digitalRead(PIN_SELECTOR_ONE) * 1)); // 0, 1 or 2    

      if (!magOut && !safetyOn) {
        updateSettingDisplay();
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
        if (rofLimitArr[modeROFSetted] > 50) {
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

      if (!isRevving) {
        digitalWrite(PIN_FLYWHEEL_MOSFET, LOW); // stop flywheels
      }
          
      dartToBeFire = 0;
      updateSettingDisplay();   
    
    } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerPressedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerPressedHandle(int caseModeFire) {  
  //updateSettingDisplay();
  if (dartLeft > 0 && isRevving){
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
      pusherThrottle = map(rofLimitArr[modeROFSetted] , 0, 100, THROTTLE_PUSHER_MIN, THROTTLE_PUSHER_MAX);
    }

    // Start Firing
    isFiring = true;
    display.setTextSize(3);
    pusherESC.writeMicroseconds(pusherThrottle);    
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerReleasedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerReleasedHandle() {  
  if (modeFire == MODE_AUTO && isFiring) {
    if (dartToBeFire > 1) {
      pusherESC.writeMicroseconds(THROTTLE_PUSHER_REDUCE);
      dartToBeFire = 2;    // fire off last shot
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: readVoltage
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void readVoltage() {
  int voltagePinAnalogValue = analogRead(PIN_VOLTREAD);
    
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
  int numOfCircle = 1;
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
  display.println("TEXE-V3-<T>");  
  
  display.setCursor(0,32);
  
  switch(modeFire) {
      case MODE_SINGLE: 
          display.println("Single Shot");  
        break;
      case MODE_BURST : 
          numOfCircle = burstLimit;
          display.print(burstLimit);          
          display.println(" Rounds Burst");  
        break;
      case MODE_AUTO  : 
          numOfCircle = 10;
          display.println("Full Auto");  
        break;
    }
    
  display.setCursor(0,57);
  display.println(rofLimitStrArr[modeROFSetted]);  
  
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
      
  display.setCursor(0,57);
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
// Function: updateSafetyDisplay
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
      
  display.setCursor(0,57);
  display.println(rofLimitStrArr[modeROFSelected]);  

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
  digitalWrite(PIN_FLYWHEEL_MOSFET, LOW);
  isFiring     = false;
}
