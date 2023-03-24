////////////////////////////////////////////////////////////////////////
//
//  LIBRARIES   LIBRARIES   LIBRARIES   LIBRARIES   LIBRARIES
//
////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>                       // Include library for writing/reading to/from eeprom
#include <LiquidCrystal_I2C.h>            // Include the LCD i2c library
#include "pitches.h"                      // include library of pitches for sending tones to speaker

////////////////////////////////////////////////////////////////////////
//
//  GLOBALS   GLOBALS   GLOBALS   GLOBALS   GLOBALS   GLOBALS   GLOBALS
//
////////////////////////////////////////////////////////////////////////

/////////////     PIN DEFINITIONS     ///////////////////////////
const int red_button_interrupt_pin = 2;
const int rotary_encoder_interrupt_pin = 3;
const int rotary_encoder_other_pin = 4;
const int speaker_pin = 7;
const int relay_pin = 10;
const int rotary_button_pin = 8;
const int pressure_pin_inner = A0;
const int pressure_pin_outer = A1;
const int led_pin = 13;

////////////     CONSTANTS      /////////////////////////////////////
const int DEBOUNCE_TIME = 15;                 // Time limit for debouncing [ms]
const int RED_BUTTON_LONG_PRESS_LIMIT = 2500; // Duration of a long red button press [ms]
const int ROTARY_BUTTON_SAMPLE_TIME = 50;     // How often the system samples for a rotary encoder button press [ms]
const int PRESSURE_SAMPLE_TIME = 100;         // How often the system will sample for pressure [ms]
const float ANALOG_TO_V_MULTIPLIER = 5./1023.;// Multiply analog in values by this constant to get voltage in [no units]
const int INCREMENT_PRESSURE = 1;             // Increment pressure by this amount on each rotary encoder click [%]
const int INCREMENT_TIME = 100;               // Increment time by this amount on each rotary encoder click [ms]
// const int MAX_PRESSURE_NO_VACUUM = 5;         // Pressure above which there is a vacuum being pulled, whether or not the suctions are sealed [%]
const int MAX_TIME_TO_SEAL = 1000;            // Maximum time for a packet to form a suction seal after vacuum is enabled [ms]
const int LCD_ROWS = 4;                       // Rows in the LCD display [characters]
const int LCD_COLUMNS = 20;                   // Columns in the LCD display [characters]
const int LCD_ADDRESS = 0x27;                 // Address of the LCD display

// Declare some space in the EEPROM with addresses for the stored variables (to save user-defined settings when system is shut down)

const int EEPROM_P_IN_BASE_ADDRESS = 0;
const int EEPROM_P_OUT_BASE_ADDRESS = EEPROM_P_IN_BASE_ADDRESS + sizeof(int);

const int EEPROM_P_IN_ADDRESS = EEPROM_P_OUT_BASE_ADDRESS + sizeof(int);

const int EEPROM_P_OUT_ADDRESS = EEPROM_P_IN_ADDRESS + sizeof(int);
const int EEPROM_TIME_ADDRESS = EEPROM_P_OUT_ADDRESS + sizeof(int);
const int EEPROM_AUDIO_ADDRESS = EEPROM_TIME_ADDRESS + sizeof(long);
const int EEPROM_OUTLET_ADDRESS = EEPROM_AUDIO_ADDRESS + sizeof(int);
const int EEPROM_P_BUF_ADDRESS = EEPROM_OUTLET_ADDRESS + sizeof(int);

///////////     MUSIC CONSTANTS    ////////////////////////////////////
const int NOTES[] = {NOTE_C4, NOTE_B4, NOTE_A4, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5};   // The pithces of the melody to play
const int NOTE_DURATIONS[] = {2,2,4,4,2,2,2,1};                                    // The durations of the notes in the melody [1=whole note, 4=quarter note, 8=eigth note, etc.]
                                                                                          //  (ie. the higher the number, the shorter the note)
const long MEASURE_DURATION = 2000;                                                       // The duration of a single musical measure [ms]

///////////     STATE INDEXES      ////////////////////////////////////
const int STATE_INIT = 0;          // Define the states for the state machine
const int STATE_HOME = 1;          // If adding more states, make sure to update LAST_STATE as well
const int STATE_SETTINGS_1 = 2;
const int STATE_TIME = 3;
const int STATE_P_IN_BASELINE = 4;
const int STATE_P_OUT_BASELINE = 5;
const int STATE_SETTINGS_2 = 6;
const int STATE_P_IN = 7;
const int STATE_P_OUT = 8;
const int STATE_P_BUF = 9;
const int STATE_SETTINGS_3 = 10;
const int STATE_AUDIO = 11;
const int STATE_OUTLET = 12;

const int LAST_STATE = 12;         // Index of last state in state machine (UPDATE IF STATES ARE ADDED OR REMOVED!!!)

//////////      GLOBALS CHANGED IN INTERRUPTS //////////////////////////////////////
volatile int ROTATION = 0;                    // Var for holding rotary encoder change (Flag with options -1: widdershins turn, 0: no turn, 1: clockwise turn)
volatile int RED_BUTTON_ACTIVE = 0;           // Flag indicating whether red button is currently pushed down (1: yes, 0: no)
volatile long RED_BUTTON_START_TIME = 0;      // Stores time at which red button was first pushed down [ms]
volatile long RED_BUTTON_PRESS_DURATION = 0;  // Stores duration that red button was held down [ms]

//////////      GLOBALS CHANGED IN MAIN LOOP  ////////////////////////////////////////////////////////
int START_MUSIC = 0;                          // Flags that music should start playing (1 to start) [binary]
int PLAYING_MUSIC = 0;                        // Flags whether music is actively playing (0 for no, 1 for yes) [binary]
int ROTARY_PRESS = 0;                         // Flags whether there is a rotary button press that needs handling [binary]
int MISSED_PACKET_COUNTED = 0;                // Flags whether a missed packet has been counted for this vacuum session [binary]

long NOTE_START_TIME = 0;                     // Marks the start time of the current note [ms]
int CURRENT_NOTE = 0;                         // Stores the index of NOTES[] to indicate which note is currently playing [int]
int STATE = 0;                                // Stores the state of the state machine [int]
int NEXT_STATE = 0;                           // Stores the state of the state machine to go to next [int]

int VACUUM_INNER_ON = 0;                      // Flag that stores whether inner vacuum is running [binary]
int VACUUM_OUTER_ON = 0;                      //    Same for outer
int CUP_SEALED_INNER = 0;                     // Stores whether the inner suction cup is sealed to the packet [binary]
int CUP_SEALED_OUTER = 0;                     //    Same for outer

long VACUUM_START_TIME_INNER = 0;             // Stores the time when the vacuum starts (whether or not the suction cup is grabbing the packet) [millis]
long VACUUM_START_TIME_OUTER = 0;             //    Same for outer
long SEAL_START_TIME_INNER = 0;               // Stores the time when the suction cup seals to the packet [millis]
long SEAL_START_TIME_OUTER = 0;               //    Same for outer

//////////      GLOBALS CHANGED BY USER       ///////////////////////////////////////////////////
// These are all stored to EEPROM so that they are saved when sensor is shut down
// They are initialized in setup

int PRESSURE_INNER_BASELINE = 0;		       // The pressure read by the inner sensor with no vacuum at all
int PRESSURE_OUTER_BASELINE = 0;		       // The pressure read by the outer sensor with no vacuum at all

int PRESSURE_INNER_LIMIT = 0;                 // The minimum pressure below which there is no sealed cup on the inner sensor [%]
int PRESSURE_OUTER_LIMIT = 0;                 // The minimum pressure below which there is no sealed cup on the outer sensor [%]
int UPPER_PRESSURE_BUFFER = 0;                // How much the upper pressure limit should be set above the displayed value for an open cup pulling a vacuum [%]
long MIN_VACUUM_TIME = 0;                     // The minimum time that a sucessful packet fill could take [ms]
int AUDIO_ALARM_ENABLED = 0;                  // 1 if user enables sound during the alarm, 0 if not
int OUTLET_ALARM_ENABLED = 0;                 // 1 if user enables electrical relay during the alarm, 0 if not

//////////      VARIABLES CHANGED BY I/Os     ///////////////////////////////////////////////////
int PRESSURE_INNER = 0;                       // Pressure read from the inner pressure sensor [%]
int PRESSURE_OUTER = 0;                       // Pressure read from the outer pressure sensor [%]
long VACUUM_TIME = 0;                         // Current time a vacuum has been held [ms]
int MISSED_PACKET_COUNT = 0;                  // Counts the number of missed packets (can be reset by user) [int]

/////////       DECLARE LCD AS GLOBAL         ///////////////////////////////////////////////////
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);           // Initialize the i2c address with 20 char x 4 row LCD as global

////////////////////////////////////////////////////////////////////////
//
//  SETUP   SETUP   SETUP   SETUP   SETUP   SETUP   SETUP   SETUP
//
////////////////////////////////////////////////////////////////////////

void setup() {

  // Initialize the LCD screen
  lcd.init();                             // initialize the display
  lcd.backlight();                        // turn on backlight
  lcd.clear();                            // clear LCD
  displayWelcomeScreen();                 // display welcome message

  /////////////////////////////// Init serial for testing //////////////////
  Serial.begin(9600);
  //////////////////////////////////////////////////////////////////////////

  // Pin definitions
  pinMode(led_pin, OUTPUT);
  pinMode(rotary_button_pin, INPUT_PULLUP);       // The circuit for the rotary pushbutton switch needs a pullup resistor so we'll use arduino's build in one
  pinMode(rotary_encoder_interrupt_pin, INPUT);
  pinMode(rotary_encoder_other_pin, INPUT);
  pinMode(relay_pin, OUTPUT);
  pinMode(pressure_pin_inner, INPUT);
  pinMode(pressure_pin_outer, INPUT);
  pinMode(speaker_pin, OUTPUT);

  // grab variable values from last time from EEPROM

  EEPROM.get(EEPROM_P_IN_BASE_ADDRESS, PRESSURE_INNER_BASELINE);
  EEPROM.get(EEPROM_P_OUT_BASE_ADDRESS, PRESSURE_OUTER_BASELINE);
  EEPROM.get(EEPROM_P_IN_ADDRESS, PRESSURE_INNER_LIMIT);
  EEPROM.get(EEPROM_P_OUT_ADDRESS, PRESSURE_OUTER_LIMIT);
  EEPROM.get(EEPROM_P_BUF_ADDRESS, UPPER_PRESSURE_BUFFER);
  EEPROM.get(EEPROM_TIME_ADDRESS, MIN_VACUUM_TIME);
  EEPROM.get(EEPROM_AUDIO_ADDRESS, AUDIO_ALARM_ENABLED);
  EEPROM.get(EEPROM_OUTLET_ADDRESS, OUTLET_ALARM_ENABLED);

  // Check those EEPROM values
  limitIOs();

  ////// EEPROM TESTING!!! ///////////////////////////     ---------------------------___
  /*
--->  Serial.print("Inner pressure off: ");
--->  Serial.println(PRESSURE_INNER_BASELINE);
--->  Serial.print("Outer pressure off: ");
--->  Serial.println(PRESSURE_OUTER_BASELINE);

  Serial.print("P_in: ");
  Serial.println(PRESSURE_INNER_LIMIT);
  Serial.print("P_out: ");
  Serial.println(PRESSURE_OUTER_LIMIT);
  Serial.print("Time: ");
  Serial.println(MIN_VACUUM_TIME);
  Serial.print("Audio? ");
  Serial.println(AUDIO_ALARM_ENABLED);
  Serial.print("Outlet? ");
  Serial.println(OUTLET_ALARM_ENABLED);*/

  ////////////////////////////////////////////////////////////////////////////////////////// ----^

  // Init interrupts for rotary encoder and red button
  attachInterrupt(digitalPinToInterrupt(rotary_encoder_interrupt_pin), rotary_interrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(red_button_interrupt_pin), red_button_interrupt, CHANGE);

  // Load welcome state
  STATE = 0;                              // set starting state of state machine
  displayMessageToLCD(STATE);             // have LCD display starting state message
  NEXT_STATE++;                           // Go to the next state

  ////////////////////// FOR Serial testing ///////////////////////////
  Serial.println("Start!");
  ////////////////////////////////////////////////////////////////////

}

////////////////////////////////////////////////////////////////////////
//
//  MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP
//
//  Algorithm:
//    1. Declares some timing variables
//    2. Checks the state of the state machine and updates it if necessary
//    3. Processes red button press (short and long)
//    4. Handles long red button presses (short presses are handled in interrupt handler)
//    5. Checks for rotary button push
//
////////////////////////////////////////////////////////////////////////

void loop() {
  // init timing variables (static variables are initialized once but carry over between runs of the function)
  static unsigned long lastMainLoopRotarySampleTime = 0;              // init timer for main loop to sample button on rotary encoder
  unsigned long thisMainLoopRotarySampleTime = millis();              // load current time
  static unsigned long lastMainLoopPressureSampleTime = 0;            // init timer for main loop to sample pressure sensors
  unsigned long thisMainLoopPressureSampleTime = millis();            // load current time
  static unsigned int rotaryButtonDown = 0;                           // init var that tracks if rotary button is currently down (0 for no, 1 for yes)

  // Change the state of the state machine if necessary
  if (NEXT_STATE != STATE) {          // if NEXT_STATE has been incremented
    updateEEPROM(STATE);              //  update the EEPROM appropriately based on current state
    if (NEXT_STATE > LAST_STATE) {    //  if NEXT_STATE is impossible
      NEXT_STATE = STATE_HOME;        //    set state to home state
    }
    STATE = NEXT_STATE;               //  set the current state to the next state
    displayMessageToLCD(STATE);       //  print the message corresponding to the current (ie. newly arrived at) state to the display
  }

  // if red button is currently pushed down, check to see if its a "long" push
  if (RED_BUTTON_ACTIVE) {
    long timePressed = millis() - RED_BUTTON_START_TIME;  // get the time that the button has been held down for
    if (timePressed >= RED_BUTTON_LONG_PRESS_LIMIT) {     // if time pressed exceeds the threshold for a "long" push
      RED_BUTTON_PRESS_DURATION = timePressed;            // store that duration
      RED_BUTTON_ACTIVE = 0;                              // and flag the button as no-longer-active
    }
  }

  // if red button has experienced a "long" press, reset the counter of missed packets
  if (RED_BUTTON_PRESS_DURATION) {                                      // if the red button has been pressed
    if (RED_BUTTON_PRESS_DURATION >= RED_BUTTON_LONG_PRESS_LIMIT) {     //  and if that press was longer than the limit for a "long" press
      MISSED_PACKET_COUNT = 0;                                          //      reset the missed packet count
      printMissedPacketCount();                                         //      update display accordingly
      Serial.println("LONG press registered");
    }
    RED_BUTTON_PRESS_DURATION = 0;                                      //  and reset the duration so that press will not be handled again
  }

  // Sample rotary encoder push-switch and detect presses (includes debouncing)
  // This next section of code raises the ROTARY_PRESS flag when a press has been detected
  if (thisMainLoopRotarySampleTime - lastMainLoopRotarySampleTime > ROTARY_BUTTON_SAMPLE_TIME) {    // runs code in conditional every ROTARY_BUTTON_SAMPLE_TIME milliseconds
    if (digitalRead(rotary_button_pin) == LOW) {      // If switch is pushed down
      if (!rotaryButtonDown) {                        //  if this is the first time we're picking up a press
        delay(DEBOUNCE_TIME);                         //    wait debounce time to ignore all bounces
        if (digitalRead(rotary_button_pin) == LOW) {  //    and if the button is ~still~ down
          rotaryButtonDown = 1;                       //      register a press!
        }
      }                                               // if the press has already been registered, do nothing
    } else {                                          // when the switch is not being pressed
      if (rotaryButtonDown) {                         //  if a press has been registered and not yet handled
        delay(DEBOUNCE_TIME);                         //    wait the debounce time
        if (digitalRead(rotary_button_pin) == HIGH) { //    if the switch is still not being pressed
          ROTARY_PRESS = 1;                           //      update global flag indicating there is a press to handle
          rotaryButtonDown = 0;                       //      reset the button tracker to indicate event is over
        }
      }
    }
    lastMainLoopRotarySampleTime = thisMainLoopRotarySampleTime;
  }

  // Sample pressure sensors and check for missed packets
  //
  // Algorithm:
  //    1. Read analog signals from pressure sensors
  //    2. Convert these signals to a percentage between 0 and 100
  //    3. Set flags to indicate whether the vacuums are on or off (regardless of whether the cups have sealed)
  //    3b. When both vacuums are off, reset the flag that indicates that a missed packet has been detected
  //        (this is to avoid double-counting the same packet as a miss)
  //    4. Set flags to indicate whether the cups have sealed
  //    5. If a vacuum is on but its corresponding cup has not sealed AND it has been on for:
  //        a. Longer than the time it takes to grab a packet / form a tight seal (global MAX_TIME_TO_SEAL) AND
  //        b. Shorter than the quickest time it takes to completely fill a packet (user-defined)
  //    6. Register this packet as a miss and trip the alarms!
  //
  if (thisMainLoopPressureSampleTime - lastMainLoopPressureSampleTime > PRESSURE_SAMPLE_TIME) {     // runs code in conditional every PRESSURE_SAMPLE_TIME milliseconds

    // Sample pressure sensors
    int p_inner = analogRead(pressure_pin_inner);   // Get analog pressure inner
    int p_outer = analogRead(pressure_pin_outer);   // Get analog pressure outer
    PRESSURE_INNER = rawPressure2pct(p_inner);      // Convert analog pressure inner to a percentage 0-100
    PRESSURE_OUTER = rawPressure2pct(p_outer);      // Convert analog pressure outer to a percentage 0-100
    printPressuresToLCD();                          // Update the display accordingly

	// Define maximum pressure below which we can still consider the vacuums to be entirely off as halfway between off and on without being sealed
  float maxPressureNoVacuumInner = PRESSURE_INNER_BASELINE + (((PRESSURE_INNER_LIMIT - UPPER_PRESSURE_BUFFER) - PRESSURE_INNER_BASELINE) / 2.);
  float maxPressureNoVacuumOuter = PRESSURE_OUTER_BASELINE + (((PRESSURE_OUTER_LIMIT - UPPER_PRESSURE_BUFFER) - PRESSURE_OUTER_BASELINE) / 2.);

	///////////////////////////// SERIAL TESTING ////////////////////////////////////////
  Serial.print("Pressure inner baseline: ");
  Serial.println(PRESSURE_INNER_BASELINE);
  Serial.print("Pressure outer baseline: ");
  Serial.println(PRESSURE_OUTER_BASELINE); 
  
  Serial.print("Pressure inner limit: ");
  Serial.println(PRESSURE_INNER_LIMIT);
  Serial.print("Pressure outer limit: ");
  Serial.println(PRESSURE_OUTER_LIMIT); 
  
  Serial.print("Inner pressure off LIMIT: ");
  Serial.println(maxPressureNoVacuumInner);
  Serial.print("Outer pressure off LIMIT: ");
  Serial.println(maxPressureNoVacuumOuter);
	/////////////////////////////////////////////////////////////////////////////////////



    // Check if the inner suction system is pulling a vacuum
    if (PRESSURE_INNER > maxPressureNoVacuumInner) {  // if vacuum is higher than atmospheric
      if (!VACUUM_INNER_ON) {                       //  and if vacuum was previously listed as off
        VACUUM_INNER_ON = 1;                        //    set vacuum flag to on
        VACUUM_START_TIME_INNER = millis();         //    and start vacuum timer
      }
      if (PRESSURE_INNER > PRESSURE_INNER_LIMIT) {  //  if pressure exceeds user-set limit for sealed packet
        if (!CUP_SEALED_INNER){                     //    if suction cup is listed as unsealed  ---------///////////////////// THIS CONDITIONAL LIKElY UNNECESSARY!!!!!
          CUP_SEALED_INNER = 1;                     //      set cup sealed flag   // [keep this line tho]
        }                                                                             /////////////////////////////////////////////////////////////////////////////////
      } else {                                      //  otherwise (if pressure is less than user-set limit)
        CUP_SEALED_INNER = 0;                       //    reset cup sealed flag
      }
    } else {                                        // otherwise (if there is no vacuum)
      VACUUM_INNER_ON = 0;                          //  set vacuum flag to off
      CUP_SEALED_INNER = 0;                         //  and set the cup sealed flag to off as well
    }

    // Do the same for the outer suction system
    if (PRESSURE_OUTER > maxPressureNoVacuumOuter) {  // if vacuum is higher than atmospheric
      if (!VACUUM_OUTER_ON) {                       //  and if vacuum is listed as off
        VACUUM_OUTER_ON = 1;                        //    set vacuum flag to on
        VACUUM_START_TIME_OUTER = millis();         //    and start vacuum timer
      }
      if (PRESSURE_OUTER > PRESSURE_OUTER_LIMIT) {  //  if pressure exceeds user-set limit for sealed packet
        if (!CUP_SEALED_OUTER){                     //    if suction cup is listed as unsealed
          CUP_SEALED_OUTER = 1;                     //      set cup sealed flag
        }
      } else {                                      //  otherwise (if pressure is less than user-set limit)
        CUP_SEALED_OUTER = 0;                       //    reset cup sealed flag
      }
    } else {                                        // otherwise (if there is no vacuum)
      VACUUM_OUTER_ON = 0;                          //  set vacuum flag to off
      CUP_SEALED_OUTER = 0;                         //  and set the cup sealed flag to off as well
    }

    if (PRESSURE_INNER <= maxPressureNoVacuumInner && PRESSURE_OUTER <= maxPressureNoVacuumOuter) {   // Once both vacuums shut off
      MISSED_PACKET_COUNTED = 0;                                                                  // Reset the flag for detecting a missed packet
    }

    // Check the pressure flags to determine if an alarm should start
    // First for the inner pressure
    long vacTime = 0;
    if (VACUUM_INNER_ON && !CUP_SEALED_INNER) {                         // if vacuum is active but suctions are not sealed
      vacTime = millis() - VACUUM_START_TIME_INNER;                     // grab the time the vacuum has been active
      if (vacTime > MAX_TIME_TO_SEAL && vacTime < MIN_VACUUM_TIME) {    // if that time is between the longest time it should take to seal and the shortest time it should take to pack
        if (!MISSED_PACKET_COUNTED) {                                   // and if no missed packets have been detected since this vacuum run started
          MISSED_PACKET_COUNTED = 1;                                    //  set the missed packet counted flag
          soundTheAlarms();                                             //  and start the alarm
        }
      }
    }

    // Then for the outer pressure
    if (VACUUM_OUTER_ON && !CUP_SEALED_OUTER) {                         // if vacuum is active but suctions are not sealed
      vacTime = millis() - VACUUM_START_TIME_OUTER;                     // grab the time the vacuum has been active
      if (vacTime > MAX_TIME_TO_SEAL && vacTime < MIN_VACUUM_TIME) {    // if that time is between the longest time it should take to seal and the shortest time it should take to pack
        if (!MISSED_PACKET_COUNTED) {                                   // and if no missed packets have been detected since this vacuum run started
          MISSED_PACKET_COUNTED = 1;                                    //  set the missed packet counted flag
          soundTheAlarms();                                             //  and start the alarm
        }
      }
    }
    lastMainLoopPressureSampleTime = thisMainLoopPressureSampleTime;    // Set last timer for pressure sampling to this one
  }

  // Check if the rotary encoder's momentary push-switch has been pressed. If so, handle
  if (ROTARY_PRESS) {       // if a press has been registered
      NEXT_STATE++;         //  go to the next state
      ROTARY_PRESS = 0;     //  reset the push-switch pressed flag

      ///////// For testing!!! ////////////
      Serial.println("there's been a rotary button press!");
      /////////////////////////////////////
  }

  // Check if music should be playing, if so, play!
  if (START_MUSIC || PLAYING_MUSIC) {
    playMusic();
  }

  // Check if the rotary encoder has been turned (one clockwise (+1) or widdershins (-1) turn) and handle turn based on state
  if (ROTATION) {                                                                   		// If a rotation (+1 or -1 has been flagged)

    if (STATE == STATE_P_IN_BASELINE) { 													//  If the inner pressure baseline is currently editable
      PRESSURE_INNER_BASELINE = PRESSURE_INNER_BASELINE + ROTATION * INCREMENT_PRESSURE;   	//    Adjust the global variable accordingly
 	  } else if (STATE == STATE_P_OUT_BASELINE) { 											//  If the outer pressure baseline is currently editable
 	    PRESSURE_OUTER_BASELINE = PRESSURE_OUTER_BASELINE + ROTATION * INCREMENT_PRESSURE;   	//    Adjust the global variable accordingly
    } else if (STATE == STATE_P_IN) {                                               	//  If the inner pressure limit is currently editable
      PRESSURE_INNER_LIMIT = PRESSURE_INNER_LIMIT + ROTATION * INCREMENT_PRESSURE;  		//    Adjust the global variable accordingly
    } else if  (STATE == STATE_P_OUT) {                                             		//  If the outer pressure limit is currently editable
      PRESSURE_OUTER_LIMIT = PRESSURE_OUTER_LIMIT + ROTATION * INCREMENT_PRESSURE;  		//    Adjust the global accordingly
    } else if (STATE == STATE_P_BUF) {
      UPPER_PRESSURE_BUFFER = UPPER_PRESSURE_BUFFER + ROTATION * INCREMENT_PRESSURE;
    } else if  (STATE == STATE_TIME) {                                              		// Etc.
      MIN_VACUUM_TIME = MIN_VACUUM_TIME + ROTATION * INCREMENT_TIME;
    } else if  (STATE == STATE_AUDIO) {
      AUDIO_ALARM_ENABLED = AUDIO_ALARM_ENABLED + ROTATION;
    } else if  (STATE == STATE_OUTLET) {
      OUTLET_ALARM_ENABLED = OUTLET_ALARM_ENABLED + ROTATION;
    }
    limitIOs();                                                                     // After adjusting appropriate GLOBALS, check to make sure none exceed their bounds
    printVariable(STATE);                                                           // Print the new variable's value to the LCD
    ROTATION = 0;                                                                   // Reset the rotary encoder flag so the next turn will be detectable

    //////////// FOR TESTING!!! ///////////////
    Serial.println(ROTATION);
    ///////////////////////////////////////////

  }
}



////////////////////////////////////////////////////////////////////////
//
//  INTERRUPTS        INTERRUPTS      INTERRUPTS      INTERRUPTS
//
////////////////////////////////////////////////////////////////////////

void rotary_interrupt() {                                                   // (This interrupts when one of the rotary encoder pins goes LOW)
  static unsigned long lastRotaryInterruptTime = 0;                         // Get time of last rotary interrupt (static vars only init once and are then passed between function runs)
  unsigned long thisRotaryInterruptTime = millis();                         // Get current time
  if (thisRotaryInterruptTime - lastRotaryInterruptTime > DEBOUNCE_TIME) {  // If it has been longer than the debounce period between the last interrupt and this one
    if (!digitalRead(rotary_encoder_other_pin)) {                           //  If the other rotary encoder pin is also LOW
      ROTATION = 1;                                                         //    Flag a clockwise rotation
    } else {                                                                //  If the otehr rotary encoder pin is HIGH
      ROTATION = -1;                                                        //    Flag a widdershins rotation
    }
  }
  lastRotaryInterruptTime = thisRotaryInterruptTime;                        // Set last interrupt time to time of this interrupt
}

void red_button_interrupt() {                           // (This interrupts when signal pin is CHANGED [goes to HIGH or to LOW])
  if (digitalRead(red_button_interrupt_pin) == LOW) {   // if button has just been pressed down
    RED_BUTTON_ACTIVE = 1;                              //  flag that it is down
    RED_BUTTON_START_TIME = millis();                   //  store the current time
    RED_BUTTON_PRESS_DURATION = 0;                      //  and set the duration of the press to 0
  } else {                                              // otherwise, the button has just been released so
    RED_BUTTON_ACTIVE = 0;                              //  flag that it is no longer down
    long duration = millis() - RED_BUTTON_START_TIME;   //  calculate the duration for which it was held down
    if (duration < RED_BUTTON_LONG_PRESS_LIMIT) {       //  if the duration is shorter than the limit for a "long" press
      quietTheAlarms();                                 //    consider it a short button press and quiet the alarms and
      RED_BUTTON_PRESS_DURATION = 0;                    //    reset the DURATION global

      ///////////// FOR TESTING!!! /////////////
      Serial.println("short press registered.");
      //////////////////////////////////////////
    }
  }
}

////////////////////////////////////////////////////////////////////////
//
//  CALLABLE FUNCTIONS        CALLABLE FUNCITONS      CALLABLE FUNCTIONS
//
////////////////////////////////////////////////////////////////////////

void soundTheAlarms() {             // This fn is called once for each missed packet
  MISSED_PACKET_COUNT++;            // increment the MISSED_PACKET_COUNT
  printMissedPacketCount();         // And update LCD accordingly
  if (AUDIO_ALARM_ENABLED) {        // If audio alarm is enabled by the user
    START_MUSIC = 1;                //  Set the start music flag
  }
  if (OUTLET_ALARM_ENABLED) {       // If the electrical outlet alarm is enabled by user
    digitalWrite(relay_pin, HIGH);  //  Send power to the outlet!
  }

  // FOR TESTING!!!!
  digitalWrite(13, HIGH);   // Light arduino pin13 yellow LED
  ///////////////////////
}

void quietTheAlarms() {
  PLAYING_MUSIC = 0;                // Reset PLAYING_MUSIC flag so no new tones play
  noTone(speaker_pin);              // Silence any tones that may be playing
  digitalWrite(relay_pin, LOW);     // Cut power from the electrical outlet alarms

  // FOR TESTING //////////
  digitalWrite(13,LOW);             // Turn off arduino pin13 yellow LED
  //////////////////////
}

void playMusic() {
  if (AUDIO_ALARM_ENABLED) {
    int dur = 0;
    if (START_MUSIC) {                                // if this is the start of the song
      START_MUSIC = 0;                                //  reset the START_MUSIC flag because we're starting!
      PLAYING_MUSIC = 1;                              //  set the PLAYING_MUSIC flag
      CURRENT_NOTE = 0;                               //  Set the CURRENT_NOTE to the first note in the melody
      NOTE_START_TIME = millis();                     //  Set the NOTE_START_TIME to the current arduino time
      //dur = getNoteDuration(CURRENT_NOTE);
      tone(speaker_pin, NOTES[CURRENT_NOTE]);    //  and start playing the first note!
      Serial.println("started playing!");
    }

    long noteTime = millis() - NOTE_START_TIME;                    //  Calculate the duration of the note thus far
    dur = getNoteDuration(CURRENT_NOTE);                      // Calculate desired duration of current note
    if (noteTime >= dur) {                                    // If the current note has been playing for longer than its duration
      if (CURRENT_NOTE >= sizeof(NOTES)/sizeof(NOTES[0])-1) {   //   If its the last note,
        CURRENT_NOTE = 0;                                     //     Go back to the beginning!
      } else {                                                //  Otherwise,
        CURRENT_NOTE++;                                       //    Go to the next note!
      }
      NOTE_START_TIME = millis();                                  // Set the NOTE_TIMER to the current time
      noTone(speaker_pin);                                    // Stop the current tone
      tone(speaker_pin, NOTES[CURRENT_NOTE]);                        // Start the next tone
    }
  } else {
    noTone(speaker_pin);
  }





  /*int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
  int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4};
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    int noteDuration = 1000 / noteDurations[thisNote];
    tone(speaker_pin, melody[thisNote], noteDuration);
    //int pauseBetweenNotes = noteDuration * 1.30;
    //delay(pauseBetweenNotes);
    //noTone(speaker_pin);
  }*/

  //tone(speaker_pin, NOTE_C4, 5000);
  //delay(5000);
  //noTone(speaker_pin);
}

/////////////////////////////////////////////////////
//
//    fn  getNoteDuration(i)
//
//  Takes in the argument 'i', an integer index for the current note on the array DURATIONS[]
//  Returns the milliseconds for which a note should be played
//  Algorithm: DURATIONS holds numbers representing the denominator of the note fraction
//    for example, a quarter (1/4) note is represented with a 4. An eigth note with an 8, etc.
//    This function divides the measure lenght in [ms] by the denominator.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

long getNoteDuration(int i) {
  return MEASURE_DURATION / NOTE_DURATIONS[i];
}

/////////////////////////////////////////////////////////////////
//
// fn rawPressure2pct
// converts a raw pressure value from the analog pins to a percentage
// accepts a float n
// multiplies it by a multiplier to convert the raw value to a voltage between 1 and 5
// scales to map that range to 0-100%
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////

float rawPressure2pct(float n) {
  n = n*ANALOG_TO_V_MULTIPLIER;
  return 25*(n - 1);
}

///////////////////////////////////////////////
//
// fn updateEEPROM(i)
// takes in a state in the state machine
// pushes the appropriate variable to EEPROM
//
/////////////////////////////////////////////////////////////////////////////////////////

void updateEEPROM(int state) {

  if (state == STATE_P_IN_BASELINE) {
    EEPROM.put(EEPROM_P_IN_BASE_ADDRESS, PRESSURE_INNER_BASELINE);
  } else if (state == STATE_P_OUT_BASELINE) {
    EEPROM.put(EEPROM_P_OUT_BASE_ADDRESS, PRESSURE_OUTER_BASELINE);
  } else if (state == STATE_P_IN) {
    EEPROM.put(EEPROM_P_IN_ADDRESS, PRESSURE_INNER_LIMIT);
  } else if (state == STATE_P_OUT) {
    EEPROM.put(EEPROM_P_OUT_ADDRESS, PRESSURE_OUTER_LIMIT);
  } else if (state == STATE_P_BUF) {
    EEPROM.put(EEPROM_P_BUF_ADDRESS, UPPER_PRESSURE_BUFFER);
  } else if (state == STATE_TIME) {
    EEPROM.put(EEPROM_TIME_ADDRESS, MIN_VACUUM_TIME);
  } else if (state == STATE_AUDIO) {
    EEPROM.put(EEPROM_AUDIO_ADDRESS, AUDIO_ALARM_ENABLED);
  } else if (state == STATE_OUTLET) {
    EEPROM.put(EEPROM_OUTLET_ADDRESS, OUTLET_ALARM_ENABLED);
  }
}

///////////////////////////////////////////////
//
// fn limitIOs()
// Checks the five user inputs. If an input is above the valid range, sets it to the max
//    if an input is below the valid range, sets i to the min
//
/////////////////////////////////////////////////////////////////////////////////////////

void limitIOs() {

  if (PRESSURE_INNER_BASELINE > 100) {PRESSURE_INNER_BASELINE = 100;}
  else if (PRESSURE_INNER_BASELINE < 0) {PRESSURE_INNER_BASELINE = 0;}
  if (PRESSURE_OUTER_BASELINE > 100) {PRESSURE_OUTER_BASELINE = 100;}
  else if (PRESSURE_OUTER_BASELINE < 0) {PRESSURE_OUTER_BASELINE = 0;}
  if (PRESSURE_INNER_LIMIT - UPPER_PRESSURE_BUFFER > 100) {PRESSURE_INNER_LIMIT = 100 + UPPER_PRESSURE_BUFFER;}
  else if (PRESSURE_INNER_LIMIT - UPPER_PRESSURE_BUFFER < 0) {PRESSURE_INNER_LIMIT = UPPER_PRESSURE_BUFFER;}
  if (PRESSURE_OUTER_LIMIT - UPPER_PRESSURE_BUFFER > 100) {PRESSURE_OUTER_LIMIT = 100 + UPPER_PRESSURE_BUFFER;}
  else if (PRESSURE_OUTER_LIMIT - UPPER_PRESSURE_BUFFER < 0) {PRESSURE_OUTER_LIMIT = UPPER_PRESSURE_BUFFER;}
  if (UPPER_PRESSURE_BUFFER > 50) {UPPER_PRESSURE_BUFFER = 50;}
  else if (UPPER_PRESSURE_BUFFER < 0) {UPPER_PRESSURE_BUFFER = 0;}
  if (MIN_VACUUM_TIME > 9900) {MIN_VACUUM_TIME = 9900;}
  else if (MIN_VACUUM_TIME < 0) {MIN_VACUUM_TIME = 0;}
  if (AUDIO_ALARM_ENABLED > 1) {AUDIO_ALARM_ENABLED = 1;}
  else if (AUDIO_ALARM_ENABLED < 0) {AUDIO_ALARM_ENABLED = 0;}
  if (OUTLET_ALARM_ENABLED > 1) {OUTLET_ALARM_ENABLED = 1;}
  else if (OUTLET_ALARM_ENABLED < 0) {OUTLET_ALARM_ENABLED = 0;}
}

///////////////////////////////////////////////
//
// fn displayMessageToLCD()
// Takes in a state
// Based on the state, prints the appropriate screen to the LCD
//
/////////////////////////////////////////////////////////////////////////////////////////


void displayMessageToLCD(int state) {
  //lcd.clear();
  if (state == STATE_INIT) {                // on the init state
    lcd.noBlink();                          //  disable blinking cursor
    displayWelcomeScreen();                 //  display welcome screen
    delay(3000);                            //    and hold it for a bit
    displayCheckVarsMessage();              //  display check variables message
    delay(5000);                            //    and hold it for a bit
    displayReadyMessage();                  //  display ready message
    delay(3000);                            //  and hold it for a bit
  } else if (state == STATE_HOME) {         // on the home state
    displayHomeScreen();                    //  display the home screen
    lcd.noBlink();                          //  disable blinking cursor
  } else if (state == STATE_SETTINGS_1) {   // on first settings page
    displaySettingScreen1();                //  display first settings page
    lcd.noBlink();                          //  disable blinking cursor
  } else if (state == STATE_SETTINGS_2) {   // on 2nd settings page
    displaySettingScreen2();                //  display 2nd settings page
    lcd.noBlink();                          // disable blinking cursor
 	  NEXT_STATE++;						// advance to next state
  } else if (state == STATE_SETTINGS_3) {   // on 3rd settings page
    displaySettingScreen3();                //  display 2nd settings page
    lcd.noBlink();                          // disable blinking cursor
 	  NEXT_STATE++;						// advance to next state
  } else {//if (state == STATE_P_IN || state == STATE_P_OUT || state == STATE_P_BUF || state == STATE_TIME || state == STATE_AUDIO || state == STATE_OUTLET ) {  // on any edit-settings page
    printVariable(state);                   // print the editable variable (this also sets cursor to var's start)
    lcd.blink();                            // enable blinking cursor
  }
}

///////////////////////////////////////////////
//        display[__________]() FUNCTION GROUP
// fn displayWelcomeScreen()
// fn displayCheckVarsMessage()
// fn displayReadyMessage()
// fn displayHomeScreen()
// fn displaySettingScreen1()
// fn displaySettingScreen2()
//
// Each of these functions clear the LCD then display a preset message
// To display a variable to the LCD, they call the printVariable() function.
//
/////////////////////////////////////////////////////////////////////////////////////////

void displayWelcomeScreen() {
  lcd.clear();
  lcd.print("     welcome! ");
  lcd.setCursor(0 ,2);
  lcd.print("...system loading...");
  lcd.setCursor(19,3);
}

void displayCheckVarsMessage() {
  lcd.clear();
  lcd.print(" *   Please check   ");
  lcd.setCursor(0,1);
  lcd.print(" settings before    ");
  lcd.setCursor(0,2);
  lcd.print(" starting run     * ");
}

void displayReadyMessage() {
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("       READY!");
  lcd.setCursor(0,3);
  lcd.print("  >:0  =D  :(   =)  ");
}

void displayHomeScreen() {
  lcd.clear();
  lcd.print("Inner pressure: ");
  lcd.print(int2str(PRESSURE_INNER));
  lcd.print("% ");
  lcd.setCursor(0,1);
  lcd.print("Outer pressure: ");
  lcd.print(int2str(PRESSURE_OUTER));
  lcd.print("%");
  lcd.setCursor(0,2);
  lcd.print("--  --  ----  --  --");
  lcd.setCursor(0,3);
  lcd.print(" MISSED PACKS: ");
  printMissedPacketCount();
}

void displaySettingScreen1() {
  lcd.clear();
  lcd.print("  Adjust settings");

  lcd.setCursor(0,1);
  lcd.print("Fastest pack:   ");
  printVariable(STATE_TIME);
  lcd.setCursor(19,1);
  lcd.print("s");

  lcd.setCursor(0,2);
  lcd.print("Inner P. off:   ");
  printVariable(STATE_P_IN_BASELINE);
  lcd.setCursor(19,2);
  lcd.print("%");

  lcd.setCursor(0,3);
  lcd.print("Outer P. off:   ");
  printVariable(STATE_P_OUT_BASELINE);
  lcd.setCursor(19,3);
  lcd.print("%");
}

void displaySettingScreen2() {
  lcd.clear();
  lcd.print("  Adjust settings");
  lcd.setCursor(0,1);
  lcd.print("Inner P. open:  ");
  printVariable(STATE_P_IN);
  lcd.setCursor(19,1);
  lcd.print("%");
  lcd.setCursor(0,2);
  lcd.print("Outer P. open:  ");
  printVariable(STATE_P_OUT);
  lcd.setCursor(19,2);
  lcd.print("%");
  lcd.setCursor(0,3);
  lcd.print("Pressure buffer: ");
  printVariable(STATE_P_BUF);
  lcd.setCursor(19,3);
  lcd.print("%");
}

void displaySettingScreen3() {
  lcd.clear();
  lcd.print("  Adjust settings");
  lcd.setCursor(0,1);
  lcd.print("Audio alarm: ");
  printVariable(STATE_AUDIO);
  lcd.setCursor(0,2);
  lcd.print("Outlet alarm: ");
  printVariable(STATE_OUTLET);
}

///////////////////////////////////////////////
//
// fn printVariable()
// Takes in a state
// Based on the state, prints the appropriate variable the the appropriate loaction
//  on the LCD. Sets cursor to the first character in that printed variable
// LIMITATIONS: Doesn't work for STATE_HOME because multiple variables can change. They
//              each have their own function below. See:
//                    - printPressuresToLCD()
//                    - printMissedPacketCount()
//
/////////////////////////////////////////////////////////////////////////////////////////

void printVariable(int state) {
  int home_col = 0;
  int home_row = 0;
  int size = 0;

  if (state == STATE_P_IN_BASELINE) {             // If the inner pressure baseline can be changed:
    home_col = 16;
    home_row = 2;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-digit number would go
    lcd.print(int2str(PRESSURE_INNER_BASELINE));  //  Print inner pressure limit
    size = String(PRESSURE_INNER_BASELINE).length(); //  get length of inner pressure limit
    lcd.setCursor(home_col+(3-size), home_row);   //  adjust cursor position accordingly so it stays on largest digit
  } else if (state == STATE_P_OUT_BASELINE) {	  // If the outer pressure baseline can be changed:
    home_col = 16;
    home_row = 3;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-digit number would go
    lcd.print(int2str(PRESSURE_OUTER_BASELINE));  //  Print outer pressure limit
    size = String(PRESSURE_OUTER_BASELINE).length(); //  get length of outer pressure limit
    lcd.setCursor(home_col+(3-size), home_row);   //  adjust cursor position accordingly so it stays on largest digit
  } else if (state == STATE_P_IN) {                      // If the inner pressure limit can be changed:
    home_col = 16;
    home_row = 1;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-digit number would go
    lcd.print(int2str(PRESSURE_INNER_LIMIT-UPPER_PRESSURE_BUFFER));     //  Print inner pressure limit
    size = String(PRESSURE_INNER_LIMIT-UPPER_PRESSURE_BUFFER).length(); //  get length of inner pressure limit
    lcd.setCursor(home_col+(3-size), home_row);   //  adjust cursor position accordingly so it stays on largest digit
  } else if (state == STATE_P_OUT) {              // If the outer pressure limit can be changed:
    home_col = 16;
    home_row = 2;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-digit number would go
    lcd.print(int2str(PRESSURE_OUTER_LIMIT-UPPER_PRESSURE_BUFFER));     //  Print inner pressure limit
    size = String(PRESSURE_OUTER_LIMIT-UPPER_PRESSURE_BUFFER).length(); //  get length of inner pressure limit
    lcd.setCursor(home_col+(3-size), home_row);   //  adjust cursor position accordingly so it stays on largest digit
  } else if (state == STATE_P_BUF) {
    home_col = 16;
    home_row = 3;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-digit number would go
    lcd.print(int2str(UPPER_PRESSURE_BUFFER));     //  Print inner pressure limit
    size = String(UPPER_PRESSURE_BUFFER).length(); //  get length of inner pressure limit
    lcd.setCursor(home_col+(3-size), home_row);   //  adjust cursor position accordingly so it stays on largest digit
  } else if (state == STATE_TIME) {               // If the time limit can be changed:
    home_col = 16;
    home_row = 1;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-digit number would go
    lcd.print(millis2str(MIN_VACUUM_TIME));       //  Print time limit
    lcd.setCursor(home_col, home_row);            //  Reset cursor to start of time limit
  } else if (state == STATE_AUDIO) {              // If AUDIO_ALARM_ENABLED can be changed:
    home_col = 13;
    home_row = 1;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-letter word would go
    lcd.print(binary2str(AUDIO_ALARM_ENABLED));   //  Print ON/OFF
    lcd.setCursor(home_col, home_row);            //  Reset cursor to start of ON/OFF
  } else if (state == STATE_OUTLET) {             // If OUTLET_ALARM_ENABLED can be changed:
    home_col = 14;
    int home_row = 2;
    lcd.setCursor(home_col, home_row);            //  Set cursor to beginning of where 3-letter word would go
    lcd.print(binary2str(OUTLET_ALARM_ENABLED));  //  Print ON/OFF
    lcd.setCursor(home_col, home_row);            //  Reset cursor to start of ON/OFF
  }
}

///////////////////////////////////////////////
//
// fn printPressuresToLCD()
//  if the state is home, prints the two pressure sensor values to the appropriate locations
//
/////////////////////////////////////////////////////////////////////////////////////////

void printPressuresToLCD() {
  if (STATE == STATE_HOME) {
    lcd.setCursor(16,0);
    lcd.print(int2str(PRESSURE_INNER));
    lcd.setCursor(16,1);
    lcd.print(int2str(PRESSURE_OUTER));

  }
}

///////////////////////////////////////////////
//
// fn printMissedPacketCount()
//  if the state is home, prints the missed packet count to the appropriate location
//
/////////////////////////////////////////////////////////////////////////////////////////

void printMissedPacketCount() {
  if (STATE == STATE_HOME) {
    lcd.setCursor(15,3);
    lcd.print(int2str(MISSED_PACKET_COUNT));
  }
}

///////////////////////////////////////////////////
//                  [___]2str FUNCTION GROUP
//  fn int2str
//  takes in an integer between 0-999
//  returns a string that is padded with spaces such that the 1s digit is always in the 3rd position,
//    the 10s is always in the 2nd, and the hundreds is always in the first. For example, if given
//    the integer 7, it will return "  7". If given 41, it will return " 41" and if given 222 it will
//    return "222".
//
//  fn millis2str
//  takes in a time in milliseconds between 0 and 9900 (type: long)
//  returns a decimal string with first two digits and rest of decimal truncated
//    (eg. 0 ->  "0.0", 4321 -> "4.3" or 9900 -> "9.9" )
//
//  fn binary2str
//  takes in a binary int (either 0 or 1)
//  if flag is false (0), returns string "OFF" otherwise returns string "ON"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String int2str(int num) {
  String str = String(num);   // cast num as a string
  for (int i=2; i<=3; i++) {  // loop through tens and hundreds places
    if (i>str.length()) {     // if there is nothing there
      str = " "+str;          // prepend a space
    }
  }
  return str;
}

String millis2str(long ms) {
  float f = ms / 1000.;             // divide by 1000 to get a float in units of seconds
  return String(f).substring(0,3);  // cast float as string and truncate before character index 3
}

String binary2str(int flag) {
  if (flag == 0) {
    return "OFF";
  } else {
    return "ON ";
  }
}

