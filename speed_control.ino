/*
 MC-2100 Treadmill Motor Controller Interface
 Lathe Motor Controller via PWM
 Seven Segment Tachometer
 ON/OFF Toggle
 
 Joe Schoolcraft
 Brian Schoolcraft
 May 2013
 https://sonsofinvention.wordpress.com/
*/
#include <TimerOne.h>
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include <Rotary.h>
#include "Adafruit_LEDBackpack.h"

#define SPEED_SENSE 2 //connected to reed switch - magnet closes switch, pulls to ground (Spindle Speed Input - 2 pulses per rev)
#define PWM_OUT 10 //Connected to blue wire of MC2100 (50ms period PWM out)
#define REV_OUT 1 //Reverse Relay Output (Not Used)
#define FWD_OUT 0 //Forward Relay Output (Not Used)
//#define FWD_REV 3 //Fwd/Rev Switch Input (Not Used)
#define ON_OFF 9 //On/Off Switch Input
 
#define PWM_CYCLE 50.0 //Output Signal PWM Period (50ms)
#define MAX_DUTY 869 //Max Duty Cycle expected by MC-2100 (85% of 1023)
#define MIN_DUTY 77 //Min Duty Cycle expected by MC-2100 (0% of 1023)
#define REED_DELAY 20 //Deboounce time for reed switch
#define RESET_DELAY 1000 //Time delay before resetting speed to 0
#define NUM_SAMPLES 50 //Number of Samples to average before smoothing function
#define SMOOTH_LEVEL .8 //Input to smoothing function
#define TO_LOW_DELAY 50 //Debounce time for HI to LO switch transition
#define TO_HIGH_DELAY 50 //Debounce time for LO to HI switch transition
 
long intervals[NUM_SAMPLES] = {0,0}; //Last NUM_SAMPLES reedInterval values for averaging spindle speed
long intervalSum; //Sum of intervals vector
int avgInterval; //Averaged interval value
int spindleSpeed = 0; //Spindle speed converted from avgInterval and smoothed, used for tachometer calculation
int rawSpindleSpeed = 0; //Spindle speed converted from avgInterval, used for tachometer calculation
int intervalCount = 0;
int initialSpeedLevel = 77; // we want to start the motor with enough power to actually turn
int motor_speed_set_point = 77;
bool motor_on = 0;
int oldButtonState = LOW;

unsigned long lastFwdRevTime = 0;
unsigned long lastOnOffTime = 0;
byte reading = 0;
byte fwdRevState = 0;
byte buttonState = 0;
byte lastfwdRevState = 0;
byte lastonOffState = 0;
byte pendingDecel = 0;
 
//ISR Variables stored as volatile (kept in RAM)
volatile unsigned long reedTime = 0; //Last time reed switch closed
volatile unsigned long reedInterval; //Time since last reed switch closure
volatile boolean reedSet = 0; //Reed switch tracker

Adafruit_7segment matrix = Adafruit_7segment();
Rotary r = Rotary(3, 4);
 
void setup()
{
    Serial.begin(115200);
    pinMode(PWM_OUT, OUTPUT);
    pinMode(SPEED_SENSE,INPUT);

    pinMode(ON_OFF, INPUT_PULLUP); //Enable internal pullup resistor to simplify external circuit

    Timer1.initialize(PWM_CYCLE*1000); //Set pin 9 and 10 period to 50 ms
    Timer1.pwm(PWM_OUT, 0); //Start PWM at 0% duty cycle

    matrix.begin(0x70);
}
 
//Interrupt Service Routine - Store time interval from reed switch each magnet pulse 
void intervalCalc() {  
    if ((millis() - reedTime) >= REED_DELAY) 
    { //ignore switch bounce for (REED_DELAY) ms (not the normal debounce routine)
	   if (reedTime !=0) { //Is this the first interrupt since reset?
		  reedSet = 1; //set flag
		  reedInterval = millis() - reedTime; //calculate interval
	   }
	   reedTime = millis(); //record last interrupt time
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
/* Function for debouncing digital inputs
 
 Arguments:
 _debouncePin - ID of pin to be read/debounced
 *lastReading - pointer to variable storing the previous reading (HIGH/LOW) of the input pin
 *lastDebounceTime - pointer to variable storing the last time (ms) the input changed (not debounced)
 _toLowDelay - debounce time for HIGH to LOW transition
 _toHighDelay - debounce time for LOW to HIGH transition
 
 Returns:
 _state - debounced state (HIGH/LOW) of _debouncePin
 */
////////////////////////////////////////////////////////////////////////////////////////////
 
byte debounce(byte _debouncePin, byte *lastReading, unsigned long *lastDebounceTime, int _toLowDelay, int _toHighDelay)
{
 byte _reading = digitalRead(_debouncePin);
 byte _state = *lastReading;
 
if (_reading != *lastReading) { // pin state just changed
 *lastDebounceTime = millis(); // reset the debouncing timer
 }
 
if ((millis() - *lastDebounceTime) >= _toLowDelay && _reading == LOW) {
 // whatever the reading is at, it's been there for longer
 // than the hold delay, so take it as the actual current state for use in the rest of the script
 _state = _reading;
 *lastReading = _reading;
 return _state;
 }
 
if ((millis() - *lastDebounceTime) >= _toHighDelay && _reading == HIGH){
 // whatever the reading is at, it's been there for longer
 // than the hold delay, so take it as the actual current state for use in the rest of the script
 _state = _reading;
 *lastReading = _reading;
 return _state;
 }
 *lastReading = _reading;
 return _state;
}

///Function for smoothing sensor readings: http://playground.arduino.cc//Main/Smooth
int smooth(int data, float filterVal, int smoothedVal){
 
if (filterVal > 1){ // check to make sure param's are within range
 filterVal = .99;
 }
 else if (filterVal <= 0){
 filterVal = 0;
 }
 
smoothedVal = (data * (1 - filterVal)) + (smoothedVal * filterVal);

 
return (int)smoothedVal;
}


void loop() {

    attachInterrupt(0, intervalCalc, FALLING); //Attach interrupt to pin 2 for spindle speed sensing

    int result = r.process();
    if (result) {
        if (result == DIR_CW ) {
            motor_speed_set_point += 10;
        } else {
            motor_speed_set_point -= 10;
        }
    }
    Serial.println(motor_speed_set_point);

    initialSpeedLevel = map(motor_speed_set_point,MIN_DUTY,1023,MIN_DUTY,MAX_DUTY); //Convert Pot input to pwm level to send to MC-2100


    buttonState = debounce(ON_OFF, &lastonOffState, &lastOnOffTime, TO_LOW_DELAY, TO_HIGH_DELAY);


    // Get the current state of the button

    // Has the button gone high since we last read it?
    if (buttonState == HIGH && oldButtonState == LOW) {

        if (motor_on == 0) {
            // Toggle on
            matrix.blinkRate(0); // Don't blink display
            motor_on = 1;

        } else {
            // Toggle off
            Timer1.setPwmDuty(PWM_OUT, 0); // Shut down MC-2100
            matrix.blinkRate(2); // Blink the display
            motor_on = 0;
        }
    }

    if (motor_on) {
        Timer1.setPwmDuty(PWM_OUT, initialSpeedLevel); //Send speed command to MC-2100
    }

  // Store the button's state so we can tell if it's changed next time round
  oldButtonState = buttonState;


////////////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////Relays/Braking not implemented at this time
 // //Set Relay States and MC-2100 PWM Levels
 //
 // if (fwdRevState != lastfwdRevState){ //The direction switch has been flipped
 // pendingDecel = HIGH; //Set decel flag. This gets reset low once the spindle stops
 // }
 //
 // if (buttonState == LOW){ //Off
 // digitalWrite(FWD_OUT, LOW); //Set relays for braking
 // digitalWrite(REV_OUT, LOW);
 // Timer1.setPwmDuty(PWM_OUT, 0); //Shut down MC-2100
 // }
 //
 // if (buttonState == HIGH){ //On
 // if (pendingDecel == LOW){
 // if (fwdRevState == HIGH){ //Fwd
 // digitalWrite(FWD_OUT, HIGH); //Set relays for FWD operation
 // digitalWrite(REV_OUT, LOW);
 // Timer1.setPwmDuty(PWM_OUT, initialSpeedLevel); //Send speed command to MC-2100
 // }
 // if (fwdRevState == LOW){ //Rev
 // digitalWrite(FWD_OUT, LOW); //Set relays for REV operation
 // digitalWrite(REV_OUT, HIGH);
 // Timer1.setPwmDuty(PWM_OUT, initialSpeedLevel); //Send speed command to MC-2100
 // }
 // }
 // else{
 // digitalWrite(FWD_OUT, LOW); //Set relays for braking
 // digitalWrite(REV_OUT, LOW);
 // Timer1.setPwmDuty(PWM_OUT, 0); //Shut down MC-2100
 // }
 // }
 ////////////////////////////////////////////////////////////////////////////////////////////
 
 //Calculate RPM from interrupt routine output, display on 7 segment
 
if (reedSet == 1) { //Interrupt triggered since last loop, calculate RPM
    reedSet = 0; //reset interrupt flag
    intervalCount++; //increment counter (used for averaging first few pulses)

    //Store interval from ISR in vector for averaging
    for( int n=NUM_SAMPLES-1; n>0; n-- ) {
	   intervals[n]=intervals[n-1]; //Shift all values to the right. Oldest value discarded
    }
    intervals[0] = constrain(reedInterval,0,RESET_DELAY); //new value at beginning of vector

    //Caculate RPM
    intervalSum = 0; //reset sum value
    for( int n=0; n<NUM_SAMPLES; n++ ) {  
	   intervalSum += intervals[n]; //add intervals together  
    } 

    avgInterval = intervalSum/min(intervalCount,NUM_SAMPLES); //calculate average interval 
    rawSpindleSpeed = 60000/(avgInterval); //convert to RPM  
    spindleSpeed = smooth(rawSpindleSpeed, SMOOTH_LEVEL, spindleSpeed); //smooth RPM reading 
    matrix.println(spindleSpeed);
    matrix.writeDisplay();
} 

if( (millis() - reedTime) >= RESET_DELAY) { //If no pulses received for (RESET_DELAY) ms, reset all counters/intervals
    for(int n=0; n<NUM_SAMPLES; n++) 
    {   
        intervals[n] = 0;  
        intervalCount = 0;  
        reedTime = 0;  
        spindleSpeed = 0;  
        pendingDecel = LOW; //The spindle has stopped, OK to reverse direction. (Not used yet)  
    }  
    matrix.println(0);
    matrix.writeDisplay();
} 

} //////end loop //////////////////////////////////////////////////////////////////////////////////////////// 
