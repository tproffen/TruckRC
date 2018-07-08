/*------------------------------------------------------------------------------
 * BLE Truck Controller
 * (c) Thomas Proffen, 2018
 *
 * Commands 
 *     0x01 - Operate lights, Parameters: light, status
 *     0x02 - Set gear, Parameters: gear
 *     0x03 - Drive, Parameters: stear, throttle
 *     0xFF - Reset
 *------------------------------------------------------------------------------
 */

SYSTEM_MODE(SEMI_AUTOMATIC);

#include "ESC.h"

// Toogle debug output
static bool debug = true;

// Define pins on RedBear Duo here
#define STEER_SERVO_PIN             A4
#define GEAR_SERVO_PIN              A5
#define THROTTLE_SERVO_PIN          A6
#define BREAK_LIGHTS_PIN            D0
#define INDICATOR_LEFT_LIGHTS_PIN   D1
#define INDICATOR_RIGHT_LIGHTS_PIN  D2
#define MAIN_LIGHTS_PIN             D3
#define HIGH_BEAM_PIN               D4
#define REVERSE_LIGHT_PIN           D5
#define POSITION_LIGHT_PIN          D6
#define FOG_LIGHT_PIN               D7

// Define light commands
#define MAIN_LIGHTS         1
#define HIGH_BEAM           2
#define HAZARD_FLASHERS     3
#define FOG_LIGHTS          4

// Servos
Servo steerServo;
Servo gearServo;

// ESC
ESC esc(ESC::MODE_FORWARD_BACKWARD);

// Conversions
static int gearAngle[3] = {10, 85, 170};   // Servo angles for gears
static int maxSteer = 65;                   // Maximum Steer angle
static long maxThrottle = 250;              // Maximum power (500=100%)

// Some globals
static bool blink_left = false;
static bool blink_right = false;
static bool blink_both = false;
static bool lights_on = false;
static long oldThrottle = 0;
static long oldLight = 0;

// Include BLE code
#include "ble.h"

/*------------------------------------------------------------------------------
 * Truck controller routines
 *------------------------------------------------------------------------------
 */
void truckLightController(uint8_t light, uint8_t status) {
 
    if (light == MAIN_LIGHTS) {
        lightToggle(MAIN_LIGHTS_PIN, status);
        lightToggle(POSITION_LIGHT_PIN, status);
        if (status) {oldLight=64;} else {oldLight=0;}
        analogWrite(BREAK_LIGHTS_PIN, oldLight);
        
    }
    else if (light == HIGH_BEAM) {
        lightToggle(HIGH_BEAM_PIN, status);
    }
    else if (light == HAZARD_FLASHERS) {
        lightToggle(INDICATOR_LEFT_LIGHTS_PIN, status);
        lightToggle(INDICATOR_RIGHT_LIGHTS_PIN, status);
        blink_both = (status==0x01);
    }
    else if (light == FOG_LIGHTS) {
         lightToggle(FOG_LIGHT_PIN, status);
    }
}

//------------------------------------------------------------------------------
void truckController(uint8_t steerRaw, uint8_t throttleRaw) {
 
    // Map values from app (0..255) to desired range.
    
    long steer=map(steerRaw,0,255,90-maxSteer,90+maxSteer);
    long throttle=map(throttleRaw,0,255,0,maxThrottle);
     
    // Steering and indicators numbers from 0-180 with 90 being neutral
    steerServo.write(steer);
    
    if (steer < 80) {
        blink_left = true;
    } 
    else if (steer > 100) {
        blink_right = true;
    }
    else {
        blink_left = false;
        blink_right = false;
        lightToggle(INDICATOR_LEFT_LIGHTS_PIN, 0);
        lightToggle(INDICATOR_RIGHT_LIGHTS_PIN, 0);
    }
    
    // Throttle
    esc.setSpeed(throttle);
    
    if (throttle < oldThrottle) {
        analogWrite(BREAK_LIGHTS_PIN, 255);
    }
    else {
        analogWrite(BREAK_LIGHTS_PIN, oldLight);
    }
    oldThrottle = throttle;
}

//------------------------------------------------------------------------------
void truckGearController(uint8_t gear, uint8_t value) {
 
    // Reverse/Drive
    
    if (gear == 255) {
        if (value == 1) {
            esc.setSpeed(0);
            esc.setDirection(ESC::BACKWARD);
            lightToggle(REVERSE_LIGHT_PIN, 1);
        } 
        else {
            esc.setSpeed(0);
            esc.setDirection(ESC::FORWARD);
            lightToggle(REVERSE_LIGHT_PIN, 0);
        }
    }
    
    // Gears 1,2,3
    
    else if (gear>=1 && gear<=3) {
        gearServo.write(gearAngle[gear-1]); 
    }
}

//------------------------------------------------------------------------------
void truckReset() {
 
    // all stop
    blink_left = false;
    blink_right = false;
    blink_both = false;
    
    analogWrite(BREAK_LIGHTS_PIN, 0);
    digitalWrite(INDICATOR_LEFT_LIGHTS_PIN, LOW);
    digitalWrite(INDICATOR_RIGHT_LIGHTS_PIN, LOW);
    digitalWrite(MAIN_LIGHTS_PIN, LOW);
    digitalWrite(HIGH_BEAM_PIN, LOW);
    digitalWrite(REVERSE_LIGHT_PIN, LOW);
    digitalWrite(POSITION_LIGHT_PIN, LOW);
    digitalWrite(FOG_LIGHT_PIN, LOW);
    
    
    steerServo.write(90);
    gearServo.write(90);
    esc.setSpeed(0);
}

//------------------------------------------------------------------------------
void lightToggle(uint8_t pin, uint8_t status) {

    if (status == 0x01)
        digitalWrite(pin, HIGH);
    else
        digitalWrite(pin, LOW);
}

//------------------------------------------------------------------------------
void readySignal() {

    digitalWrite(INDICATOR_LEFT_LIGHTS_PIN, HIGH);
    digitalWrite(INDICATOR_RIGHT_LIGHTS_PIN, HIGH);
    
    delay(1000);

    digitalWrite(INDICATOR_LEFT_LIGHTS_PIN, LOW);
    digitalWrite(INDICATOR_RIGHT_LIGHTS_PIN, LOW);
}

/*------------------------------------------------------------------------------
 * Calibration functions (called from cloud)
 *------------------------------------------------------------------------------
 */
long setThrottle(String command) {

    long number = (long) strtol(&command[0], NULL, 10);
    
    if (number>0) {
        esc.setSpeed(0);
        esc.setDirection(ESC::FORWARD);
        esc.setSpeed(number);
    }
    else {
        esc.setSpeed(0);
        esc.setDirection(ESC::BACKWARD);
        esc.setSpeed(-number);
    }
    return number;
}

long setGear(String command) {
    
    long number = (long) strtol(&command[0], NULL, 10);
    
    if (number>0 && number<180) {
        gearServo.write(number);
    }
    
    return number;
}

/*------------------------------------------------------------------------------
 * Setup
 *------------------------------------------------------------------------------
 */
void setup() {

    // Initialize servos and pins
    esc.attach(THROTTLE_SERVO_PIN);
    steerServo.attach(STEER_SERVO_PIN);
    gearServo.attach(GEAR_SERVO_PIN);

    pinMode(BREAK_LIGHTS_PIN, OUTPUT);
    pinMode(INDICATOR_LEFT_LIGHTS_PIN, OUTPUT);
    pinMode(INDICATOR_RIGHT_LIGHTS_PIN, OUTPUT);
    pinMode(MAIN_LIGHTS_PIN, OUTPUT);
    pinMode(HIGH_BEAM_PIN, OUTPUT);
    pinMode(REVERSE_LIGHT_PIN, OUTPUT);
    pinMode(POSITION_LIGHT_PIN, OUTPUT);    
    pinMode(FOG_LIGHT_PIN, OUTPUT);    
    
    // Connect to particle cloud
    Particle.connect();
    
    // Bluetooth setup
    setupBLE();
    
    // Reset everything to starting values
    truckReset();
  
    // Flash to signal truck is ready
    readySignal();
    
    // Publish test functions
    if(debug) {
        Particle.function("throttle",setThrottle);
        Particle.function("gear",setGear);
    }
}

/*------------------------------------------------------------------------------
 * Loop
 *------------------------------------------------------------------------------
 */
void loop() {
    
    // code for blinkers
    if(blink_left || blink_both) {
        digitalWrite(INDICATOR_LEFT_LIGHTS_PIN, millis()>>9 &1); // 9 for 2Hz blinking, use 10 for 1Hz
    }
    if(blink_right || blink_both) {
        digitalWrite(INDICATOR_RIGHT_LIGHTS_PIN, millis()>>9 &1); // 9 for 2Hz blinking, use 10 for 1Hz
    }
}