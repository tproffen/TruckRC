
/*------------------------------------------------------------------------------
 * BLE Truck Controller
 * (c) Thomas Proffen, 2018
 *
 * Commands 
 *     0x01 - Operate lights, Parameters: light, status
 *     0x02 - Set gear, Parameters: gear
 *     0x03 - Drive, ParametersL stree, throttle
 *     0xFF - Reset
 *------------------------------------------------------------------------------
 */
#include <DFRobotDFPlayerMini.h>

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

// Define light commands
#define MAIN_LIGHTS         1
#define HIGH_BEAM           2
#define HAZARD_FLASHERS     3

// Servos
Servo steerServo;
Servo throttleServo;
Servo gearServo;

// Sound
DFRobotDFPlayerMini myDFPlayer;

// Conversions
static int gearAngle[3] = {105, 90, 75}; // Servo angles for gears


// Some globals
static bool blink_left = false;
static bool blink_right = false;
static bool blink_both = false;
static uint8_t oldThrottle = 0;

// Include BLE code
#include "ble.h"

/*------------------------------------------------------------------------------
 * Truck controller routines
 *------------------------------------------------------------------------------
 */
void truckLightController(uint8_t light, uint8_t status) {
 
    if (light == MAIN_LIGHTS) {
        lightToggle(MAIN_LIGHTS_PIN, status);
    }
    else if (light == HIGH_BEAM) {
        lightToggle(HIGH_BEAM_PIN, status);
    }
    else if (light == HAZARD_FLASHERS) {
        lightToggle(INDICATOR_LEFT_LIGHTS_PIN, status);
        lightToggle(INDICATOR_RIGHT_LIGHTS_PIN, status);
        blink_both = (status==0x01);
    }
}

/*------------------------------------------------------------------------------
 * Truck controller routines
 *------------------------------------------------------------------------------
 */
void truckSounds(uint8_t sound, uint8_t status) {
 
    if (status == 1) {
        myDFPlayer.loop(sound);
    }
    else 
    {
        myDFPlayer.stop();    
    }
}

//------------------------------------------------------------------------------
void truckController(uint8_t steer, uint8_t throttle) {
 
    steerServo.write(steer);
    throttleServo.write(throttle);
    
    // Indicators
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
    
    // Going forward
    if (throttle >=90) {
        if (throttle < oldThrottle) {
            lightToggle(BREAK_LIGHTS_PIN, 1);
        }
        else {
            lightToggle(BREAK_LIGHTS_PIN, 0);
        }
        lightToggle(REVERSE_LIGHT_PIN, 0);
    } 
    // Going backwards
    else 
    {
        if (throttle > oldThrottle) {
            lightToggle(BREAK_LIGHTS_PIN, 1);
        }
        else {
            lightToggle(BREAK_LIGHTS_PIN, 0);
        }
        lightToggle(REVERSE_LIGHT_PIN, 1);
    }
    oldThrottle = throttle;
}

//------------------------------------------------------------------------------
void truckGearController(uint8_t gear) {
 
    gearServo.write(gearAngle[gear-1]); 
}

//------------------------------------------------------------------------------
void truckReset() {
 
    // all stop
    blink_left = false;
    blink_right = false;
    blink_both = false;
    
    digitalWrite(BREAK_LIGHTS_PIN, LOW);
    digitalWrite(INDICATOR_LEFT_LIGHTS_PIN, LOW);
    digitalWrite(INDICATOR_RIGHT_LIGHTS_PIN, LOW);
    digitalWrite(MAIN_LIGHTS_PIN, LOW);
    digitalWrite(HIGH_BEAM_PIN, LOW);
    digitalWrite(REVERSE_LIGHT_PIN, LOW);
    
    steerServo.write(90);
    throttleServo.write(90);
    gearServo.write(90);
    
    myDFPlayer.stop();
}

//------------------------------------------------------------------------------
void lightToggle(uint8_t pin, uint8_t status) {

    if (status == 0x01)
        digitalWrite(pin, HIGH);
    else
        digitalWrite(pin, LOW);
}

//------------------------------------------------------------------------------
void setupSound() {
    
    // Setting up sound player
    Serial1.begin(9600);

    unsigned long startedWaiting = millis();
    while((!myDFPlayer.begin(Serial1)) && millis() - startedWaiting <= 5000)
    {
        delay(500);
    }
    
    if (debug) {
        Particle.publish ("status","DFPlayer Mini online");
    }
  
    myDFPlayer.volume(25);
}

/*------------------------------------------------------------------------------
 * Setup
 *------------------------------------------------------------------------------
 */
void setup() {

    // Bluetooth setup
    setupBLE();

    // Initialize pins for lights.
    pinMode(BREAK_LIGHTS_PIN, OUTPUT);
    pinMode(INDICATOR_LEFT_LIGHTS_PIN, OUTPUT);
    pinMode(INDICATOR_RIGHT_LIGHTS_PIN, OUTPUT);
    pinMode(MAIN_LIGHTS_PIN, OUTPUT);
    pinMode(HIGH_BEAM_PIN, OUTPUT);
    pinMode(REVERSE_LIGHT_PIN, OUTPUT);

    // Servos (90 is center)
    steerServo.attach(STEER_SERVO_PIN);
    steerServo.write(90);
    gearServo.attach(GEAR_SERVO_PIN);
    gearServo.write(90);
    throttleServo.attach(THROTTLE_SERVO_PIN);
    throttleServo.write(90);
  
    // Setting up sound player
    setupSound();
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