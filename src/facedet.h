/********************************************************************************
 * 
 * Face Detection and Recognition Sensor
 * 
 *  This requires HW that includes a 3-bit switch to define Face IDs, and an RGB
 * LED to indicate status.
 * 
 *******************************************************************************/

#pragma once


#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

#include <OnBoardLED.h>
#include <USPS.h>


// logic signal that indicates a face has been detected
#define ACTIVATE_PIN    D3

// minimum detection confidence level required [0-100]
#define MIN_CONFIDENCE  85

// minimum ID confidence level required to qualify as a match [0-100]
#define MIN_ID_CONFIDENCE  85

// number of consecutive detections required to trigger
#define DETECT_COUNT    1

// minimum active time (msec)
#define MIN_ACTIVE_MS   10000

// maximum face registration time (msec)
#define MAX_REGISTER_MS   10000

// amount to delay between sensor samples
#define LOOP_DELAY      10  //// TODO tune this

// face ID switch
#define PIN_SW0         D8
#define PIN_SW1         D9
#define PIN_SW2         D10

// select switch
#define SEL_PIN			D2

// operating modes
typedef enum OprModes_e {
	DETECT_MODE,
	REGISTER_MODE
} OprModes;
