/********************************************************************************
 * 
 * Face Detection and Recognition Sensor
 * 
 * N.B. The SEN12131 sensor takes significant time to boot and so it must be
 *      accounted for.
 *
 ********************************************************************************/

#include "facedet.h"


#define TESTING


#ifdef TESTING
#define     print(val)      Serial.print(val);
#define     println(val)    Serial.println(val);
#define     serialInit()    Serial.begin(115200); while (!Serial) { ; }; delay(500);
#else /* TESTING */
#define     print(val)      ;
#define     println(val)    ;
#define     serialInit()    ;
#endif /* TESTING */


void (* resetFunc)(void) = 0;


bool active = false;
uint8_t detects = 0;
unsigned long activeTime;

USPS *usps;

OnBoardLED *userLED;
OnBoardLED *neoPix;


void setup() {
    serialInit();

    pinMode(ACTIVATE_PIN, OUTPUT);
    digitalWrite(ACTIVATE_PIN, LOW);

    pinMode(PIN_SW0, INPUT_PULLUP);
    pinMode(PIN_SW1, INPUT_PULLUP);
    pinMode(PIN_SW2, INPUT_PULLUP);

    userLED = new OnBoardLED(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    neoPix = new OnBoardLED(NEOPIXEL_POWER, PIN_NEOPIXEL);

    // start up with User LED Blue and Green NeoPixel
    userLED->setColor(BLUE);
    neoPix->setColor(GREEN);

    Wire.begin();

    // have to wait for the sensor to come up
    int8_t n = -1;
    while (n < 0) {
        USPSface_t faces[1];
        delay(500);
        usps = new USPS();
        n = usps->getFaces(faces, 1);
    }

    // turn off User LED and NeoPixel on boot completion
    userLED->off();
    neoPix->off();
};

void loop() {
    USPSface_t faces[USPS_MAX_FACES];
    int8_t numFaces;

    int8_t n = usps->getFaces(faces, USPS_MAX_FACES);
    numFaces = n;
    for (int i = 0; (i < n); i++) {
        if (!faces[i].isFacing || (faces[i].boxConfidence < MIN_CONFIDENCE)) {
            numFaces--;
        }
    }

    if (numFaces < 0) {
        println("ERROR: failed to read, resetting...");
        // turn User LED red to indicate failure of peripheral read
        userLED->setColor(RED);
        usps = new USPS();
        return;
    }
    if (numFaces) {
        detects++;
        if (!active) {
            if (detects >= DETECT_COUNT) {
                active = true;
                activeTime = millis();
                digitalWrite(ACTIVATE_PIN, HIGH);

                // turn User LED White to indicate active
                userLED->setColor(WHITE);
            }
        }
    } else {
        detects = 0;
        if (active) {
            if ((millis() - activeTime) > MIN_ACTIVE_MS) {
                active = false;
                digitalWrite(ACTIVATE_PIN, LOW);

                // turn off User LED to indicate inactive
                userLED->off();
            }
        }
    }
    uint8_t val = (digitalRead(PIN_SW0) < 2) || (digitalRead(PIN_SW1) < 1) || digitalRead(PIN_SW0);
    print("> "); println(val);

    delay(LOOP_DELAY);
};
