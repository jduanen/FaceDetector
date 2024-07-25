# FaceDetector
A Face Detection and Recognition Sensor Based on the SEN21231 sensor

This is a standalone device that allows (up to 7) faces to be registered
and, whenever a face is detected, an indication of which of the registered
faces is detected (or an indication of an unknown face).

An example of this type of sensor uses an RP2040-based microcontroller
connected to the SEN21231, a set of switches used to select the IDs for
registered faces, and an RGB LED to indicate the state of the registration
and detection/recognition functions.

This design is modularized so that it can be used in different applications.
The attention sensor controller manages face registration and emits face
detection indications over a serial port. A separate controller can be used
that takes this serial input and interfaces the face detection/recognition
subsystem to Home Assistant.

* Features
  - Integrated with Home Assistant via independent microcontroller.
    * generate events when faces are detected and provide Face ID
  - Register Face
    * select face number with number switch
    * get Green LED indication of face detected
    * press momentary button to register face
    * colored LED (NeoPixel) on Xiao RP2040 indicates status
      - off: scanning for faces
      - Red: error
      - Green: ?
      - Blue: ?
      - Cyan: ?
      - Magenta: ?
      - Yellow: ?
  - Erase Registered Faces
    * hold momentary button while turning on power
    * NeoPixel indicates status
      - ?: ?
      - ?: ?
      - ?: ?
* Add HW to controller
  - [0-7] switch
  - register (momentary) button

# Links
* Xiao RP2040
  - https://wiki.seeedstudio.com/XIAO-RP2040/
* Useful Sensors SEN21231
  - https://github.com/usefulsensors/person_sensor_docs
