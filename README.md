# AD24000 Teensyduino (Arduino for Teensy) Demo
### An example C++ library and Teensyduino project for the [AD24000](http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16460.pdf) iSensor Calibrated, Compact, Precision Six Degrees of Freedom Inertial Sensor

This example library was written to give engineers, students, and makers a starting point for using a high-performance, compact, precision inertial sensor. The code in this repository will provide the user with:
- A header file listing all of the unit's available registers
- Functions for reading output registers and writing control registers using **8-bit** frames
    - Note that the AD24000 requires 16 bit SPI transactions. spi.transfer() is called twice for each transfer and CS is manually toggled to overcome the Arduino language's limitation 
- Functions for performing common routines such as resetting the sensor
- Example Arduino sketches which synchronously read data from the sensor and write it to the serial port

### What do I need to get started?

- In order to compile and execute the Teensyduino sketch, you'll need to download the "legacy" Arduino package (v1.0.6 as of this writing). You can download the IDE [here](http://arduino.cc/download.php?f=/arduino-1.0.6-windows.zip).
- You'll also need to install the Teensyduino [library](https://www.pjrc.com/teensy/td_download.html) provided by PJRC.
- Finally, you'll need a Teensy sold by PJRC [here](https://www.pjrc.com/store/teensy32.html). Version 3.x or LC is supported.
- The main Teensyduino sketch issues a command to clear the terminal window after displaying data. For best results, connect to your Teensy using [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html), an open source terminal program.

### How do I install the library?

Once you've installed the Arduino IDE and Teensyduino libraries, copy the AD24000 folder into `My Documents > Arduino > libraries`

Be sure to restart the Arduino IDE to refresh the library directory!

### How do I connect the IMU to my Arduino?

**Note that the AD24000 is a 3.3V part! If using this code with a legacy Arduino, it must be modified before connecting the sensor to it! DIO is 5V tolerant, but should be connected to a 3.3V MCU for increased reliability. A guide to modifying an Arduino Uno can be found [here](https://learn.adafruit.com/arduino-tips-tricks-and-techniques/3-3v-conversion).**

**If using a Teensy, the onboard regulator should provide enough current for the AD24000 to properly operate.**

After modifying the Arduino, you'll need to build a cable to interface the sensor with the [ADIS16IMU4/PCBZ](http://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADIS16IMU4.html#eb-overview). The image below shows an AD24000 connected to an Arduino Uno.

![AD24000-Arduino Cable Interface](https://raw.githubusercontent.com/juchong/AD24000-Arduino-Teensy/master/AD24000/images/IMG_6015.JPG)

Pin assignments for the Teensy can be found in the example sketch comments.

### How do I know it's working?

Once you have the sensor connected and have opened the **AD24000_Teensy_Example** example sketch, use PuTTY to connect to the arduino using the following settings. Note that your COM port may be different:

![AD24000 Example PuTTY Config](https://raw.githubusercontent.com/juchong/ADIS16209-Arduino-Demo/master/setup_pictures/PuTTYConfig.PNG)

If everything is working, you should see a screen like this:

![AD24000 Example PuTTY Output]()
