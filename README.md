This repo contains software and information for controlling a DC motor using an existing motor controller scavenged from a treadmill.  Most of this information comes from [Sons of Invention](https://sonsofinvention.wordpress.com/2013/05/22/arduino-compatible-mc-2100-controller-and-lathe-tachometer/) blog which has excellent details and schematics. 

## Parts
-Arduino Nano
-rotary encoder
-4 digit 7 segment display
-MC-2100 Motor control board


## Building and Uploading Firmware

The firmware is uploaded to the arduino use arduino makefiles rather than the Arduino IDE in order to avoid some of the limitations of the IDE.
You can build and upload the firmware via the command-line.

Check out [Arduino CMake](https://github.com/queezythegreat/arduino-cmake) for more information.

### Pre-requisites

You must have Arduino Core and CMake (version 2.8 or greater) installed on
your machine.

``
sudo apt install arduino-core cmake
``

### Building the Firmware

Navigate to the root directory of the repository and use `make`:

``
make
``

### Uploading the Firmware
``
make upload
``


