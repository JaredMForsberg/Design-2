🔧 Project Overview

This project enables capturing still images or frame data from an embedded camera module such as an OV2640 or compatible module. These camera modules use serial and SPI/I²C interfaces typical in Arduino-based systems.

The Arduino sketch (Test.ino) initializes the camera hardware and streams captured image data over USB/Serial to a host computer. On the computer, a Python script (grab_arducam_frame.py) reads and processes the incoming data.

🚀 Quick Start

🛠 Requirements

Arduino IDE (with board support for your microcontroller)

Python 3 with pyserial installed

OV2640-compatible camera module (e.g., ArduCAM Mini 2MP)

💡 Arduino Setup

Connect your camera module to the microcontroller (SPI/I²C pins, power, ground).

Open Test.ino in the Arduino IDE.

Set the correct board and port under Tools > Board / Port.

Upload the sketch.

🐍 Grab Frames With Python

After the Arduino starts streaming frames over serial:

pip install pyserial
python grab_arducam_frame.py


This script reads raw pixel data and can save it or display it as an image.

🧪 Example Output

You’ll find an example image data file (example_bmp_serial_pixel_data__160x120.txt) and a reference capture (capture.jpg) in the repo. These are intended to help validate the capture workflow and format conversions.

🧠 Notes

The project currently has no official documentation — this README aims to fill that gap.

The camera module uses protocols similar to standard OV2640 boards (e.g., over SPI/I²C).

If you’re working with an ESP32-CAM variant, consider using the official ArduCAM/ESP32 libraries for higher-level camera APIs.
