# M5Atom Echo GPS Navigation System

## Overview

This project uses an M5Atom Echo ESP32 board to create a simple GPS navigation device. It reads data from a BN-880 GPS module (which includes a QMC5883L compass), displays coordinates, altitude, heading, and satellite count on an SSD1306 OLED display, and provides different display modes. It also allows for compass calibration.

## Hardware Requirements

1.  **M5Atom Echo:** The main microcontroller board.
2.  **BN-880 GPS Module:** Provides GPS location and compass heading via UART and I2C respectively.
3.  **SSD1306 OLED Display (128x64, I2C):** Used to display information.
4.  **Grove Cable:** For connecting the M5Atom Echo to the I2C devices (Display and Compass).
5.  **Dupont Wires:** For connecting the GPS UART and power.
6.  **(Optional) USB to TTL Serial Converter:** For viewing debug messages sent from the M5Atom Echo.
7.  **Power Source:** USB power supply for the M5Atom Echo.

## Wiring Instructions

**Important:** Ensure the M5Atom Echo is **NOT** powered while wiring.

**1. I2C Connections (Display and Compass):**

*   The BN-880 compass and the SSD1306 display both use the I2C protocol. They can share the same I2C bus connected to the M5Atom Echo's Grove port (Port A).
*   **M5Atom Grove Port A <--> Grove Cable <--> I2C Hub/Splitter (if needed) <--> SSD1306 & BN-880**
    *   Connect the **SDA** pin (usually Grove Pin 1 / M5Atom Pin **G26**) to the SDA pins of both the SSD1306 and the BN-880 compass I2C interface.
    *   Connect the **SCL** pin (usually Grove Pin 2 / M5Atom Pin **G32**) to the SCL pins of both the SSD1306 and the BN-880 compass I2C interface.
    *   Connect the **5V** pin from the Grove port to the VCC/5V pins of both the SSD1306 and BN-880.
    *   Connect the **GND** pin from the Grove port to the GND pins of both the SSD1306 and BN-880.

**2. GPS UART Connections (BN-880):**

*   Connect the **TX** (Transmit) pin of the BN-880 module to the M5Atom Echo's **G25** pin (GPIO 25 - configured as GPS RX).
*   Connect the **RX** (Receive) pin of the BN-880 module to the M5Atom Echo's **G21** pin (GPIO 21 - configured as GPS TX).
*   Ensure the BN-880 also receives 5V power and GND (can be shared with the I2C connections).

**3. Host/Debug UART Connections (Optional):**

*   If you want to view debug messages on a computer:
    *   Connect the **RX** pin of your USB-to-TTL converter to the M5Atom Echo's **G23** pin (GPIO 23 - configured as Host TX).
    *   Connect the **TX** pin of your USB-to-TTL converter to the M5Atom Echo's **G33** pin (GPIO 33 - configured as Host RX).
    *   Connect the **GND** of the USB-to-TTL converter to the M5Atom Echo's GND.

**Pin Summary Table:**

| M5Atom Pin | Function         | Connects To                | Protocol | Notes                            |
| :--------- | :--------------- | :------------------------- | :------- | :------------------------------- |
| G26 (SDA)  | I2C Data         | BN-880 SDA, SSD1306 SDA    | I2C      | Grove Port A Pin 1               |
| G32 (SCL)  | I2C Clock        | BN-880 SCL, SSD1306 SCL    | I2C      | Grove Port A Pin 2               |
| 5V         | Power            | BN-880 VCC, SSD1306 VCC    | Power    | Grove Port A                     |
| GND        | Ground           | BN-880 GND, SSD1306 GND    | Ground   | Grove Port A                     |
| G25        | GPS RX (ESP32)   | BN-880 **TX**              | UART     |                                  |
| G21        | GPS TX (ESP32)   | BN-880 **RX**              | UART     |                                  |
| G33        | Host RX (ESP32)  | TTL Converter **TX**       | UART     | Optional Debug Output            |
| G23        | Host TX (ESP32)  | TTL Converter **RX**       | UART     | Optional Debug Output            |

## Software Setup (PlatformIO)

1.  **Install PlatformIO:** Follow the instructions on the [PlatformIO website](https://platformio.org/) to install the IDE extension for VSCode.
2.  **Clone Repository:** Get the project code onto your computer.
3.  **Open Project:** Open the project folder in VSCode with PlatformIO installed.
4.  **Build & Upload:**
    *   Connect the M5Atom Echo to your computer via USB.
    *   PlatformIO should automatically detect the required libraries (`M5Unified`, `TinyGPS++`, `QMC5883LCompass`, `Adafruit GFX`, `Adafruit SSD1306`, etc.) specified in `platformio.ini` and install them.
    *   Use the PlatformIO toolbar (usually at the bottom of VSCode) to:
        *   **Build** the project (checkmark icon).
        *   **Upload** the compiled code to the M5Atom Echo (right arrow icon).

## Usage Guide

**Power On:**

*   Connect the M5Atom Echo to a USB power source.
*   The device will initialize, displaying messages on the OLED screen and sending logs to the Host serial port (if connected). It will start searching for a GPS signal.

**Display Modes:**

*   The device has three main display modes. Briefly press the M5Atom Echo's button to cycle through them:
    1.  **NORMAL Mode:** Displays current Latitude, Longitude, Altitude, Satellites, Compass Heading (Textual N, NE, E, etc.), and Azimuth (0-359 degrees). If no GPS fix is available, it shows "No Fix" and satellite count.
    2.  **STATUS Mode:** Shows the latest GPS status (coordinates or waiting status) and raw compass sensor readings (X, Y, Z ordinates). Useful for diagnostics.
    3.  **GRAPHIC COMPASS Mode:** Shows a graphical compass needle pointing North, the current heading (Text + Azimuth), and Latitude/Longitude. This mode is also used for initiating calibration.

**Compass Calibration:**

*   Accurate compass readings require calibration specific to the magnetic environment of the device.
*   **To Start Calibration:**
    1.  Cycle to the **GRAPHIC COMPASS Mode**.
    2.  **Press and Hold** the button until the display changes to the calibration screen ("--- Calibrate ---").
*   **Calibration Steps:**
    1.  The screen will prompt you to "Point NORTH, Click". Orient the device so the top points geographically North and briefly **press** the button.
    2.  The screen will then prompt "Point SOUTH, Click". Rotate the device 180 degrees to point South and **press** the button.
    3.  Continue following the prompts for **EAST**, **WEST**, **NE**, **SE**, **SW**, and **NW**, pressing the button briefly at each orientation.
    4.  After the 8th point (NW) is captured, the device will automatically calculate the calibration offsets and scales, save them to non-volatile memory, and return to the normal Graphic Compass display.
*   **To Cancel Calibration:** While in any calibration step, **press and hold** the button until the screen returns to the normal Graphic Compass display. The calibration process will be aborted without saving.
*   The calibration data is saved and loaded automatically on subsequent boots.

## Troubleshooting / Notes

*   **No GPS Fix:** Ensure the BN-880 has a clear view of the sky. It can take several minutes to get the first fix, especially indoors or on first use (cold start). Check wiring.
*   **No Display:** Check I2C wiring (SDA, SCL, 5V, GND) for the SSD1306. Verify the I2C address (the code tries 0x3C and 0x3D). Use the `scanI2CDevices()` output on the Host serial port to see if the display is detected.
*   **No Compass:** Check I2C wiring for the BN-880. Use the `scanI2CDevices()` output on the Host serial port to see if the compass (address 0x0D) is detected.
*   **Inaccurate Compass:** Perform the 8-point calibration procedure in an area away from large metal objects or strong magnetic fields.
*   **Serial Output:** Use a USB-to-TTL converter connected to the Host UART pins (G33 RX, G23 TX) and a terminal program (like PuTTY, Arduino Serial Monitor, PlatformIO Serial Monitor) set to **115200 baud** to view detailed logs and NMEA sentences.
*   **Baud Rates:** GPS UART is set to 115200, Host UART is 115200. Ensure your modules/settings match. 