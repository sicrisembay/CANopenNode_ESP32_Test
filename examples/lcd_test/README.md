# LCD Test

This is an ESP-IDF example project to test both CANopenNode port for ESP32 and LVGL.  For this example project, ESP32-S3 is used.

## LVGL
This example project uses LVGL v8.3.11.

For UI design, I'm using [SquareLine Studio](https://squareline.io/) v1.3.4.  For hobby projects, SquareLine Studio provides free Personal License (see [licences](https://squareline.io/pricing/licenses)).


## Hardware Test Setup
The setup is composed of ESP-S3-32S kit on a custom carrier board with SN65HVD230 CAN transceiver and 3.5" TFT LCD.  LCD is off-the-shelf 320px by 480px module with SKU MRB3511.  The LCD connects to ESP32 using 8-bit i80 interface.

TWAI connection to ESP32:

| Function | ESP-S3-32S |
|----------|------------|
| CAN-TX   | GPIO01     |
| CAN-RX   | GPIO02     |


ILI9488 LCD controller connection to ESP32:

| Function | ESP-S3-32S |
| ---------|------------|
| CS       | GPIO 15    |
| WR       | GPIO 17    |
| DC       | GPIO 4     |
| RESET    | GPIO 8     |
| BACKLIGHT| GPIO 36    |
| RD       | GPIO 16    |
| DATA0    | GPIO 18    |
| DATA1    | GPIO 20    |
| DATA2    | GPIO 19    |
| DATA3    | GPIO 11    |
| DATA4    | GPIO 10    |
| DATA5    | GPIO 9     |
| DATA6    | GPIO 38    |
| DATA7    | GPIO 3     |


GT911 Touch controller connection to ESP32:

| Function | ESP-S3-32S |
| ---------|------------|
| TC_RST   | GPIO 42    |
| TC_INT   | GPIO 40    |
| TC_SCL   | GPIO 41    |
| TC_SDA   | GPIO 37    |

