# Build Test

This is an example project to test CANopenNode port for ESP32.  For this example project, ESP32-S3 is used.

## Hardware Test Setup
The setup is composed of ESP-S3-32S kit and SN65HVD230 CAN transceiver module.

<p align="center">
  <img src="https://github.com/sicrisembay/CANopenNode_ESP32_Test/blob/main/examples/build_test/doc/img/ESP-S3-32S.png">
</p>

<p align="center">
  <img src="https://github.com/sicrisembay/CANopenNode_ESP32_Test/blob/main/examples/build_test/doc/img/SN65HVD230.png">
</p>

TWAI connection to ESP32:

| Function | ESP-S3-32S |
|----------|------------|
| CAN-TX   | GPIO01     |
| CAN-RX   | GPIO02     |

