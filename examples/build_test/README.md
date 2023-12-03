# Build Test

This is an ESP-IDF example project to test CANopenNode port for ESP32.  For this example project, ESP32-S3 is used.

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

For CiA 303-3 test, the following LEDs are used:

| LED   | ESP-S3-32S  |
|-------|-------------|
| RED   | GPIO39      |
| GREEN | GPIO06      |

## Compile and Run
Build the project.

```
idf.py build
```

Flash into the ESP-S3-32S Kit.  Change the COM port as needed.

```
idf.py -p COM13 flash monitor
```

ESP32 serial log is shown below.

<p align="center">
  <img src="https://github.com/sicrisembay/CANopenNode_ESP32_Test/blob/main/examples/build_test/doc/img/log.png">
</p>

## CANopen Test
You can do a lot of test using Robin Cornelius' [CanOpenMonitor](https://github.com/robincornelius/CanOpenMonitor) C# project.  Since I only have PCAN adapter, I've slightly modified his project to support Peak Adapter (see [here](https://github.com/sicrisembay/CanOpenMonitor/tree/pcan)).


CanOpenMonitor Device OD Editor is used to test SDO read and write.

<p align="center">
  <img src="https://github.com/sicrisembay/CANopenNode_ESP32_Test/blob/main/examples/build_test/doc/img/SDO_test.png">
</p>



For CiA 303-3, you can use CANopenMonitor to change NMT states and observe the CANopen LED indicator pattern (see description [here](https://github.com/CANopenNode/CANopenNode/blob/4f68e4404a8d329958ec4db6e9da8d1e93e947e2/303/CO_LEDs.h#L54-L69)).

<p align="center">
  <img src="https://github.com/sicrisembay/CANopenNode_ESP32_Test/blob/main/examples/build_test/doc/img/NMT_test.png">
</p>
