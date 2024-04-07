# CANopenNode ESP32 Test

Main objective of this repo is to test the CANopenNode_ESP32 submodule.

```
git clone --recursive https://github.com/sicrisembay/CANopenNode_ESP32_Test.git
cd CANopenNode_ESP32_Test
```


# Examples
All examples uses ESP-IDF v5.2.
```
<ESP-IDF_v5.2 PATH>/export.ps1
```

## Example01: [build_test](https://github.com/sicrisembay/CANopenNode_ESP32_Test/tree/main/examples/build_test)

This is an example project to test CANopenNode port for ESP32. For this example project, ESP32-S3 is used.  The setup is composed of ESP-S3-32S kit and SN65HVD230 CAN transceiver module.  Detailed description is found in the example [README](https://github.com/sicrisembay/CANopenNode_ESP32_Test/tree/main/examples/build_test).

```
cd examples/build_test
```

Enable CANopenNode in the configuration.

```
idf.py menuconfig
```

![](doc/img/menuconfig_canopennode.png)


Build project

```
idf.py build
```

