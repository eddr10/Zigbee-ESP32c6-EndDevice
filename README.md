| Supported Targets | ESP32-C6 | ESP32-H2 |
| ----------------- | -------- | -------- | 

# ESP32-C6 Zigbee End device

## How to Use

Before project configuration and build, be sure to set the correct chip target using `idf.py set-target <chip_name>`.

### Hardware Required

* A development board with Espressif SoC with Zigbee support
* An USB cable for Power supply and programming
* DHT11 sensor
* HC-SR501 sensor
* LDR module sensor 

| Sensor               | Pin                  |
| -------------------- | -------------------- |
| DHT 11               | GPIO 11              |
| HC-SR501             | GPIO 10              |
| LDR                  | GPIO 0               |

See [https://www.espressif.com/sites/default/files/documentation/esp32-c6-wroom-1_wroom-1u_datasheet_en.pdf] for more information about pinout.

### Configure the Project

Open the project configuration menu (`idf.py menuconfig`).


### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.



## Troubleshooting
