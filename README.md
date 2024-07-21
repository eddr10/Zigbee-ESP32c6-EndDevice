| Supported Targets | ESP32-C6 | ESP32-H2 |
| ----------------- | -------- | -------- | 

# ESP32-C6 Zigbee End device

## How to Use

Before project configuration and build, be sure to set the correct chip target using `idf.py set-target <chip_name>`.

### Hardware Required

* A development board with Espressif SoC with Zigbee support
* A USB cable for Power supply and programming
* DHT11 sensor
* HC-SR501 sensor
* LDR module sensor 

| Sensor               | Pin                  |
| -------------------- | -------------------- |
| DHT 11               | GPIO10               |
| HC-SR501             | GPIO8                |
| LDR                  | GPIO0                |

See [https://www.espressif.com/sites/default/files/documentation/esp32-c6-wroom-1_wroom-1u_datasheet_en.pdf] for more information about pinout.

### Configure the Project

Open the project configuration menu (`idf.py menuconfig`).

In the `Example Configuration` menu:

* Select the LED type in the `Blink LED type` option.
  * Use `GPIO` for regular LED blink.
* Set the GPIO number used for the signal in the `Blink GPIO number` option.
* Set the blinking period in the `Blink period in ms` option.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

As you run the example, you will see the LED blinking, according to the previously defined period. For the addressable LED, you can also change the LED color by setting the `led_strip_set_pixel(led_strip, 0, 16, 16, 16);` (LED Strip, Pixel Number, Red, Green, Blue) with values from 0 to 255 in the [source file](main/blink_example_main.c).

```text
I (315) example: Example configured to blink addressable LED!
I (325) example: Turning the LED OFF!
I (1325) example: Turning the LED ON!
I (2325) example: Turning the LED OFF!
I (3325) example: Turning the LED ON!
I (4325) example: Turning the LED OFF!
I (5325) example: Turning the LED ON!
I (6325) example: Turning the LED OFF!
I (7325) example: Turning the LED ON!
I (8325) example: Turning the LED OFF!
```

Note: The color order could be different according to the LED model.

The pixel number indicates the pixel position in the LED strip. For a single LED, use 0.

## Troubleshooting

* If the LED isn't blinking, check the GPIO or the LED type selection in the `Example Configuration` menu.

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
