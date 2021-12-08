# Telemetrix4RpiPico

![](images/tmx.png)

The Pico server code may be viewed [here.](https://github.com/MrYsLab/Telemetrix4RpiPico)

You may download the .uf2 file [here.](https://github.com/MrYsLab/Telemetrix4RpiPico/raw/master/cmake-build-release/Telemetrix4RpiPico.uf2)

The following functionality is implemented in this release:

* Analog Input
* Digital Input, Digital Input Pullup, Digital Input Pulldown
* PWM output
* Loopback (for client/server link debugging)
* I2C Support
* SPI Support
* NeoPixel Support
* Servo Support
* HC-SR04 Type Sonar Distance Sensor Support
* DHT 11 and 22 Humidity/Temperature Sensor Support
* Autodetect PICO device over USB Serial.
* Automatic board reset of the PICO using the watchdog timer when application exits.
  * Board will blink twice upon reset.
* Retrieval of the PICO's unique ID.

The Telemetrix4RpiPico server, in conjunction with its [client peer](https://github.com/MrYsLab/telemetrix-rpi-pico),
allows you to control a Raspberry Pi Pico remotely from your
PC. A complete [User's Guide](https://mryslab.github.io/telemetrix-rpi-pico/) is available describing how to 
install and use both the server and client.

To install the server, follow these [installation instructions](https://mryslab.github.io/telemetrix-rpi-pico/install_pico_server/).
