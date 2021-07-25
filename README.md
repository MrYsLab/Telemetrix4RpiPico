# This is a working branch to implement an SPI interface.
## An update log is below:

* July 21 2021
  * Created the spi branch.
  
* July 24 2021
  * Updated Telemetrix4RpiPico.h for SPI processing
  * init_spi method coded. Not yet tested.
  * write_blocking_spi coded. Not yet tested.
  
* July 25 2021
  * Modified write_blocking_spi function. Not yet tested.
  * Implemented read_blocking_spi function. Not yet tested.
  * Seperated out SPI chip select control as a seperate function 

# Telemetrix4RpiPico

![](images/tmx.png)

The Telemetrix4RpiPico server, in conjunction with its [client peer](https://github.com/MrYsLab/telemetrix-rpi-pico),
allows you to control a Raspberry Pi Pico remotely from your
PC. A complete [User's Guide](https://mryslab.github.io/telemetrix-rpi-pico/) is available describing how to 
install and use both the server and client.

To install the server, follow these [installation instructions.](https://mryslab.github.io/telemetrix-rpi-pico/install_pico_server/)
