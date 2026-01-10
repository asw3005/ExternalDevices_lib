It used I2C to 1-Wire transceiver. 
Not all functions are tested.
I2C tested. Three and two power supply variants are working now. Hot connection is working.

UART variant isn't tested here, use DS18B20V1 folder for working through UART module.
01.10.26 UART version tested.
To use it, you need to connect TX and RX pins of STM32 device together through the 470 ohm resistor, pull up TX pin to VCC through the 1k resistor and set the pins to open drain in your program. Three and two power supply variants are working now. 