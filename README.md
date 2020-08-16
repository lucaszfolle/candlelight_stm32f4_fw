# candlelight_stm32f4_fw

Porting from the [candlelight_fw](https://github.com/candle-usb/candleLight_fw) to run on a STM32F4-Discovery board.
This project was generated with STM32CubeMX and is based on the STM32CubeIDE.

## Pinout

* CAN_RX : PD0;
* CAN_TX : PD1;

Red and blue LEDs are used to indicate operation and packets RX/TX.

## CANgaroo compatibility

The CAN peripheral clock in this MCU is 42 MHz, whereas in the STM32F04x family it is 48 MHz.
Since the bit timings are calculated by the host application, the CANgaroo had to be modified to be compatible with this MCU.

[Here](https://github.com/lucaszfolle/cangaroo) is a link to the forked cangaroo repo (pending PR).