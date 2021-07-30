# dspic-control-rc-circuit
This is my attempt to further my knowledge about control theory, by using a microcontroller to apply some of its concepts on a RC circuit

The code is compiled using XC16 v1.60 and has been successfully tested on a dsPIC33EV32GM102.

## 1. PI compensator

### About

This is an implementation of a PID compensator that can be tuned by sending the parameters through the serial port (tailored for [qt-pid-scope](https://github.com/odjadane/qt-pid-scope), but easily customizable for other uses).

A demonstration of this program can be seen [in this video](https://www.youtube.com/watch?v=Imp_jYF0e8U).

### Hardware

- The internal fast RC oscillator is used with PLL to boost the system frequency up to 80 MHz.
- Timer2 is used with OC1 to generate a 1 kHz PWM output.
- Timer3 triggers the ADC, updates the duty cycle and sends data at a sampling rate of 10ms.
- UART module operates at a bitrates of 38400 bit/s.
- Used pins:
  - RB2 → RX / RB5 → TX
  - RB7 → ADC input (voltage across the capacitor).
  - RB10 → PWM output (voltage applied across the whole RC circuit).

### Software

- Due to the use of utoa(), the option "Use legacy libc" must be disabled (Project properties > XC16 > XC16 (Global Options) in MPLAB X).
- This program can be optimized by converting floating-point computations to fixed-point.



## Acknowledgments

- [Hardware-based activities - CTMS](https://ctms.engin.umich.edu/CTMS/index.php?aux=Index_Activities)