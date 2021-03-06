# dspic-control-rc-circuit

This is my attempt to further my knowledge about control theory, by using a microcontroller to apply some of its concepts on a RC circuit

The code is compiled using XC16 v1.60 and has been successfully tested on a dsPIC33EV32GM102.



## 0. Notes

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
- These programs can be optimized by converting floating-point computations to fixed-point.



## 1. PI compensator

This is an implementation of a PID compensator that can be tuned by sending the parameters through the serial port (tailored for [qt-pid-scope](https://github.com/odjadane/qt-pid-scope), but easily customizable for other uses).

A demonstration of this program can be seen [in this video](https://www.youtube.com/watch?v=Imp_jYF0e8U).



## 2. Digital controller

This is an implementation of a controller designed entirely in the z-domain.

The setpoint is sent through the serial port (tailored for [qt-pid-scope](https://github.com/odjadane/qt-pid-scope), but easily customizable).

The design process is explained [in this video](https://www.youtube.com/watch?v=c4NSm0-ceeI).



## 3. Analog Lead compensator

No code has been written for this project as it was entirely implemented using op amps.

The design process is explained [in this video](https://www.youtube.com/watch?v=VcRpDT5GvP8). 



## 4. Digital Lead compensator

This is a discrete equivalent of a Lead compensator (previously implemented using op amps).

The setpoint is sent through the serial port (tailored for [qt-pid-scope](https://github.com/odjadane/qt-pid-scope), but easily customizable).

The design process is explained [in this video](https://www.youtube.com/watch?v=9xWMm3ugPQQ).



## Acknowledgments

- [Hardware-based activities - CTMS](https://ctms.engin.umich.edu/CTMS/index.php?aux=Index_Activities)
- [Discrete Control - B. Douglas](https://www.youtube.com/playlist?list=PLUMWjy5jgHK0MLv6Ksf-NHi7Ur8NRNU4Z)