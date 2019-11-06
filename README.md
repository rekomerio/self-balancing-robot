# Self balancing robot

Robots purpose is to simply try to keep itself up. When pushed, it will also slow down the gained speed.  

### [PID controller for this project can be found here](https://github.com/rekomerio/simple-pid)

### Parts

**Required:**

- 1x Arduino Nano
- 2x DRV8825 Stepper motor driver
- 2x Nema 17 Stepper motor
- 1x MPU6050 Gyroscope / accelerometer
- 1x Stepdown voltage regulator
- 2x 100uF 50V Capacitor for DRV8825 to eliminate IC voltage spikes
- 1x 11.1V 2100mAh Li-Po battery
- 2x Tires
- 1x Frame

**Optional:**
- 1x RGB LED
- 2x 220 ohm resistor for the LED
- 2x Buttons for PID adjustment / mode changing / whatever

### Pins

- D2 to left motor pulse
- D3 to left motor direction
- D4 to right motor pulse
- D5 to right motor direction
- D8 to RGB LED green pin
- D9 to RGB LED red pin
- D11 to button 1
- D12 to button 2

### Click image for video

[![Robot](https://raw.githubusercontent.com/rekomerio/self-balancing-robot/master/img/self-balancing-robot.jpg)](http://www.youtube.com/watch?v=9eCU7sBP9oE "Robot")
