- ESP32 has dedicated multiple PCNT (Pulse Counter) modules. It can be used to count the number of rising or falling edges of digital input signals. 
- Each PCNT module contains an independent counter and provides multiple channels.
- Each channel can be configured and used separately to measure rising/falling edges and values of counter increases or decreases accordingly.
- Each pulse counter signal can detect edge or level of signals such as positive and negative edges or active high and low level.
- Combining both types will make each channel acts as a quadrature decoder. Most importantly each module also has a separate glitch filter which is useful to filter out noise from input signals.
Rotary encoders are commonly used to measure the angular position of rotating objects eg shafts.  The angular position gives a determination of the amount as well as type of rotation. 
- Depending on the movement of rotation, an analog or digital signal is produced.
- Rotary Encoders are of two types a) output signal  b) sensing technology.
- One interesting feature of these types of encoders is that we can rotate them endlessly as there is no starting or stopping position.
### Rotary Encoders Working

When the knob of the rotary encoder is moved, it provides us with an output signal A and B that consists of two square waves which are 90 degrees out of phase which each other. When signal A goes form positive to zero then the value of pulse B is read. Rotating the knob clockwise, results in positive pulse for signal B and rotating the knob anti-clockwise results in a negative pulse for signal B. This way the direction of rotation can be monitored by comparing both output signals through the micro-controller and also calculating the total number of pulses. The total number of pulses for every turn do not remain same and vary for each turn. Moreover, the speed can also be calculated by counting the (number of pulses per unit time (frequency).
![[Screenshot 2023-09-20 at 23-41-09 ESP32 Pulse Counter PCNT with ESP-IDF and Rotary Encoder Example.png]]
### KY-040 Pins
![[Pasted image 20231022225259.png]]

Description of pins of Rotary Encoder:![[Screenshot 2023-09-20 at 23-41-09 ESP32 Pulse Counter PCNT with ESP-IDF and Rotary Encoder Example.png]]
The CLK and DT pins produce square waves (Output A and Output B respectively) that are 90 degree out of phase with each other. These two signals are used to measure the direction of rotation (Clockwise/Anti-clockwise).