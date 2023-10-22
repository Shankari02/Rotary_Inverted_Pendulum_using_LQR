A quadrature encoder is an incremental encoder with two out-of-phase output channels used in many general automation applications where sensing the direction of movement is required. Each channel provides a specific number of equally spaced pulses per revolution (PPR), and the direction of motion is detected by the phase relationship of one channel leading or trailing the other channel.

## **Quadrature encoder signal**
Rotary sensor has a disk (wheel) that is attached on a shaft. Their special marks are located on its surface in contact zone. Such labels are transparent, and the disk surface between them is made of impenetrable material.
![](https://eltra-encoder.eu/files/uploads/News/Optical-quadrature-encoder-working-principle.jpg)

In the case of a **linear quadrature encoder**, a scale replaces the measuring disk. Special reader (slider) travels along it. A quadrature encoder pulse in the form of a light beam comes from the source and, depending on the opposite type of surface (solid or opaque), falls or does not fall into the receiver (detector).
## **Quadrature encoding**

![](https://eltra-encoder.eu/files/uploads/News/Quadrature-encoder-signal-scheme.jpg)
Rotary quadrature encoder is **digital IC**  so it uses a binary code. Square waves show digital signals at a high (1) and a low (0) state. Binary code is needed to transfer data through interfaces (USB) and protocols to control systems.
## **Position and velocity determination**

The number of pulses per one full disk revolution depends on the number of marks on its surface. This principle is a basic definition of encoder resolution. Thus, two output signals characterize the number of pulses for a certain time. Using these values, a control system can determine the angular velocity, distance and acceleration.!![[Pasted image 20231022225135.png]]
## **Direction**
The wheel can rotate in both directions.We need two points and two lines instead of one.If the encoder moves forward (clockwise), line A is at the highest point (1) and channel B is at the lowest point (0). We get a combination of 1 and 0. The next one is 11, 00, etc.
If we start from the same point in a different direction (counterclockwise), we see that first the line will be at the highest point (11) and then the following combinations are: 01,00, etc.
By comparing the difference of points in each period, we get a unique code that indicates the order in which combinations of digits are received. ![](https://eltra-encoder.eu/files/uploads/News/Quadrature-encoders-direction-determition.jpg)
The micro controller or position counter can determine which direction the disc is spinning by analyzing such data. After the decoding, such information enters the control systems or is displayed in the usual format on the display.