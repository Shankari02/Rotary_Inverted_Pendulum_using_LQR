# Rotary_Inverted_Pendulum_using_LQR

#### Introduction 

Control systems provide the foundational framework to ensure that automated systems execute their designated tasks accurately, efficiently, and optimally, even during dynamic environmental conditions. In this project, our focus is on implementing a Rotary Inverted Pendulum system using the Linear Quadratic Regulator (LQR) approach for control.

The design and mechanics of an inverted pendulum system depend on two key factors: the method of actuation and the degree of freedom (DOF) inherent to the system. Actuation methods can be categorized as linear or rotary, and the most basic controllable structure of an inverted pendulum system typically encompasses two DOFs, one for the base position and the other for managing the pendulum's angle.

The Rotary Inverted Pendulum system consists of three fundamental components: a motor, an arm, and a pendulum. In this configuration, one end of the arm is affixed to the motor's shaft, while the pendulum is connected to the shaft of a rotary encoder, enabling it to rotate freely within the vertical plane. This system architecture forms the core of our project, where we'll apply LQR control to effectively manage and stabilize the rotary inverted pendulum.

#### Basis

An **inverted pendulum** is a pendulum in which its center of mass is situated above its pivot point. This configuration renders it inherently unstable, and it would naturally topple over without external intervention. However, through the use of a control system, it can be maintained in a stable, inverted position.

The control system constantly monitors the angle of the pendulum and makes adjustments to shift the pivot point horizontally. This action effectively repositions the pivot point beneath the center of mass, preventing the pendulum from falling over and keeping it balanced. The inverted pendulum presents a classic challenge in the fields of dynamics and control theory. It serves as a benchmark for testing and evaluating various control strategies.

One common implementation involves mounting the pivot point on a cart that can be moved horizontally under the control of an electronic servo system. This setup is often referred to as a "cart and pole apparatus." The pendulum's stability is maintained by supplying a minimal, constant current at a high frequency, which induces small oscillations around its inverted position.

### Research Topics

1. **Linear Algebra**: This area is pivotal for representing various aspects of physical systems and assessing their stability, making it a foundation for many applications in engineering and science.

2. **Control Systems**: A deep understanding of control systems is essential for managing and optimizing the behavior of dynamic systems, offering opportunities to enhance the performance of a wide range of applications, from robotics to industrial automation.

### List of Software Used
* [MATLAB R2023a](https://in.mathworks.com/products/matlab.html) for modelling and simulation of inverted pendulum. 
* [SOLIDWORKS](https://www.solidworks.com/), for designing custom components.
* Embedded C

### List of Hardware Used
In our project, the following key components play crucial roles:

1. **NEMA-17 Stepper Motor**: This motor provides control over the rotational movement, making it an integral part of the system for controlling the pendulum's motion.

2. **Motor Driver**: The motor driver serves as the intermediary between the microcontroller and the stepper motor, enabling the translation of control signals into precise motor movements.

3. **Rotary Encoder**: The rotary encoder is used for measuring and monitoring the deviation of the pendulum rod from its initial position, providing feedback to the control system.

4. **Custom Designed Pendulum Rod, Horizontal Rod, and Base Plate**: These custom-designed components form the physical structure of our system, and their design impacts the system's mechanical stability and performance.

These components collectively create a system that allows for precise control and measurement of the inverted pendulum, enabling us to implement and test various control strategies effectively.
  
## Modelling the system in MATLAB

A **simulation** is a method of replicating the behavior of a real-world process or system across a span of time. It involves creating a model that evolves and represents the dynamic changes in the real system over time.

In the context of control systems, we often describe a system using the differential equation:

$$ẋ = Ax + Bu$$

Here, 'x' is the state vector, and 'u' represents an external input to the system. In an ideal system, there may be no external input (u = 0).

To control a system using the Linear Quadratic Regulator (LQR) approach, we calculate the external input as follows:

$$u = -Kx$$

Where 'K' is a control gain matrix that is derived using the LQR method. This input minimizes the cost function associated with LQR and plays a vital role in stabilizing the system, ensuring it behaves as desired.

## Implementation of Inverted Pendulum in Hardware

## **Future Work**

## Contributors
-- [Shankari](https://github.com/Shankari02)

-- [Roshan](https://github.com/RoshAd-06)

## Acknowledgements
 Special thanks to our Mentors
 
-- [Mahesh](https://github.com/Asc91)

-- [Janhavi](https://github.com/janhavi1803)

## Resources
* [Linear Algebra playlist by 3Blue1Brown](https://www.youtube.com/playlist?list=PL0-GT3co4r2y2YErbmuJw2L5tW4Ew2O5B)
* [Control Bootcamp playlist by Steve Brunton](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
* [ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
* [LEDC-PIN](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html)
