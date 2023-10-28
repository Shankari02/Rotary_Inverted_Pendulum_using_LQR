# Rotary_Inverted_Pendulum_using_LQR

#### Introduction 
Control systems form the basis for ensuring that every automatic system performs its ordained task accurately, efficiently and quickly (optimally) despite being subjected to varying environmental conditions. In this project, we will implement MPC(Model Predictive control) on Rotary Inverted Pendulum system using LQR(Linear Quadratic Regulator) as an optimizer. The mechanical structure of inverted pendulum is based on two factors, the actuation method and degree of freedom (DOF) of the system. The actuation methods are linear and rotary. The simplest controllable structure of inverted pendulum system has two DOF, one for the base position and other for the pendulum angle. 
Rotary inverted pendulum system is essentially a combination of three elements: a motor, an arm and a pendulum. In this system one end of the arm is connected to the shaft of the motor and pendulum is attached to other end of the arm by a pin joint that is allowed to rotate freely in the vertical plane.

#### Basis
An **inverted pendulum** is a pendulum that has its Centre of mass above its pivot point. It is unstable and without additional help will fall over. It can be suspended stably in this inverted position by using a control system to monitor the angle of the pole and move the pivot point horizontally back under the Centre of mass when it starts to fall over, keeping it balanced. The inverted pendulum is a classic problem in dynamics and control theory and is used as a benchmark for testing control strategies. It is often implemented with the pivot point mounted on a cart that can move horizontally under control of an electronic servo system as shown in the photo; this is called a cart and pole apparatus. The Pendulum is made stable by supplying minimum constant current at high frequency which just makes it oscillate at its position.

### Research Topics
* **Linear Algebra**, for representation of various aspects of physical systems, stability of systems, etc.
* A thorough understanding of **Control Systems**.

### List of Software Used
* [MATLAB R2023a](https://in.mathworks.com/products/matlab.html) for modelling and simulation of inverted pendulum. 
* [SOLIDWORKS](https://www.solidworks.com/), for designing custom components.
* Embedded C

### List of Hardware Used
* NEMA-17 Stepper Motor
* Motor Driver
* Rotary Encoder (used for measuring the deviation of the pendulum rod from the initial position)
* Custom designed pendulum rod, horizontal rod and base plate
  
## Modelling the system in MATLAB
A **simulation** is the imitation of the operation of a real-world process or system over time.  Simulation represents the evolution of the model over time.
We consider any system to be of the form
_ẋ_ = Ax + Bu
where x is the state vector and u is the external input provided.
In an ideal system, we do not provide any external input u.
For a system to be controlled using LQR, the external input is given by $$u = -Kx$$
This minimises the cost function of LQR and helps in stabilising the system.

## Implementation of Inverted Pendulum in Hardware

## **Future Work**

## Contributors
-- [Shankari](https://github.com/Shankari02)

-- [Roshan](https://github.com/RoshAd-06)

## Acknowledgements
 Special thanks to our Mentors
 
-- [Mahesh]()

-- [Janhavi]()

## Resources
* [Linear Algebra playlist by 3Blue1Brown](https://www.youtube.com/playlist?list=PL0-GT3co4r2y2YErbmuJw2L5tW4Ew2O5B)
* [Control Bootcamp playlist by Steve Brunton](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
* [ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
* [LEDC-PIN](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html)
