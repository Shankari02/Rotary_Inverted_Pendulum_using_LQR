This MATLAB [Code File](.\MATLAB_simulation\inverted_pendulum.m) contains the simulation done on MATLAB for inverted pendulum.
It includes the simulation for an ideal system as well as the system stabilised using LQR (Liner Quadratic Regulator).
It consists all the state-space equations and the A and B matrices according required to stabilise our system.

The stabilisation is based on Control-Theory where in our system consists of 4 state-variables : theta_1 , theta_1_dot , theta_2 , theta_2_dot
The state-equations have been derived by using the Lagrangian function described in the function inverted_pendulum_dynamics as per the code file.
Further, we use the lqr function where in we calculate k. We feed the penalties Q and R which helps stabilise the system.
The function draw includes the animation of an inverted pendulum system.

