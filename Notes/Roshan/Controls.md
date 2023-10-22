CONTROL BOOT CAMP

Types of control:

Passive control - Passive vibration control does not require an outside source of power, but
instead uses the mass of the structure to mitigate vibration. Ex: tuned mass dampers, viscous dampers.

Active control - Active  control is an area of research in which the motion of a structure is controlled or modified by means of the action of a control system through some external energy supply.

We consider a state variable X which is a vector that describes all the quantities required in our system. In case of a pendulum it could be an angle or angular velocity.
 X• =Ax
We have x(t) which has a solution x(t)= e^ (At x(0))
If all eigenvalues of the system are positive then the system is stable.
In control systems, we write X• = Ax +Bu
u is  an actuator or a control knob. It could be  the voltage of the motor.
B indicates how the control knob affects the time rate change of our state.
X• =Ax where A is a matrix and X is a vector and x € R

The eigenvectors that should be used can be found out using LQR.
We  build a cost function in which Q is a matrix which tells us about the penalty that could be developed if x is not desirable.To stabilize pendulum matrix Q needs to be large.
The R vector has to be small so that we can actuate it aggressively. When we have Q and R then we will have the K matrix (u=-Kx). This is known as the Linear Quadratic Regulator control.This minimizes the cost function. 

Observability- Gives us an estimation of the full state of vector x with lesser measurements.
A system is observable if the rank of the matrix C (y=Cx) is n.
Can estimate x from any time series of y.

Model predictive control(MPC) is an optimal control technique in which the calculated control actions minimize a cost function for a constrained dynamical system over a finite, receding, horizon. At each time step, an MPC controller receives or estimates the current state of the plant.
- It is effective as it has some constraints
- it can convert a non linear system to linear.
- Optimisation
- fast hardware.

#2

![](https://lh3.googleusercontent.com/fpG2qzR6Nqm6fgSATNCndY6YMS-gjHxcx3dwHblAJjmcL0PUZfrzbDWGqa9d5C9MnQMDZ2XuakRSnCivV3vVKk7jPE9IJ5Q5oWdAWhBkHTwGiMYHErTkfT2KAM5D7TC9W1ZJxpwQBO_pmcDKVj1Qq-w)

#3

![](https://lh3.googleusercontent.com/2Ojfm8r4n_s3qHB5-jQq4QdrufzEpbdVVNZxC5M9Oed1aEHmgGt7w9bPhgaug2Ptjn34ITf76syW7miPxEPIlayy2sc4r_cJs1RSuiuUCRi7EFzZ1rKMS77UseqOhXLl3WB3UZf35tuo1VdQKbuR6LQ)

#4

![](https://lh6.googleusercontent.com/ylAbCpCJhY3fhFFnSbspC1tce7aRg8gOR795avJjaKhTtie8ztOZ0Zucqd7JciIP3Oq5bF_km6VUzY-iHH_74UIRpv-00N_XHNW4vjmU730JrIo9u-rP-T1CdRcHudY6GRutskDgI9NGUi5D8XvMFWs)

#5

![](https://lh6.googleusercontent.com/XlFQZaTcGvpZpFYfGmSbXOqJAZEHGzapoxYMJgSYeF9qplbFTn00MRf33jRDROg4CYTgTywa21Y1F1ZojConsRGNyTuiwkWWWajlVNzEksLrIf8yts67PsyVeqN2NmhNY2Lt5E7V834sJ17v9GeFhco)

#17

![](https://lh6.googleusercontent.com/TgeWoG2w5qyoJG9Y8zg9p2XqI3BRd1zNMXosWdvoHEmSHxJEYqnXDxHhpTmalM9d92fMHcEF4R0Pgv2iOIbbAKeL99ANnVhH-fUVDuXlin19eqhnhrN8f7zr3lp7Z-KhO9tN17OLGBoQEcGc26FpJXM)


