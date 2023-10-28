function pendulum_test()
m = 1.5; % mass of the bob
g = 9.8;
L = 0.8; % length of the string
y_setpoint = [pi ; 0]; %stable point
y0 = [-pi/3 ; 0]; %initial point


%[t,y] = ideal_pendulum(m,g,L,y0);
[t,y] = lqr_pendulum(m,g,L, y_setpoint, y0);

 for k = 1:length(t) %initialzing for loop to animate
    draw_pendulum(y(k, :), L);
 end
end

function draw_pendulum(y,L) % function for animation
theta = y(1); % equating variable theta as a state vector y
x_pos = L*sin(theta); % positions of the bob as a function of time
y_pos = 1-L*cos(theta);
d= 0.1;
 hold on;
  clf;
  axis equal;
  rectangle('Position',[x_pos-(d/2),y_pos-(d/2),d,d],'Curvature',1,'FaceColor',[1 0 0]); % animation of pendulum bob
  line ([0 x_pos], [1 y_pos], "linestyle", "-", "color", "k"); % animation of string
  xlim([-1 1]) 
  ylim([-0.5 2])
  drawnow
  hold off
end
% A system is expressed in the form of xdot = Ax+ Bu where x is the state
% vector and u is the external input. A and B are the matrices.

function dy= pendulum_dynamics(y, m ,L, g ,u)
sin_theta = sin(y(1));
 cos_theta = cos(y(1));
  dy(1,1) = y(2);
  dy(2,1) = -g*sin_theta/L + u/(m*L^2); %lagrangian eqn for pendulum
  
end

function [t,y] = ideal_pendulum(m, g, L, y0)
time = 0:0.1:10; %time span for simulation
u = 0; %ideal pendulum so no external input
[t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, u),time,y0);
end

function [A, B] = pendulum_AB(m, g, L)
 A = [0 1; g/L 0]; % matrices A and B 
  B = [0; 1/(m*L^2)];
end
% To achieve LQR we need a cost function where Q is the matrix for
% determining penalty and R is the penalty for energy used.

function [t,y] = lqr_pendulum(m, g, L, y_setpoint, y0)
  [A,B] = pendulum_AB(m,g,L);      
  Q = [10 0; 0 10];                   
  R = 0.001;                   
  K = lqr(A,B,Q,R); 
  time = 0:0.1:10; % time span for simulation
 [t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, -K*(y-y_setpoint)),time,y0);
end
