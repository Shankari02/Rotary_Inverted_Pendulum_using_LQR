% m - mass of block
% k - spring constant
% y0 - initial point
% y_setpoint - stable point

function spring_test() % main function
  m = 0.5;
  k = 0.8;
  y0 = [1; 0];
  y_setpoint = [0; 0];

% [t,y] = ideal_spring(m,k,y0); % calling out ideal spring function
 [t,y] = lqr_mass_spring(m, k, y_setpoint, y0); % calling out lqr controlled spring function

% for loop for animation
  for k = 1:length(t) 
    drawspring(y(k, :));
  end

end

% draw spring function for animation
function drawspring(y)
  l = 0.2; 
  b = 0.2;
  x_pos = y(1);
  y_pos = 0;
  hold on;
  clf;
  axis equal;
  rectangle('Position',[x_pos-l/2,y_pos,l,b],'Curvature',0,'FaceColor',[0.5 0 1]); % block
  line ([0 0], [0 0.6], "linestyle", "-", "color", "k"); % equibrium position
  line ([-1 x_pos], [(y_pos+b)/2 (y_pos+b)/2], "linestyle", "--", "color", "k"); % spring
  text(-0.05, 0.65, "Eqbm Pt")
  xlim([-1.2 1])
  ylim([0 1])
  drawnow
  hold off;
end
% A system is expressed in the form of xdot = Ax+ Bu where x is the state
% vector and u is the external input. A and B are the matrices.

function dy = springdynamics(y, m, k, u)
  dy(1,1) = y(2);
  dy(2,1) = -k*y(1)/m + u/m; % langragian equation for spring mass system
end

function [t,y] = ideal_spring(m, k, y0)
time = 0:0.1:10;
u=0;
[t,y] = ode45(@(t,y)springdynamics(y, m, k, u),time,y0);
end

function [A,B] = spring_AB(m, k)
  A = [0 1; -k/m 0];
  B = [0; 1/m];
end

% To achieve LQR we need a cost function where Q is the matrix for
% determining penalty and R is the penalty for energy used.

function [t,y] = lqr_mass_spring(m, k, y_setpoint, y0)
  [A,B] = spring_AB(m,k); 
  Q = [1 0; 0 1];               
  R = 0.001;               
  K = lqr(A,B,Q,R);       
  time = 0:0.1:50;
  [t,y] = ode45(@(t,y)springdynamics(y, m, k,-K*(y-y_setpoint)),time,y0);
end

