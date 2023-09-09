function pulley_main()
  m1 = 0.21;
  m2 = 0.25;
  m3 = 0.38; % mass of the 3 blocks

  % pulleys are considered to be massless
  
  g = 9.8;
  rA = 0.2; % radius of the pulleys
  rB = 0.1;
  y_setpoint = [0.6 ; 0; 0.8; 0]; 
  y0 = [0.4 ; 0; 0.2; 0];

 %[t,y] = sim_pulley(m1, m2, m3, g, rA, rB, y0)
[t,y] = lqr_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0);

 % for loop for animation
  for k = 1:length(t)
    draw_complex_pulley(y(k, :));
  end

  end

  function draw_complex_pulley(y) % function to animate the pulley
  ml = 0.2; % constants
  mb = 0.1;
  L_A = 1.3; % length of the strings
  L_B = 1.1;

  rA = 0.2;
  rB = 0.1;

  x1 = y(1);
  x2 = L_A - y(1);
  y1 = y(3);
  y2 = L_B - y(3);

  pulley_A_pos = {0, 1};
  pulley_B_pos = {(-rA), 1-x2};
  m1_pos = {(rA), 1-x1};
  m2_pos = {(-rA-rB), 1-x2-y2};
  m3_pos = {(-rA+rB), 1-x2-y1};
  x1_string = {(rA), 1, (rA), (1-x1)};
  x2_string = {(-rA), 1, (-rA), (1-x2)};
  y1_string = {-(rA - rB), 1-x2, (-(rA - rB)), 1-x2-y1};
  y2_string = {(-rA - rB), 1-x2, (-(rA + rB)), 1-x2-y2};

  hold on;      
   clf;
  axis equal;
  rectangle('Position',[pulley_A_pos{1}-(rA),pulley_A_pos{2}-(rA) 2*rA, 2*rA],'Curvature',1,'FaceColor',[1 0 1]); % Pulley1
  rectangle('Position',[pulley_B_pos{1}-(rB),pulley_B_pos{2}-(rB),2*rB, 2*rB],'Curvature',1,'FaceColor',[1 0 1]); % Pulley2
  rectangle('Position',[m1_pos{1}-(ml/2),m1_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0 0 1]); % block 1
  rectangle('Position',[m2_pos{1}-(ml/2),m2_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0.5 1 0]); % block 2
  rectangle('Position',[m3_pos{1}-(ml/2),m3_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0.5 1 1]); % block 3
  line ([x1_string{1} x1_string{3}], [x1_string{2} x1_string{4}], "linestyle", "-", "color", "k"); 
  line ([x2_string{1} x2_string{3}], [x2_string{2} x2_string{4}], "linestyle", "-", "color", "k");
  line ([y2_string{1} y2_string{3}], [y2_string{2} y2_string{4}], "linestyle", "-", "color", "k");
  line ([y1_string{1} y1_string{3}], [y1_string{2} y1_string{4}], "linestyle", "-", "color", "k");
  xlim([-1 1]);
  ylim([-1 1.3]);
  drawnow
  hold off
  end

  function dy = complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, u) % langragian equations for the system
  dy(1,1) = y(2);
  dy(2,1) = (-(m2-m3)^2*g + u(2)*(m2-m3)/rB + (m2-m3)*(-m1+m2+m3)*g + u(1)*(m2-m3))/(m2*rA*(m1 + 5*m3));
  dy(3,1) = y(4);
  dy(4,1) = ((m2-m3)*dy(2,1) - (m2 - m3)*g + u(2)/rB)/(m2+m3);
  end

% A system is expressed in the form of xdot = Ax+ Bu where x is the state
% vector and u is the external input. A and B are the matrices.

function [t,y] = sim_pulley(m1, m2, m3, g, rA, rB, y0)
  tspan = 0:0.1:10;               
  u = [0; 0];                        
  [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, u),tspan,y0);
end

function [A,B] = complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB)
  A = [0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
  B = [0 0;(m2-m3)/(m2*rA*(m1+5*m3)) (m2-m3)/(m2*rA*rB*(m1+5*m3));0 0;(m2-m3)^2/((m2*rA*(m1+5*m3))*(m2+m3)) ((1/(rB*(m2+m3)))+(((m2-m3)^2)/(m2*rA*rB*(m1+5*m3)*(m2+m3))))];
end

% To achieve LQR we need a cost function where Q is the matrix for
% determining penalty and R is the penalty for energy used.

function [t,y] = lqr_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
  [A, B] = complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB);
  Q = [10 0 0 0;0 5 0 0;0 0 10 0;0 0 0 5];
  R = [0.001 0;0 0.001];
  K = lqr(A,B,Q,R);
  tspan = 0:0.1:10;                 
  [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, -K*(y - y_setpoint)),tspan,y0);
end
