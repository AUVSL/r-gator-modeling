clear all; clc;
fis  = readfis("controller.fis")

% parameters
caf  = 2.787e-4; % cornering stiffness of front tire
car  = 2.787e-4; % cornering ... rear tire
lf   = 0.5;      % cog to front axle,              [m]
lr   = 0.5;      % cog to rear axle,               [m]
m    = 12;       % bicycle weight,                 [kg]
Iz   = 2;        % z-axis moement of inertia,      [kg m^2] 
v_lon = 5.0;     % forward, longitudinal velocity, [m/s]

% inputs
theta = pi/2; % steering angle, [radians]

% get put equations of kimetmic motion into state space form
a11 = -(car      +      caf)/(m *v_lon);
a12 =  (car*lr   -   caf*lf)/(m *v_lon);
a21 =  (car*lr   -   caf*lf)/(Iz*v_lon);
a22 = -(car*lr^2 + caf*lf^2)/(Iz*v_lon);

b11 = caf/m;
b21 = caf*lf/Iz;

A = [a11, a12; a21, a22];
B = [b11; b21];
C = [1, 0; 0, 1];
D = [0; 0];

% put the state space equation into tranfer function form (output = tf * input) 
[b,a]   = ss2tf(A,B,C,D); 
v_lat   = tf(b(1,:), a)
ang_vel = tf(b(2,:), a);

% save these for later export/ use in simulink
b1 = b(1,:);
b2 = b(2,:);


% Sameer the undergrad was usng this code to understand the time response
% a transfer function which originated in the frequency domain to an input 
% step function 
syms s
[num,den] = tfdata(v_lat);
G_sym     = (poly2sym(num,s)/poly2sym(den,s));
Y_lap_sym = G_sym/s; % U(s) = 1/s for the unit step
y_time_sym = ilaplace(Y_lap_sym)
step(v_lat);

% Sameer also messed around with saving and loading waypoints for a path
waypoints_x = [0, 1, 2, 3, 4];
waypoints_y = [0, 1, 3, 4, 5];

save('gatorwaypoints.mat', 'waypoints_x', 'waypoints_y');
load('gatorwaypoints.mat');