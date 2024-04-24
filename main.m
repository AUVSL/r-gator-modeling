clear all; clc;
fis  = readfis("controller.fis");

% parameters
caf  = 2.787e-4; % cornering stiffness of front tire
car  = 2.787e-4; % cornering ... rear tire
lf   = 0.5;      % cog to front axle,              [m]
lr   = 0.5;      % cog to rear axle,               [m]
m    = 12;       % bicycle weight,                 [kg]
Iz   = 2;        % z-axis moement of inertia,      [kg m^2] 

params = [0, caf, car, lf, lr, m, Iz];

% run the simulation
out = sim("simulation.slx").simout.data;
plot(out(1,:), out(2,:))
