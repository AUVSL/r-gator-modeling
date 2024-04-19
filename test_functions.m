caf = 2.787e-4; % cornering stiffness of front tire
car = 2.787e-4; % cornering ... rear tire
lf  = 0.5;      % cog to front axle,              [m]
lr  = 0.5;      % cog to rear axle,               [m]
m   = 12;       % bicycle weight,                 [kg]
Iz  = 2;        % z-axis moement of inertia,      [kg m^2] 

% inputs
str_ang = pi/2;    % steering angle, [radians]
v_lon = 5.0; % forward, longitudinal velocity, [m/s]
v_lat = 1.0;
w = 0.5;
params = [caf, car, lf, lr, m, Iz];

[dv_lat, dw] = forward_dynamcis(v_lat, w, v_lon, str_ang, params)
[dX, dY] = new_position(v_lat, v_lon, str_ang)

function [dv_lat, dw] = forward_dynamcis(v_lat, w, v_lon, str_ang, params)
    % dv_lat = -(car + caf)/(m *v_lon) * v_lat + (car*lr - caf*lf)/(m *v_lon) * w- v_lon * w + caf/m * str_ang; 
    % dw     = (car*lr - caf*lf)/(Iz*v_lon) * v_lat - (car*lr^2 + caf*lf^2)/(Iz*v_lon) * w + caf*lf/Iz  * str_ang;
    
    % parameters
    caf = params(1); car = params(2); lf  = params(3); lr  = params(4); 
    m   = params(5); Iz  = params(6);

    % get put equations of kimetmic motion into state space form
    a11 = -(car + caf)/(m *v_lon);
    a12 =  (car*lr   -   caf*lf)/(m *v_lon) - v_lon;
    a21 =  (car*lr   -   caf*lf)/(Iz*v_lon);
    a22 = -(car*lr^2 + caf*lf^2)/(Iz*v_lon);
    
    b11 = caf/m;
    b21 = caf*lf/Iz;
    
    A = [a11, a12; a21, a22];
    B = [b11; b21];
    
    dX = A * [v_lat; w] + B * str_ang;

    dv_lat = dX(1); 
    dw = dX(2);
end


function [dX, dY] = new_position(v_lat, v_lon, str_ang)
    R_gl = [cos(str_ang), sin(str_ang);
           -sin(str_ang),  cos(str_ang)].';
    dV = R_gl * [v_lon; v_lat];
    
    dX = dV(1); 
    dY = dV(2);
end