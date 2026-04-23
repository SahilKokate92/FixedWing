clc
clear
close all

Coef_body = [
   -0.03;   % Cx (drag, negative along body x)
    0.00;   % Cy (side force, zero in symmetric flight)
   -0.6;    % Cz (lift, negative in body z)
    0.00;   % Cl (roll moment)
   -0.05;   % Cm (pitch moment, stable aircraft)
    0.00    % Cn (yaw moment)
];

V = 25;
rho = 1.225;
q = 0.5*rho*V^2;
CG = [0 0 0];
CP = [0.25 0 0];
m = 5; %kg
Ixx = 0.25;
Iyy = 0.30;
Izz = 0.45;
Ixz = 0.02;  
I = [ Ixx   0   -Ixz;
       0   Iyy    0;
     -Ixz   0   Izz ];
