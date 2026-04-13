clc
clear
close all

% aircraft pitch dynamics 
[T,a,P,rho] = atmosisa(0);
M = 0.60;
V = 200;
c = 9.55; % ft
Sw = 196.1; % ft^2
Iyy = 58611; % slug-ft^2


% longitudinal stability and control parameter
Cmde = -1.46;
Cma = -0.64;
Cmq = -5.8;
A = (0.5*rho*V^2*Sw*c)/Iyy;
B = A*Cmde;
A1 = -A*Cmq*(c/(2*V));
A2 = -A*Cma;

G_num = [B];
G_den = [1 A1 A2];
G = tf(G_num,G_den)

figure(1)
bode(G)

figure(2)
rlocus(G)