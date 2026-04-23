clc
clear
close all

% aircraft roll dynamics 
[T,a,P,rho] = atmosisa(0);
M = 0.227;
a = 340;
V = M*a;
b = 21.94; % ft
Sw = 196.1; % ft^2
Ixx = 3549; % slug-ft^2


% lateral stability and control parameter
Cl_da = 0.039;
Clp = -0.285;
A = (0.5*rho*V^2*Sw*b)/Ixx;
B = A*Cl_da;
A1 = -A*Clp*(b/(2*V));
In = deg2rad(5);

G_num = [B];
G_den = [1 A1];
G = tf(G_num,G_den)

figure(1)
bode(G)

figure(2)
rlocus(G)