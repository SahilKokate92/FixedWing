clc
clear
close all

% Aircraft data
W = 16300/32.1850; % slug
S = 196.1;
b = 21.94;
c = 9.55;
W = 16300; %lb
Ix = 3549; %slug-ft^2
Iy = 58611;
Iz = 59669;
Inertia = [Ix 0 0; 0 Iy 0; 0 0 Iz];
CG = [0.07*c 0 0]; % CG at 7% MAC
CP = [0.25*c 0 0]; % CP at 25% MAC

M = 0.257;
h_ft = 0; % Altitude in feet (Sea Level)

h_m = h_ft * 0.3048; % Convert feet to meters
[T_K, a_ms, P_Pa, rho_kgm3] = atmosisa(h_m);

% 1 meter = 3.28084 feet
% 1 kg/m^3 = 0.00194032 slugs/ft^3
a_fts = a_ms * 3.28084;
rho_slugs = rho_kgm3 * 0.00194032;

U = M * a_fts;
F104A = fixedWingAircraft('F104A',S,b,c,'PM4', 'UnitSystem', 'English (ft/s)');

% aerodynamic and stability coefficients
% Longitudinal
CL = 0.735;    
CD = 0.263;
CL_alpha = 3.44;   
CD_alpha = 0.45;  
Cm_alpha = -0.64;
CL_alpha0 = 0.00;
Cm_alpha0 = -1.6; 
CL_q = -5.8;

% Lateral-Directional
Cy_beta = -1.17;
Cl_beta = -0.175; 
Cn_beta = 0.50;
Cl_p = -0.285;    
Cn_p = -0.14;     
Cl_r = 0.265;   
Cn_r = -0.75;

% Control Surfaces
CL_de = 0.68;       
Cm_de = -1.46;
Cl_da = 0.039;      
Cn_da = 0.0042;
Cy_dr = 0.208;     
Cl_dr = 0.045;    
Cn_dr = -0.16;

F104A = setCoefficient(F104A, "CD", "Zero", CD);
F104A = setCoefficient(F104A, "CD", "Alpha", CD_alpha);
F104A = setCoefficient(F104A, "CL", "Zero", CL); 
F104A = setCoefficient(F104A, "CL", "Alpha", CL_alpha);
F104A = setCoefficient(F104A, "CL", "Q", CL_q);
F104A = setCoefficient(F104A, "Cm", "Zero", Cm_alpha0);
F104A = setCoefficient(F104A, "Cm", "Alpha", Cm_alpha);
F104A = setCoefficient(F104A, "CY", "Beta", Cy_beta);
F104A = setCoefficient(F104A, "Cl", "Beta", Cl_beta);
F104A = setCoefficient(F104A, "Cl", "P", Cl_p);
F104A = setCoefficient(F104A, "Cl", "R", Cl_r);
F104A = setCoefficient(F104A, "Cn", "Beta", Cn_beta);
F104A = setCoefficient(F104A, "Cn", "P", Cn_p);
F104A = setCoefficient(F104A, "Cn", "R", Cn_r);

state = fixedWingState(F104A,'UnitSystem','English (ft/s)');

state.Mass = W;
state.Inertia = Inertia;
state.CenterOfGravity = CG;
state.CenterOfPressure = CP;
state.U = U;


