clc
clear
close all

% Aircraft data 
S = 196.1/10.764;  
b = 21.94/3.281;
c = 9.55/3.281;
W = 16300*0.4535; % lb to kg
Ix = 3549*157.087; %slug-ft^2 to kg/m^2
Iy = 58611*157.087;
Iz = 59669*157.087;
Inertia = [Ix 0 0; 0 Iy 0; 0 0 Iz];
CG = [0.07*c 0 0]; % CG at 7% MAC
CP = [0.25*c 0 0]; % CP at 25% MAC
delta = deg2rad(4); % radians
M = 0.257;
h = 0;
[T, a, P, rho] = atmosisa(h);
U = M * a;

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

%creating fixed wing object
F104A = fixedWingAircraft('F104A',S,b,c,'PM6');

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

% creating aircraft properties
Properties = aircraftProperties('MyPlane','This aircraft is used for analysis','Fighter','Version 1.0');
F104A.Properties = Properties;

% creating aircraft environment
Environment = aircraftEnvironment("ISA",0);

% creating fixed wing state
state = fixedWingState(F104A);

state.Mass = W;
state.Inertia = Inertia;
state.CenterOfGravity = CG;
state.CenterOfPressure = CP;
state.U = U;
state.Environment = Environment;

% creating fixed wing surfaces (Aileron)
aileron = fixedWingSurface('Aileron','on','Asymmetric',[-25,25]);
Cla = Cl_da * delta;
Cna = Cn_da * delta;
aileron = setCoefficient(aileron,"Cl","Zero",Cla);
aileron = setCoefficient(aileron,"Cn","Zero",Cna);


% creating Elevator
elevator = fixedWingSurface('Elevator','on','Symmetric',[-25,25]);
CLe = CL_de * delta;
Cme = Cm_de * delta;
elevator = setCoefficient(elevator,"CL","Zero",CLe);
elevator = setCoefficient(elevator,"Cm","Zero",Cme);

% creating Rudder 
rudder = fixedWingSurface('Rudder','on','Symmetric',[-25,25]);
Cyr = Cy_dr * delta;
Clr = Cl_dr * delta;
Cnr = Cn_dr * delta;
rudder = setCoefficient(rudder,"CY","Zero",Cyr);
rudder = setCoefficient(rudder,"Cl","Zero",Clr);
rudder = setCoefficient(rudder,"Cn","Zero",Cnr);

F104A.Surfaces = [aileron elevator rudder];

% creating propulsion
propulsion = fixedWingThrust('Engine1','on','Symmetric',[0 1]);
F104A.Thrusts = propulsion;


[F M] = forcesAndMoments(F104A,state)
sys = nonlinearDynamics(F104A,state)
