clc
clear
close all

S = 0.5;
b = 2.0;
c = 0.25;
m = 5; %kg
Ixx = 0.25;
Iyy = 0.30;
Izz = 0.45;
Ixz = 0.02;  
I = [ Ixx   0   -Ixz;
       0   Iyy    0;
     -Ixz   0   Izz ];
mass = 5;
u = 25;
h = 100;
[T a P rho] = atmosisa(h);
q = 0.5*rho*u^2;

% creating control surfaces
elevator = fixedWingSurface("Elevator", "on","Symmetric",[-20,20]);
elevator.Coefficients = fixedWingCoefficient("Elevator");
aileron = fixedWingSurface("Aileron", "on", "Asymmetric", [-20,20]);
aileron.Coefficients = fixedWingCoefficient("Aileron");
rudder = fixedWingSurface("Rudder", "on", "Symmetric", [-20,20]);
rudder.Coefficients = fixedWingCoefficient("Rudder");

% creating Wing and stabilizers
wing = fixedWingSurface("Wing","Surfaces", aileron);
horizontalStabilizer = fixedWingSurface("HorizontalStabilizer", "Surfaces", elevator);
verticalStabilizer = fixedWingSurface("VerticalStabilizer","Surfaces", rudder);

% creating thrust
propeller = fixedWingThrust("Propeller","on","Symmetric",[0,1],"Coefficients", fixedWingCoefficient("Propeller"));

% creating UAV property
UAV_Properties = Aero.Aircraft.Properties("Name","MY_UAV","Type","Small Reseach UAV", ...
    "Version","1.0","Description","Example for nonLinearDynamics");

% creating fixed Wing Aircraft
MYUAV = fixedWingAircraft("MyUAV",S,b,c,"6DOF","UnitSystem","Metric",...
    "Surfaces",[wing,horizontalStabilizer,verticalStabilizer],"Thrusts",propeller);

% adding aircraft coefficients
BodyCoefficients = {
    'CD', 'Zero', 0.027;
    'CL', 'Zero', 0.3;
    'Cm', 'Zero', 0.0;
    'CD', 'Alpha', 0.0;
    'CL', 'Alpha', 4.5;
    'Cm', 'Alpha', -0.5;
    'CD', 'AlphaDot', 0
    'CL', 'AlphaDot',  0;
    'Cm', 'AlphaDot', 0;
    'CD', 'Q', 0;
    'CL', 'Q', 0;
    'Cm', 'Q', -8.0;
    'CY', 'Beta', -0.98;
    'Cl', 'Beta', -0.12;
    'Cn', 'Beta', 0.25;
    'CY', 'P', 0;
    'Cl', 'P', -0.5;
    'Cn', 'P', 0.0;
    'CY', 'R', 0.0;
    'Cl', 'R', 0.0;
    'Cn', 'R', -0.3;
    };
 
MYUAV = setCoefficient(MYUAV, BodyCoefficients(:, 1), BodyCoefficients(:, 2), BodyCoefficients(:, 3));

% adding control surfaces coefficient
AileronCoefficients = {
    'CY', 'Aileron', 0;
    'Cl', 'Aileron', -0.15;
    'Cn', 'Aileron', 0.0;
    };
RudderCoefficients = {
    'CY', 'Rudder', 0.15;
    'Cl', 'Rudder', 0.0;
    'Cn', 'Rudder', -0.1;
    };
ElevatorCoefficients = {
    'CD', 'Elevator', 0;
    'CL', 'Elevator', 0.43;
    'Cm', 'Elevator', -1.2;
    };
PropellerCoefficients = {
    'CD', 'Propeller', -10.1200;
    };

MYUAV = setCoefficient(MYUAV, AileronCoefficients(:, 1), AileronCoefficients(:, 2), AileronCoefficients(:, 3), "Component", "Aileron");
MYUAV = setCoefficient(MYUAV, ElevatorCoefficients(:, 1), ElevatorCoefficients(:, 2), ElevatorCoefficients(:, 3), "Component", "Elevator");
MYUAV = setCoefficient(MYUAV, RudderCoefficients(:, 1), RudderCoefficients(:, 2), RudderCoefficients(:, 3), "Component", "Rudder");
MYUAV = setCoefficient(MYUAV, PropellerCoefficients(:, 1), PropellerCoefficients(:, 2), PropellerCoefficients(:, 3), "Component", "Propeller");

% creating aircraft environment
Environment = aircraftEnvironment(MYUAV,"ISA",h);

% creating Aircraft state
Cruise = fixedWingState(MYUAV,"Environment",Environment,"UnitSystem","Metric","Mass",mass,"AltitudeMSL",h,"U",u);
Cruise.Inertia.Variables = [
    Ixx, 0,   -Ixz    ;
    0  , Iyy, 0   ;
    -Ixz  , 0   , Izz;
    ];

Cruise.CenterOfGravity = [0.264, 0 , 0] .* MYUAV.ReferenceLength;
Cruise.CenterOfPressure = [0.25, 0, 0] .* MYUAV.ReferenceLength;

% creating control state
Cruise = setupControlStates(Cruise, MYUAV);

[F,M] = forcesAndMoments(MYUAV,Cruise);
x_dot = nonlinearDynamics(MYUAV, Cruise);

[stability, derivatives] = staticStability(MYUAV, Cruise);

