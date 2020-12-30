%% Clean Up
clearvars;  % Clear the workspace
clc;  % Clear the command window
close('all');  % Close the open figures
Simulink.sdi.clear;  % Clear the simulink data inspector

%% Simulation parameters
simPrm.solTyp = 'Fixed-step';  % Solver type
simPrm.sol    = 'ode3';  % Solver type 2
simPrm.dt  = 0.001; % Integration step size
simPrm.tEnd = 20; % Simulation end time
simPrm.t = 0:simPrm.dt:simPrm.tEnd;  %simulation time goes from 0 to 20

%% Physical Constancts
g  = 9.81;   % grav acceleration in m/s^2

%% Problem Statement Parameters
% Motor
Motor.Ka =  1; % Torque Constant(Nm/A)
Motor.Ra =  0.1;   % Armature resistance(ohm)
Motor.La =  0.018; % Inductance (H)
Motor.Jm = 5;  % rotational moment of Inertia(Kg-m2)

% Gear & Winch
Gear.R1 =  0.05; % Driving gear radius (m)
Gear.R2 =  0.2;   % Driven gear radius (m)
Gear.Rw =  0.15; % Winch radius (m)

% Spring and Mass
Load.M = 1000; % Mass of elevator car (Kg)
Load.K = 75000; % Stiffness of suspension spring (N/m)

%% Calculated Parameters
calc.N = Gear.R2/Gear.R1; % Gear Ratio

Motor.Ke = Motor.Ka;  %  Back emf constant= Torque constant

calc.Z1 = -Motor.Ra/Motor.La;
calc.Z2 = -Motor.Ke/Motor.La;
calc.Z3 = (Motor.Ka*calc.N*calc.N)/Motor.Jm;
calc.Z4 = -(Load.K*Gear.Rw*Gear.Rw)/Motor.Jm;
calc.Z5 = Load.K*calc.N*Gear.Rw/Motor.Jm;
calc.Z6 = (Load.K*Gear.Rw)/(Load.M*calc.N);
calc.Z7 = -Load.K/Load.M;

%% Matrices
      
A=[calc.Z1 calc.Z2     0     0   0
   calc.Z3    0     calc.Z4  0 calc.Z5
      0       1        0     0   0
      0       0     calc.Z6  0 calc.Z7
      0       0        0     1   0     ]; % A Matrix
     
B=[1/Motor.La; 0; 0; 0; 0]; % B Matrix

C=[0 0 0 0 1
   1 0 0 0 0
   0 1 0 0 0
   0 0 1 0 0
   0 0 0 1 0
   0 0 0 0 1]; % C Matrix

D=[0;0;0;0;0;0]; % D Matrix
  

%% Simulate the math model
set_param('HW_1_VM','SolverType',simPrm.solTyp);  % set this solver type in simulink parameters 
set_param('HW_1_VM','Solver',simPrm.sol);     % set this integration method in simulink solver
SimOut = sim('HW_1_VM','SignalLoggingName','sdata'); % Simulate the math model and save the data

%% Extract Data for plotting
Voltage = 1;  % Variable for extracting the data of voltage signal
Pos_NL = 2;
Output = 3;  % Variable for extracting the output data

% Results 
Results.Pos = SimOut.sdata{3}.Values.Data(:,1); % Linear model position
Results.Current = SimOut.sdata{3}.Values.Data(:,2); % Armature current   values
Results.AngVel = SimOut.sdata{3}.Values.Data(:,3);  % Angular Velocity values
Results.AngPos = SimOut.sdata{3}.Values.Data(:,4);  % Angualr Position values
Results.Vel = SimOut.sdata{3}.Values.Data(:,5);   % Velocity of elevator values
Results.PosNL = SimOut.sdata{2}.Values.Data(:,1);  % Non Linear model position
%% Plot the results
figure(1);
plot(simPrm.t,Results.Pos,'r','LineWidth',0.8);
hold on
xticks(0:1:20);  % Give the limits of both axis
yticks(0:5:80);
title("Displacement of Elevator car actuated by PMDC"); % Give the title
xlabel('Time (s)');  % Give X label as Time in seconds
ylabel('Displacement (m)');  % Give Y label as displacement in meters

figure(2);
plot(simPrm.t,Results.Current,'b','LineWidth',0.8);
hold on
xticks(0:1:20);  % Give the limits of both axis
yticks(-1200:200:1200);
title("Armature current of PMDC vs Time"); % Give the title
xlabel('Time (s)');  % Give X label as Time in seconds
ylabel('Current (A)');  % Give Y label as displacement in meters

figure(3);
plot(simPrm.t,Results.AngVel,'b','LineWidth',0.8);
hold on
xticks(0:1:20);  % Give the limits of both axis
yticks(-2:0.5:5);
title("Angular Velocity of DC Motor shaft vs Time"); % Give the title
xlabel('Time (s)');  % Give X label as Time in seconds
ylabel('Angular Velocity (rad/s)');  % Give Y label as displacement in meters

figure(4);
plot(simPrm.t,Results.AngPos,'b','LineWidth',0.8);
hold on
xticks(0:1:20);  % Give the limits of both axis
yticks(0:5:40);
title("Angular Velocity of DC Motor shaft vs Time"); % Give the title
xlabel('Time (s)');  % Give X label as Time in seconds
ylabel('Angular Position (rad)');  % Give Y label as displacement in meters

figure(5);
plot(simPrm.t,Results.Vel,'b','LineWidth',0.8);
hold on
xticks(0:1:20);  % Give the limits of both axis
yticks(-3:0.5:10);
title("Translational Velocity of Elevator hoist vs Time"); % Give the title
xlabel('Time (s)');  % Give X label as Time in seconds
ylabel('Velocity (m/s)');  % Give Y label as displacement in meters

figure(6);
plot(simPrm.t,Results.Pos,'r','LineWidth',0.8);
hold on
plot(simPrm.t,Results.PosNL,'k','LineWidth',0.01);
xticks(0:1:20);  % Give the limits of both axis
yticks(0:10:120);
title("Displacement of Elevator car actuated by PMDC"); % Give the title
xlabel('Time (s)');  % Give X label as Time in seconds
ylabel('Displacement (m)');  % Give Y label as displacement in meters
legend('Linear Model','Non Linear Model'); % Legends