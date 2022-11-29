clc
clear
close all

M=0.5;
g=9.81;
m=0.2;
b=0.1;
l=0.3;
I=0.006;
a1=M*l^2*m+I*M+I*m;

A=[0 , 1 , 0 , 0; ... 
   0 , (-I*b-b*l^2*m)/a1 , (m^2*g*l^2)/a1 , 0; ...
   0 , 0 , 0 , 1; ...
   0 , (-b*l*m)/a1 , (M*g*l*m+m^2*g*l)/a1 , 0];
B=[0 ; (I+l^2*m)/a1 ; 0 ; (l*m)/a1];
C=[1 , 0 , 0 , 0; 0 , 0 , 1 , 0];
D=[0 ; 0];

%part 3
max_theta=0.61;
max_x=0.55;
v=[1/(max_x^2) 1  1/(max_theta^2) 1];
Q=diag(v);
R=1;
[P,k,s]=icare(A,B,Q,R);
k2=inv(R)*B'*P;

%part 5
v2=[1/(max_x^2) 1  1/(max_theta^2) 1 1];
Q_bar=diag(v2);
C_bar=[1 , 0 , 0 , 0];
A_bar=[A , zeros(4,1) ; C_bar , zeros(1,1)];
B_bar=[B ; zeros(1,1)];
CO_bar=ctrb(A_bar,B_bar);
rank_C_bar=rank(CO_bar);

[P_bar,k_bar,s_bar]=icare(A_bar,B_bar,Q_bar,R);
%% Simulink 
% Parameters
linewidth = 2;
fontsize = 14;

% Simualtion Parameters
dt = 0.002; %integration step,
tf = 10; %final time, sec

% INTIAL CONDITONS
x_initial = [0.5 ; 0 ; deg2rad(30) ; 0];

%% Set Controller
% Set the Controller:
   % 1 = State-feedback linear model
   % 2 = .... so on
Controller = 3;

if Controller == 1
    K = k;
    K2=0;
elseif Controller == 2
    K = 0;
    K2=0;
elseif Controller ==3
   K=k_bar(1:4);
   K2=k_bar(5);
end


% Set Model
% Set the model to use: 1 = linear, 2 = nonlinear.
Model = 2;

% Set Desired State
% Set the desired state: 1 = Regulation, 2 = Setpoint Tracking.
Desired = 2;

if Desired == 1
    xd = [0 , 0 , 0 , 0];
else
    xd = [0.5 , 0 , 0 , 0];
end

% Run Simulation
[t,~,x,u1] = sim('new_Sim_Simulink');
%   x is the state vector of [x; x_dot; phi; phi_dot],  4x1 vector

x1 = x(:,1);% Cart x position plot
phi = x(:,3);
theta = rad2deg((phi+pi()))-180;

plot(t,x1)
title('X vs Time')
xlabel('Time (s)')
ylabel('X (m)')
figure
plot(t,theta)
title('Theta vs Time')
xlabel('Time(s)')
ylabel('Theta(deg)')
figure
plot(t,u1)
title('Input vs Time')
ylabel('Force (N)')
xlabel('Time (s)')

