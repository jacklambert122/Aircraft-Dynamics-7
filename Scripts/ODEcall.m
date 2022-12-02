%% Author: Jack Lambert
% ASEN 3128
% Problem 3
% Purpose: Function for ODE45 to call to calculate the State variables xE,
% zE, u_dot, w_dot, q_dot, and theta_dot. This function uses the simplified
% assumptions for the Linearized Longitudinal Dynamics Set
% Last Edited: 3/11/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dydt] = ODEcall(t,y,ks)

X_E = y(1); % x-position, Inerital Frame 
Z_E = y(2); % z-position, Inerital Frame 
u_dot = y(3); % x-component of Velocity, Body Frame
w_dot = y(4); % z-component of Velocity, Body Frame
q_dot = y(5); % y-component of Angular Velocity, Body Frame
theta_dot = y(6); % Pitch Angle 

%% State Variable Matrix for Linearized Longitudinal Set
[A_BK,theta0,u0] = Linearizedset(ks); % A matrix function based on plane and parameters
State = [u_dot, w_dot, q_dot, theta_dot]'; % Couple State Variables in Long. Set
var = A_BK*State; % Couple State Variables in Long. Set
%% Solving for Inertial Position
dydt(1) = u_dot*cosd(theta0) + w_dot*sind(theta0) - u0*theta_dot*sind(theta0); % xE
dydt(2) = -u_dot*sind(theta0) + w_dot*cosd(theta0)-u0*theta_dot*cosd(theta0); % zE
%% Solving for State Variables in the Linearized Longitudinal Set
dydt(3) = var(1); % uE
dydt(4) = var(2); % wE
dydt(5) = var(3); % q
dydt(6) = var(4); % theta

dydt = dydt'; % Inverts for ODE45   
end