%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Jack Lambert
% Dale Lawrence
% Aircraft Dynmaics Homework 7
% Problem 3
% Purpose: Sets Initial COnditions for each Pertubation Case and Calls ODE45
% to plot the State Variables vs time
% Date Modefied: 2/12/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ODE45 Variable Allocation
%                     X_E = z(1); % z-position, Inerital Frame 
%                     Z_E = z(2); % z-position, Inerital Frame 
%                     u_dot = z(3); % x-component of Velocity, Body Frame
%                     z_dot = z(4); % x-component of Velocity, Body Frame
%                     q_dot = z(6); % Angular Velocity about the y-axis [rad/s]
%                     theta_dot = z(5); % Pitch Angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Conditions
c1 = 0; % xE: Location in Inertial Coordinates [m]
c2 = 0; % zE: Location in Inertial Coordinates [m]
c3 = 0; % Delta U: x-comp, BF Interial Velocity [m/s]
c4 = 0; % Delta W: z-comp, BF Interial Velocity [m/s]
c5 = 0; % Delta q: y-comp, BF Angular Velocity [rad/s]
c6 = 0.1; % Delta Theta: Pitch Angle

condition = [c1 c2 c3 c4 c5 c6]; 
ks = [1, 2]; % What we are scaling the pitch stiffness by

%% State Variables vs. Time
t = [0 200]; % Larger times to see phugoid mode, shorter for short period mode


string = ["ks = 1","ks = 2"]; % Title for Varying IC's
% Phugoid Response (Longer Time)
for i = 1:2
    % Calling ODE45 
    [t,z] = ode45(@(t,y) ODEcall(t,y,ks(i)),t,condition);
    
   
    % U_E vs time
    figure
    subplot(4,1,1)
    plot(t ,z(:,3),'Linewidth',1)
    tit = sprintf('%s %s %s','State Variables of a B 747 (\Delta\theta = 0.1 [rad],',string(i),')');
    title(tit)
    ylabel('u_E [m/s]')
    
    
    % W_E vs time
    subplot(4,1,2)
    plot(t ,z(:,4),'Linewidth',1)
    ylabel('w_E [m/s]')
    
    % q vs time
    subplot(4,1,3)
    plot(t ,z(:,5),'Linewidth',1)
    ylabel('q [rad/s]')
    
    % Theta vs time
    subplot(4,1,4)
    plot(t ,z(:,6),'Linewidth',1)
    ylabel('\theta [rad]')
    xlabel('Time [s]')

end

%% Plotting Position
for i = 1:2
    [t,z] = ode45(@(t,y) ODEcall(t,y,ks(i)),t,condition);

    % xE vs zE
    figure
    subplot(3,1,1)
    plot(z(:,1) ,z(:,2),'Linewidth',1)
    tit = sprintf('%s %s %s','Position of a B747 (\Delta\theta = 0.1 [rad],',string(i),')');
    title(tit)
    xlabel('xE [m]')
    ylabel('zE [m]')
    
    % xE vs t
    subplot(3,1,2)
    plot(t ,z(:,1),'Linewidth',1)
    xlabel('time [s]')
    ylabel('xE [m]')
    
   % zE vs t
    subplot(3,1,3)
    plot(t ,z(:,2),'Linewidth',1)
    xlabel('time [s]')
    ylabel('zE [m]')

end
    


