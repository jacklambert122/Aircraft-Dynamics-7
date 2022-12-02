%% Author: Jack Lambert
% ASEN 3129
% Homework 7
% Problem 3 Parts a,b
% Purpose: To control the eigen values by creating gains that give us
% desired pitch stiffness. We will see trends in the changing eigenvalues
% for the phugoid and short period mode with changing the pitch stiffness
% by a scalar factor ks
%% Airplane Parameters
% Nondimensional Derivatives
% Table 6.1 -
Cx = [-.108, .2193, 0, 0];
Cz = [-.106, -4.92, -5.921, 5.896];
Cm = [.1043, -1.023, -23.92, -6.314];

% Nondimensional Elevator Derivatives (Page 229 in Etkin)
C_x_de = -3.818*10^-6;
C_z_de = -0.3648;
C_m_de = -1.444;

% Table E.1 B747 Case 3
Alt = 40000*(0.3048); % Altitude [ft] -> [m]
[T, a, P, rho] = atmosisa(Alt); % Standard Atmosphere Properties at Alt.
W = 6.366*10^5*4.44822; % Weight [lb]->[N]
Ix_PA = 1.82*10^7*1.35581795; % Moment of Interia x-PA [slug ft^2]-> [kg m^2]
Iy_PA = 3.31*10^7*1.35581795; % Moment of Interia y-PA [slug ft^2]-> [kg m^2]
Iz_PA = 4.97*10^7*1.35581795; % Moment of Interia z-PA [slug ft^2]-> [kg m^2]
Izx_PA = 9.70*10^5*1.35581795; % Moment of Interia zx-PA [slug ft^2]-> [kg m^2]
zeta = -2.4; % Angle between Stability Axis and PA [degrees] 
I = [Ix_PA, 0,-Izx_PA;...
    0, Iy_PA,0;...
    -Izx_PA, 0, Iz_PA]; % Inertia Matrix in PA
Q_PA_SA = [cosd(zeta), 0, -sind(zeta);...
    0, 1, 0;...
    sind(zeta), 0, cosd(zeta)]; % Transformation Matrix [PA-SA]
I_SA = Q_PA_SA * I * Q_PA_SA'; % MOI in Stability axis Frame
Ix = I_SA(1,1); % Moment of Interia x-SA [kg m^2]
Iy = I_SA(2,2); % Moment of Interia y-SA [kg m^2]
Iz = I_SA(3,3); % Moment of Interia z-SA [kg m^2]
Izx = (1/2)*(Ix-Iz)*sind(2*zeta)+Izx_PA*...
    (sind(zeta)^2-cosd(zeta)^2); % Moment of Interia zx-SA [kg m^2]
CD = .043; % Coefficient of Drag
cbar = 27.31*(0.3048); % Mean Chord Length [ft]->[m]
S = 5500*(0.3048)^2; % Surface Area [ft^2]->[m^2]
g = 9.81; % Gravity Constant [m/s^2]
m = W/g; % Mass of Plane [kg]

%% Trim States
Vel = 871*(0.3048);% Velocity [ft/s] -> [m/s]
u0 = Vel; % Initial Velocity in x-coord - Stability Axis Frame (Trim State)
theta0 = 0; % Initial Pitch Angle [deg]
Cw0 = W/(.5*rho*S*u0^2); 
%% Function that Computes Dimensional Derivatives from Non-Dimenional derivatives
[X, Z, M, X_c, Z_c, M_c ] = NonDimLong(rho,u0,S,W,theta0,Cx,Cz,Cm,cbar,C_x_de,C_z_de,C_m_de); 

%% State Variable Matrix A
row1 = [X(1)/m, X(2)/m, 0, -g*cosd(theta0)];
row2 = [Z(1)/(m-Z(4)), Z(2)/(m-Z(4)), (Z(3)+m*u0)/(m-Z(4)), (-W*sind(theta0))/(m-Z(4))];
row3 = [(1/Iy)*(M(1) + ((M(4)*Z(1))/(m-Z(4))) ),...
        (1/Iy)*(M(2) + ((M(4)*Z(2))/(m-Z(4))) ),...
        (1/Iy)*(M(3) + ((M(4)*(Z(3)+m*u0))/(m-Z(4))) ),...
        -((M(4)*W*sind(theta0))/(Iy*(m-Z(4))))];
row4 = [0, 0, 1, 0];

A = [row1;row2;row3;row4];

%% Input Matrix B
% Dimensionalizing Elevator Derivative

% Compenents of B Matrix
row1_C = [X_c(1)/m, X_c(2)/m];
row2_C = [Z_c(1)/(m-Z(4)), Z_c(2)/(m-Z(4))];
row3_C = [M_c(1)/Iy + (M(4)*Z_c(1))/(Iy*(m-Z(4))), M_c(2)/Iy + (M(4)*Z_c(2))...
    /(Iy*(m-Z(4)))];
row4_C = [0, 0];

B = [row1_C;row2_C;row3_C;row4_C];

%% Part a
ks = 1:0.01:3;

figure
for i = 1:length(ks)
    A1 = [M(3)/Iy * sqrt(ks(i)), (u0*M(2)/Iy)*ks(i);
    1, 0];
    [eV1,eVal1] = eig(A1);
    modes = diag(eVal1);
    hold on
    plot(real(modes(1)),imag(modes(1)),'.r')
    plot(real(modes(2)),imag(modes(2)),'.r')
     if i == 1
         % To see initial state at ks = 1
         plot(real(modes(1)),imag(modes(1)),'-o')
         text(real(modes(1)),imag(modes(1)),' \leftarrow ks = 1')
         plot(real(modes(2)),imag(modes(2)),'-o')
         text(real(modes(2)),imag(modes(2)),' \leftarrow ks = 1')
         
     end
    
    plot([0,0],[-2,2],'--k')
    plot([-1,1],[0,0],'--k')
    
    title('Eigenvalues Varying ks')
    xlabel('Re(\lambda)')
    ylabel('Im(\lambda)')

end
hold off

%% Part b
figure
for i = 1:length(ks)
    k1 = (M(3)/M_c(1))*(1-ks(i)^(1/2));
    k2 = ((u0*M(2))/M_c(1))*(1-ks(i));
    K = [0, 0, k1, k2;
        0, 0, 0, 0];
    A_BK = A-B*K;
    
    [eV2,eVal2] = eig(A_BK);
    modes = diag(eVal2);
    plot(real(modes(1)),imag(modes(1)),'.r')
    hold on
    plot(real(modes(2)),imag(modes(2)),'.r')
    plot(real(modes(3)),imag(modes(3)),'.b')
    plot(real(modes(4)),imag(modes(4)),'.b')

    if i == 1
        % To see initial state at ks = 1
        plot(real(modes(1)),imag(modes(1)),'-o')
        text(real(modes(1)),imag(modes(1)),' \leftarrow ks = 1')
        plot(real(modes(2)),imag(modes(2)),'-o')
        text(real(modes(2)),imag(modes(2)),' \leftarrow ks = 1')
        plot(real(modes(3)),imag(modes(3)),'-o')
        text(real(modes(3)),imag(modes(3)),' \leftarrow ks = 1')
        plot(real(modes(4)),imag(modes(4)),'-o')
        text(real(modes(4)),imag(modes(4)),' \leftarrow ks = 1')
    end
    
    plot([0,0],[-2,2],'--k')
    plot([-1,1],[0,0],'--k')
    xlabel('Re(\lambda)')
    ylabel('Im(\lambda)')
    legend('Phigoid Mode', 'Short Period Mode')

end


check = 1;
