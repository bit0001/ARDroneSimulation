%%%%%%%%%%%%%%%%%%%%%%%AR.DRONE 2.0 DISCRETE MODEL%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = ARDrone2_Discrete_Model(input)
%% Constants
% Sampling time
T0 = 0.1;

%% Load Inputs
V_xy_n = input(1);
V_z_n = input(2);
omega_psi_n = input(3);
t = input(4);

%% Load values computed in the previous iteration
values_nm1 = load('values_nm1.txt', 'values_nm1');

%% Index
index = t / T0;

%% values at n - 1
x_nm1 = (index == 0) * 0 + ~(index == 0) * values_nm1(1);
y_nm1 = (index == 0) * 0 + ~(index == 0) * values_nm1(2);
z_nm1 = (index == 0) * 0 + ~(index == 0) * values_nm1(3);
psi_nm1 = (index == 0) * 0 + ~(index == 0) * values_nm1(4);
V_xy_nm1 = (index == 0) * 0 + ~(index == 0) * values_nm1(5);
V_z_nm1 = (index == 0) * 0 + ~(index == 0) * values_nm1(6);
omega_psi_nm1 = (index == 0) * 0 + ~(index == 0) * values_nm1(7);

%% Mobile Robot Model
% at n - 1
psi_dot_nm1 = omega_psi_nm1;
x_dot_nm1 = computeXDot(V_xy_nm1, psi_nm1);
y_dot_nm1 = computeYDot(V_xy_nm1, psi_nm1);
z_dot_nm1 = V_z_nm1;

% at n
theta_dot_n = omega_psi_n;
psi_n = trapezoidalIntegration(psi_nm1, psi_dot_nm1, theta_dot_n, T0);
x_dot_n = computeXDot(V_xy_n, psi_n);
y_dot_n = computeYDot(V_xy_n, psi_n);
z_dot_n = V_z_n;
x_n = trapezoidalIntegration(x_nm1, x_dot_nm1, x_dot_n, T0);
y_n = trapezoidalIntegration(y_nm1, y_dot_nm1, y_dot_n, T0);
z_n = trapezoidalIntegration(z_nm1, z_dot_nm1, z_dot_n, T0);

%% Outputs
output = [x_n y_n  z_n psi_n];

%% Save variables in a file
values_nm1 = [x_n y_n z_n psi_n V_xy_n V_z_n omega_psi_n];
save('values_nm1.txt','-ascii', 'values_nm1');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%