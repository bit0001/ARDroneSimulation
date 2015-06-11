%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This program plots the simulation results.
%% Executing simulink simulation
sim('ARDrone2_Comparison.slx');
figure;

%% Plotting x
subplot(2, 2, 1)
xVectors = [t, x_ref_n, x_n_c, x_n_d];
plotAxisComparison(xVectors, 'x_{ref}_n ', 'x_n_c', ...
    'x_n_d', 't', 'x(t)');

%% Plotting y
subplot(2, 2, 2)
yVectors = [t, y_ref_n, y_n_c, y_n_d];
plotAxisComparison(yVectors, 'y_{ref}_n ', 'y_n_c', ...
    'y_n_d', 't', 'y(t)');

%% Plotting z
subplot(2, 2, 3)
zVectors = [t, z_ref_n, z_n_c, z_n_d];
plotAxisComparison(zVectors, 'z_{ref}_n ', 'z_n_c', ...
    'z_n_d', 't', 'z(t)');

%% Plotting trajectory
subplot(2,2,4)
trajectories = [x_ref_n, y_ref_n, z_ref_n, x_n_c, y_n_c, ...
    z_n_c, x_n_d, y_n_d, z_n_d];
plotTrajectoryComparison(trajectories, 'x(t)', 'y(t)', 'z(t)', ...
    'Reference', 'Continuous', 'Discrete','Quadrotor Path vs Reference Path');

%% Plotting psi
figure;
subplot(2, 1, 1)
plotAxisComparison([t, psi_ez_n_c, psi_n_c], 'Ψ_{ez}_{nc}', 'Ψ_{nc}',  ...
    '', 't', 'Ψ(t)');
subplot(2, 1, 2)
plotAxisComparison([t, psi_ez_n_d, psi_n_d], 'Ψ_{ez}_{nd}', 'Ψ_{nd}',  ...
    '', 't', 'Ψ(t)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%