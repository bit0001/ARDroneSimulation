% This program plots the results after a test has been executed.
%% Executing simulink simulation
clear; clc;
sim('ARDrone2_ContinuousSimulation.slx');
figure;
%% Loading data generated during test
PATH_TO_POSITIONS = '../../../../../WebstormProjects/ARDroneThesis/positionFiles/';
x_n = load([PATH_TO_POSITIONS 'x_n.txt']);
y_n = load([PATH_TO_POSITIONS 'y_n.txt']);
Z_n = load([PATH_TO_POSITIONS 'z_n.txt']);

%% Plotting x
subplot(2, 2, 1)
xVectors = [t, x_ref_n, x_n];
plotAxisComparison(xVectors, 'x_{ref}_n ', 'x_n', 'x_n', 't', 'x(t)');

%% Plotting y
subplot(2, 2, 2)
yVectors = [t, y_ref_n, y_n];
plotAxisComparison(yVectors, 'y_{ref}_n ', 'y_n', 'y_n', 't', 'y(t)');

%% Plotting z
subplot(2, 2, 3)
zVectors = [t, z_ref_n, z_n];
plotAxisComparison(zVectors, 'z_{ref}_n ', 'z_n', 'z_n', 't', 'z(t)');

%% Plotting trajectory
subplot(2,2,4)
trajectories = [x_ref_n, y_ref_n, z_ref_n, x_n, y_n, z_n];
plotTrajectoryComparison(trajectories, 'x(t)', 'y(t)', 'z(t)', ...
    'Reference', 'AR.Drone 2.0', 'Discrete','Reference Path vs. Quadrotor Path');
