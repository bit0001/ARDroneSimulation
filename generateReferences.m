% This script is used for generating the trajectory
% that the AR.Drone 2.0 is going to follow.
% Three txt files are generated after the simulation is run using the
% references x_ref_n, y_ref_n and z_ref_n.
clear;
clc;

PATH = ['../../../../../PycharmProjects/ROSAutonomousFlight/catkin_ws/src/' ...
    'ardrone_numeric_method_controller/scripts/referenceAndConstantFiles/'];

sim('ARDrone2_ContinuousSimulation.slx');
dlmwrite([PATH 'x_ref_n.txt'], x_ref_n);
dlmwrite([PATH 'y_ref_n.txt'], y_ref_n);
dlmwrite([PATH 'z_ref_n.txt'], z_ref_n);
dlmwrite([PATH 'x_ref_np1.txt'], x_ref_np1);
dlmwrite([PATH 'y_ref_np1.txt'], y_ref_np1);
dlmwrite([PATH 'z_ref_np1.txt'], z_ref_np1);

[K_V_XY, K_V_Z, K_OMEGA_PSI] = getControlConstants();
controlConstants = [K_V_XY K_V_Z K_OMEGA_PSI]';
dlmwrite([PATH 'control_constants.txt'], controlConstants);
