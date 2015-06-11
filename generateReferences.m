% This script is used for generating the trajectory
% that the AR.Drone 2.0 is going to follow.
% Three txt files are generated after the simulation is run using the
% references x_ref_n, y_ref_n and z_ref_n.
clear;
clc;

sim('ARDrone2_ContinuousSimulation.slx');
dlmwrite('x_ref_n.txt', x_ref_n);
dlmwrite('y_ref_n.txt', y_ref_n);
dlmwrite('z_ref_n.txt', z_ref_n);
dlmwrite('x_ref_np1.txt', x_ref_np1);
dlmwrite('y_ref_np1.txt', y_ref_np1);
dlmwrite('z_ref_np1.txt', z_ref_np1);
