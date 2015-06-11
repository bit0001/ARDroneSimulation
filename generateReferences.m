% This script is used for generating the trajectory
% that the AR.Drone 2.0 is going to follow.
% Three txt files are generated after the simulation is run using the
% references x_ref_n, y_ref_n and z_ref_n.

sim('ARDrone2_ContinuousSimulation.slx');
dlmwrite('x_ref.txt', x_ref_n);
dlmwrite('y_ref.txt', y_ref_n);
dlmwrite('z_ref.txt', z_ref_n);
