function output = commonController(input)
    %% Constants
    T0 = getSampleTime();
    [K_V_XY, K_V_Z, K_OMEGA_PSI] = getControlConstants();
    %% Inputs
    x_n = input(1);
    y_n = input(2);
    z_n = input(3);
    psi_nm1 = input(4);
    psi_ez_nm1 = input(5);
    t = input(6);
    %% Controller Logic
    % The reference trajectory is the preestablish trajectory
    % the quadrotor is supposed to follow.

    % Reference Trajectory at n
    [x_ref_n, y_ref_n, z_ref_n]  = getReferences(t);

    % Reference Trajectory at n + 1
    [x_ref_np1, y_ref_np1, z_ref_np1] = getReferences(t + T0);

    psi_ez_n = atan((y_ref_np1 - K_V_XY * (y_ref_n - y_n) - y_n) /...
        (x_ref_np1 - K_V_XY * (x_ref_n - x_n) - x_n));

    % Control Actions
    V_xy_n = (1 / T0) * ((x_ref_np1 - K_V_XY * (x_ref_n - x_n) - x_n) ...
        * cos(psi_ez_n) + (y_ref_np1 - K_V_XY * (y_ref_n - y_n) - y_n)...
        * sin(psi_ez_n));

    V_z = (1 / T0) * (z_ref_np1 - K_V_Z * (z_ref_n - z_n) - z_n);

    omega_psi_n = (1 / T0) * (psi_ez_n - K_OMEGA_PSI * (psi_ez_nm1 ...
        - psi_nm1) - psi_nm1);
    
    %% Outputs
    output = [V_xy_n, V_z, omega_psi_n, psi_ez_n, x_ref_n, y_ref_n, z_ref_n];
end