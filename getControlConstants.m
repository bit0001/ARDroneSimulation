function [K_V_XY, K_V_Z, K_OMEGA_PSI] = getControlConstants()
    % let k be a control constant, where 0 < k < 1
    K_V_XY = 0.97;
    K_V_Z = 0.87;
    K_OMEGA_PSI = 0.98;
end