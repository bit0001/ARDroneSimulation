function [x_ref, y_ref, z_ref] = getReferences(t)
    reference = 0;
    
    x_0 = 0.25;
    y_0 = 0.25;
    z_0 = 1.5;
    
    V_x = 0.200;
    V_y = 0.300;
    
    f = 1 / (2 * pi);
    
    switch reference
        case 0
            x_ref = x_0 + V_x * t;
            y_ref = y_0 + V_y * t;
            z_ref = z_0;
        case 1
            x_ref = x_0 + V_x * t;
            y_ref = y_0 + V_y * t * t;
            z_ref = z_0;
        case 2
            x_ref = x_0 + V_x * t;
            y_ref = y_0 + V_y * sin(2 * pi * f * t);
            z_ref = z_0;
    end
end