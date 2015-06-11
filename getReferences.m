function [x_ref, y_ref, z_ref] = getReferences(t)
    reference = 2;
    
    x_0 = 2.0;
    y_0 = 2.0;
    z_0 = 5.0;
    
    V_x = 1.0;
    V_y = 0.5;
    
    A_y = 0.0065;
    Am_y = 5.0;
    
    f = 1 / 50;
    
    switch reference
        case 0
            x_ref = x_0 + V_x * t;
            y_ref = y_0 + V_y * t;
            z_ref = z_0;
        case 1
            x_ref = x_0 + V_x * t;
            y_ref = y_0 + A_y * t * t;
            z_ref = z_0;
        case 2
            x_ref = x_0 + V_x * t;
            y_ref = y_0 + Am_y * sin(2 * pi * f * t);
            z_ref = z_0;
    end
end