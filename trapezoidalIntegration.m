%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function result = trapezoidalIntegration(data_nm1, ...
    data_dot_nm1, data_dot_n, T0)
    result = data_nm1 + (T0 / 2) * (data_dot_nm1 + data_dot_n);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
