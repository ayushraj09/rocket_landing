function dx = rocketODE(~, x, control, params)

    x_pos   = x(1);
    y_pos   = x(2);
    vx      = x(3);
    vy      = x(4);
    theta   = x(5);
    omega   = x(6);
    
    T     = control.T;
    alpha = control.alpha;
    
    % Parameters
    m  = params.m;
    I  = params.I;
    r  = params.r;
    
    % Dynamics
    dx = zeros(6,1);
    dx(1) = vx;
    dx(2) = vy;
    dx(3) = (T/m) * sin(theta + alpha) + (params.wind_x/m);
    dx(4) = (T/m) * cos(theta + alpha) - params.g + (params.wind_y/m);
    dx(5) = omega;
    dx(6) = (T * r * sin(alpha)) / I;
end
