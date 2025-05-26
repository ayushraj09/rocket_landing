function control = pidController(t, x, params)

    global int_error_x int_error_y int_error_theta last_t
    global alpha_history thrust_history

    if isempty(int_error_x) || isempty(last_t)
        int_error_x = 0;
        int_error_y = 0;
        int_error_theta = 0;
        last_t = t;
    end

    % Time step
    dt = t - last_t;
    if dt <= 0
        dt = 1e-3; % Ensure a small dt if time difference is zero
    end

    % Extract state variables
    x_pos = x(1);
    y_pos = x(2);
    vx    = x(3);
    vy    = x(4);
    theta = x(5);
    omega = x(6);

    %% Vertical (Y) control
    e_y = y_pos - params.y_target;
    de_y = vy; 

    int_error_y = int_error_y + e_y * dt;
    int_error_y = max(min(int_error_y, 500), -500);

    a_y = -params.Kp_y * e_y - params.Kd_y * de_y - params.Ki_y * int_error_y + params.g;

    %% Horizontal (X) control
    e_x = x_pos - params.x_target;
    de_x = vx;  

    int_error_x = int_error_x + e_x * dt;
    int_error_x = max(min(int_error_x, 500), -500);

    a_x = -params.Kp_x * e_x - params.Kd_x * de_x - params.Ki_x * int_error_x;

    %% Thrust and Desired Angle
    T_total = params.m * sqrt(a_x^2 + a_y^2);
    theta_desired = atan2(a_x, a_y);

    %% Pitch (Theta) control
    e_theta = theta - theta_desired;
    de_theta = omega;  

    int_error_theta = int_error_theta + e_theta * dt;
    int_error_theta = max(min(int_error_theta, 500), -500);

    alpha = -(params.Kp_theta * e_theta + params.Kd_theta * de_theta + params.Ki_theta * int_error_theta);

    % Thrust limits
    control.T = max(min(T_total, params.Tmax), 0);
    % Gimbal (alpha) limits: ±5°
    control.alpha = max(min(alpha, deg2rad(5)), deg2rad(-5));

    last_t = t;

    thrust_history(end+1) = control.T;
    alpha_history(end+1) = control.alpha;
end
