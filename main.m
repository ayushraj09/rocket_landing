clear; clc; close all;

global int_error_x int_error_y int_error_theta last_t
global alpha_history thrust_history

int_error_x = [];
int_error_y = [];
int_error_theta = [];
last_t = [];
alpha_history = [];
thrust_history = [];

%% Parameters
params.g = 9.81;
params.r = 1.0;
params.I = 500;
params.m = 1000;
params.x_target = 0;
params.y_target = 0;
params.Tmax = 60000;
params.wind_x = 0;
params.wind_y = 0;

% PID gains
params.Kp_y = 10 ;
params.Kd_y = 50 ;
params.Ki_y = 0.01;

params.Kp_x = 10;
params.Kd_x = 50;
params.Ki_x = 0.01;  

params.Kp_theta = 2;
params.Kd_theta = 1;
params.Ki_theta = 0.01;  

%% Simulation Settings
t_interval = 0.01;    
t_final = 100;      
time = 0:t_interval:t_final;
numSteps = length(time);

% Initial state: [x; y; vx; vy; theta; omega]
x = zeros(6, numSteps);
x(:,1) = [2000; 2000; -100; -100; deg2rad(0); 0];

for i = 1:numSteps-1
    t_current = time(i);
    control = pidController(t_current, x(:,i), params);
    
    tspan = [t_current, t_current + t_interval];
    [~, X_out] = ode45(@(t, x) rocketODE(t, x, control, params), tspan, x(:,i));
    
    x(:, i+1) = X_out(end,:)';
end

%% Plot results

figure;
subplot(3,2,1);
plot(time, x(1,:), 'LineWidth', 1.5);
ylabel('Horizontal Position (m)');
title('Rocket Trajectory');
grid on;

subplot(3,2,2);
plot(time, x(2,:), 'LineWidth', 1.5);
ylabel('Altitude (m)');
grid on;

subplot(3,2,3);
plot(time, x(3,:), 'LineWidth', 1.5);
ylabel('Horizontal Velocity (m/s)');
grid on;

subplot(3,2,4);
plot(time, x(4,:), 'LineWidth', 1.5);
ylabel('Vertical Velocity (m/s)');
grid on;

subplot(3,2,5);
plot(time, rad2deg(x(5,:)), 'LineWidth', 1.5);
ylabel('Pitch Angle (deg)');
xlabel('Time (s)');
grid on;

subplot(3,2,6);
plot(time(1:length(thrust_history)), thrust_history, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust vs Time');
grid on;

% Plot the rocket trajectory
figure;
plot(x(1,:), x(2,:), 'b-', 'LineWidth', 2);
hold on;
plot(params.x_target, params.y_target, 'r*', 'MarkerSize', 10);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Rocket Trajectory in X-Y Plane');
grid on;
legend('Trajectory', 'Target');

% Plot alpha
figure;
plot(time(1:length(alpha_history)), rad2deg(alpha_history), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Gimbal Angle alpha (deg)');
title('Control Gimbal Angle History');
grid on;