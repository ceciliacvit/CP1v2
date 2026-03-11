%% Simulate TurtleBot PID path following with lag and noise
clear
clc
close all

%% Path from PRM
path = [
    0.1000    0.1000
    0.2899    0.1623
    0.5538    1.1705
    1.5904    1.5583
    1.5000    1.5000
];

%% True robot state [x, y, theta]
x = 0.1;
y = 0.1;
theta = 0.0;

%% Actuator states (actual applied velocities)
v_actual = 0;
omega_actual = 0;

%% Controller gains
Kp_h = 1.9;
Ki_h = 0.2;
Kd_h = 0.2;

Kp_d = 0.2;
Ki_d = 0.0;
Kd_d = 0.0;

%% Controller memory
headingErrorInt = 0;
headingErrorPrev = 0;

distanceErrorInt = 0;
distanceErrorPrev = 0;

%% Simulation settings
dt = 0.05;              % seconds
Tmax = 120;             % max simulation time
N = round(Tmax/dt);

waypoint_idx = 1;
waypoint_tolerance = 0.05;

%% Actuator lag settings
tau_v = 0.30;           % linear velocity time constant [s]
tau_omega = 0.20;       % angular velocity time constant [s]

%% Measurement noise settings
sigma_x = 0.005;        % meters
sigma_y = 0.005;        % meters
sigma_theta = 0.01;     % radians

%% Logging
traj = zeros(N, 3);         % true state [x y theta]
traj_meas = zeros(N, 3);    % measured state
vel_cmd_log = zeros(N, 2);  % commanded [v omega]
vel_act_log = zeros(N, 2);  % actual [v omega]
err_log = zeros(N, 2);      % [distanceError headingError]
t_log = zeros(N, 1);
wp_log = zeros(N, 1);

%% Main simulation loop
for k = 1:N
    t = (k-1) * dt;
    t_log(k) = t;

    %% Current waypoint
    position_desired = path(waypoint_idx, :);

    %% Noisy measurements
    x_meas = x + sigma_x * randn;
    y_meas = y + sigma_y * randn;
    theta_meas = theta + sigma_theta * randn;
    theta_meas = atan2(sin(theta_meas), cos(theta_meas));

    position_meas = [x_meas, y_meas];

    %% Compute errors using measured state
    delta = position_desired - position_meas;
    desiredHeading = atan2(delta(2), delta(1));
    distanceToTarget = norm(delta);

    headingError = atan2(sin(desiredHeading - theta_meas), cos(desiredHeading - theta_meas));
    distanceError = distanceToTarget;

    %% Heading PID
    headingErrorInt = headingErrorInt + headingError * dt;
    headingErrorDer = (headingError - headingErrorPrev) / dt;

    omega_cmd = Kp_h * headingError ...
              + Ki_h * headingErrorInt ...
              + Kd_h * headingErrorDer;

    headingErrorPrev = headingError;

    %% Distance PID
    distanceErrorInt = distanceErrorInt + distanceError * dt;
    distanceErrorDer = (distanceError - distanceErrorPrev) / dt;

    v_cmd = Kp_d * distanceError ...
          + Ki_d * distanceErrorInt ...
          + Kd_d * distanceErrorDer;

    distanceErrorPrev = distanceError;

    %% Same logic as real controller
    if abs(headingError) > 0.4
        v_cmd = v_cmd * max(0, cos(headingError));
    end

    if distanceToTarget < 0.15
        v_cmd = 0.5 * v_cmd;
    end

    %% Clip commanded velocities like on TurtleBot
    v_cmd = clip(v_cmd, -0.2, 0.2);
    omega_cmd = clip(omega_cmd, -2.0, 2.0);

    %% Waypoint switching
    if distanceToTarget < waypoint_tolerance
        if waypoint_idx < size(path, 1)
            waypoint_idx = waypoint_idx + 1;
        else
            v_cmd = 0;
            omega_cmd = 0;
        end
    end

    %% Actuator lag: actual velocity follows commanded velocity gradually
    alpha_v = min(dt / tau_v, 1);
    alpha_omega = min(dt / tau_omega, 1);

    v_actual = v_actual + alpha_v * (v_cmd - v_actual);
    omega_actual = omega_actual + alpha_omega * (omega_cmd - omega_actual);

    %% Update true robot state using actual velocities
    x = x + v_actual * cos(theta) * dt;
    y = y + v_actual * sin(theta) * dt;
    theta = theta + omega_actual * dt;
    theta = atan2(sin(theta), cos(theta));

    %% Log data
    traj(k, :) = [x, y, theta];
    traj_meas(k, :) = [x_meas, y_meas, theta_meas];
    vel_cmd_log(k, :) = [v_cmd, omega_cmd];
    vel_act_log(k, :) = [v_actual, omega_actual];
    err_log(k, :) = [distanceError, headingError];
    wp_log(k) = waypoint_idx;

    %% Stop if final waypoint is reached
    if waypoint_idx == size(path, 1) && distanceToTarget < waypoint_tolerance
        traj = traj(1:k, :);
        traj_meas = traj_meas(1:k, :);
        vel_cmd_log = vel_cmd_log(1:k, :);
        vel_act_log = vel_act_log(1:k, :);
        err_log = err_log(1:k, :);
        t_log = t_log(1:k);
        wp_log = wp_log(1:k);
        break
    end
end

%% Plot true trajectory and waypoints
figure
plot(path(:,1), path(:,2), 'ro--', 'LineWidth', 1.5, 'MarkerSize', 8)
hold on
plot(traj(:,1), traj(:,2), 'b', 'LineWidth', 2)
plot(traj_meas(:,1), traj_meas(:,2), '.', 'MarkerSize', 6)
plot(path(1,1), path(1,2), 'gs', 'MarkerSize', 10, 'LineWidth', 2)
plot(path(end,1), path(end,2), 'kx', 'MarkerSize', 12, 'LineWidth', 2)
axis equal
grid on
xlabel('x [m]')
ylabel('y [m]')
legend('Waypoints', 'True trajectory', 'Measured position', 'Start', 'Goal')
title('Simulated PID Path Following with Lag and Noise')

%% Plot distance error
%figure
%plot(t_log, err_log(:,1), 'LineWidth', 1.5)
%grid on
%xlabel('Time [s]')
%ylabel('Distance error [m]')
%title('Distance Error')

%% Plot heading error
%figure
%plot(t_log, err_log(:,2), 'LineWidth', 1.5)
%grid on
%xlabel('Time [s]')
%ylabel('Heading error [rad]')
%title('Heading Error')

%% Plot commanded vs actual linear velocity
%figure
%plot(t_log, vel_cmd_log(:,1), 'LineWidth', 1.5)
%hold on
%plot(t_log, vel_act_log(:,1), '--', 'LineWidth', 1.5)
%grid on
%xlabel('Time [s]')
%ylabel('Linear velocity [m/s]')
%legend('Commanded', 'Actual')
%title('Linear Velocity')

%% Plot commanded vs actual angular velocity
%figure
%plot(t_log, vel_cmd_log(:,2), 'LineWidth', 1.5)
%hold on
%plot(t_log, vel_act_log(:,2), '--', 'LineWidth', 1.5)
%grid on
%xlabel('Time [s]')
%ylabel('Angular velocity [rad/s]')
%legend('Commanded', 'Actual')
%title('Angular Velocity')

%% Plot waypoint index over time
%figure
%stairs(t_log, wp_log, 'LineWidth', 1.5)
%grid on
%xlabel('Time [s]')
%ylabel('Waypoint index')
%title('Waypoint Progress')

%% Helper function
function y = clip(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end