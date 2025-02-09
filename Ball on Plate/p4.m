% Parameters
T = 5;           % Transition time
z0_x = 0;        % Initial position in x-direction
zT_x = 1;        % Target position in x-direction
z0_y = 0;        % Initial position in y-direction
zT_y = 1;        % Target position in y-direction
t = linspace(0, T, 100); % Time vector

% Compute desired trajectories and control inputs
[z1, z2, u1, u2] = desired_trajectory_and_controls(t, T, z0_x, zT_x, z0_y, zT_y);
5
% Plot desired trajectories
figure;
subplot(2, 1, 1);
plot(t, z1, 'LineWidth', 2); hold on;
plot(t, z2, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position');
title('Desired Trajectories for x- and y-Directions');
legend('x-direction', 'y-direction');
grid on;

% Plot control inputs
subplot(2, 1, 2);
plot(t, u1, 'LineWidth', 2); hold on;
plot(t, u2, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Control Input');
title('Control Inputs for x- and y-Directions');
legend('u1 (x-direction)', 'u2 (y-direction)');
grid on;
