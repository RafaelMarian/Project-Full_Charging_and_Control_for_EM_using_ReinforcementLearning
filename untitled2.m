clc;
clear;

% Constants
f = 50;                      % Initial Frequency in Hz
t = 0:0.0001:0.1;            % Time vector
omega_base = 2 * pi * f;     % Base angular frequency (rad/s)

% Simulate dynamic speed change
speed_ramp = 10;             % Frequency ramp rate in Hz/s
f_dynamic = f + speed_ramp * t;  % Frequency ramp over time
omega_dynamic = 2 * pi * f_dynamic; % Angular frequency over time
theta_dynamic = cumtrapz(t, omega_dynamic); % Electrical angle (integral of omega)

% Input abc phase currents (example: sinusoidal signals)
I_a = 10 * sin(omega_dynamic .* t);            % Phase A current
I_b = 10 * sin(omega_dynamic .* t - 2*pi/3);   % Phase B current
I_c = 10 * sin(omega_dynamic .* t + 2*pi/3);   % Phase C current

%% Clarke Transformation (abc -> αβ)
I_alpha = (2/3) * I_a - (1/3) * I_b - (1/3) * I_c;
I_beta = (1/sqrt(3)) * (I_b - I_c);

%% Park Transformation (αβ -> dq)
I_d = I_alpha .* cos(theta_dynamic) + I_beta .* sin(theta_dynamic);
I_q = -I_alpha .* sin(theta_dynamic) + I_beta .* cos(theta_dynamic);

%% Analyze stabilization point
% Calculate the derivative of omega to find stabilization time
d_omega = gradient(omega_dynamic, t); % Change of omega over time
stabilization_index = find(abs(d_omega) < 1e-2, 1, 'first'); % Stabilization threshold (find first occurrence)
if ~isempty(stabilization_index)
    stabilization_time = t(stabilization_index); % Time when speed stabilizes
else
    stabilization_time = NaN; % Default if stabilization not found
end

%% Plot Results
figure;

% Plot original abc currents
subplot(4,1,1);
plot(t, I_a, 'r', 'DisplayName', 'I_a');
hold on;
plot(t, I_b, 'g', 'DisplayName', 'I_b');
plot(t, I_c, 'b', 'DisplayName', 'I_c');
xlabel('Time (s)');
ylabel('Current (A)');
title('Original abc Currents');
legend;
grid on;

% Plot αβ currents (Clarke Transformation)
subplot(4,1,2);
plot(t, I_alpha, 'r', 'DisplayName', '\alpha');
hold on;
plot(t, I_beta, 'b', 'DisplayName', '\beta');
xlabel('Time (s)');
ylabel('Current (A)');
title('Clarke Transform: \alpha-\beta Frame');
legend;
grid on;

% Plot dq currents (Park Transformation)
subplot(4,1,3);
plot(t, I_d, 'r', 'DisplayName', 'I_d');
hold on;
plot(t, I_q, 'b', 'DisplayName', 'I_q');
xlabel('Time (s)');
ylabel('Current (A)');
title('Park Transform: d-q Frame');
legend;
grid on;

% Highlight stabilization point if valid
if ~isnan(stabilization_time)
    xline(stabilization_time, '--k', 'LineWidth', 1.5, 'DisplayName', 'Stabilization Time');
end

% Plot omega_dynamic to visualize speed changes
subplot(4,1,4);
plot(t, omega_dynamic, 'k', 'DisplayName', '\omega (rad/s)');
xlabel('Time (s)');
ylabel('Angular Speed (rad/s)');
title('Dynamic Rotational Speed (\omega)');
legend;
grid on;

% Highlight stabilization point if valid
if ~isnan(stabilization_time)
    xline(stabilization_time, '--k', 'LineWidth', 1.5, 'DisplayName', 'Stabilization Time');
end
