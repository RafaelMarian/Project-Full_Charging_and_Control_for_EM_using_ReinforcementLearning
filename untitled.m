clc;
clear;

% Constants
f = 50;                      % Frequency in Hz
omega = 2 * pi * f;          % Angular frequency (rad/s)
t = 0:0.0001:0.1;            % Time vector
theta = omega * t;           % Electrical angle (rotor position)

% Input abc phase currents (example: sinusoidal signals)
I_a = 10 * sin(omega * t);                     % Phase A current
I_b = 10 * sin(omega * t - 2*pi/3);            % Phase B current
I_c = 10 * sin(omega * t + 2*pi/3);            % Phase C current

%% Clarke Transformation (abc -> αβ)
% Formula:
% Iα = (2/3)*Ia - (1/3)*Ib - (1/3)*Ic
% Iβ = (1/sqrt(3))*(Ib - Ic)
I_alpha = (2/3) * I_a - (1/3) * I_b - (1/3) * I_c;
I_beta = (1/sqrt(3)) * (I_b - I_c);

% Zero-sequence component (not used in balanced systems)
I_0 = (1/3) * (I_a + I_b + I_c);

%% Park Transformation (αβ -> dq)
% Formula:
% Id = Iα*cos(θ) + Iβ*sin(θ)
% Iq = -Iα*sin(θ) + Iβ*cos(θ)
I_d = I_alpha .* cos(theta) + I_beta .* sin(theta);
I_q = -I_alpha .* sin(theta) + I_beta .* cos(theta);

%% Inverse Park Transformation (dq -> αβ)
% Formula:
% Iα = Id*cos(θ) - Iq*sin(θ)
% Iβ = Id*sin(θ) + Iq*cos(θ)
I_alpha_reconstructed = I_d .* cos(theta) - I_q .* sin(theta);
I_beta_reconstructed = I_d .* sin(theta) + I_q .* cos(theta);

%% Inverse Clarke Transformation (αβ -> abc)
% Formula:
% Ia = Iα
% Ib = -0.5*Iα + sqrt(3)/2 * Iβ
% Ic = -0.5*Iα - sqrt(3)/2 * Iβ
I_a_reconstructed = I_alpha_reconstructed;
I_b_reconstructed = -0.5 * I_alpha_reconstructed + (sqrt(3)/2) * I_beta_reconstructed;
I_c_reconstructed = -0.5 * I_alpha_reconstructed - (sqrt(3)/2) * I_beta_reconstructed;

%% Plot Results

% Plot original abc currents
figure;
subplot(3,1,1);
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
subplot(3,1,2);
plot(t, I_alpha, 'r', 'DisplayName', '\alpha');
hold on;
plot(t, I_beta, 'b', 'DisplayName', '\beta');
xlabel('Time (s)');
ylabel('Current (A)');
title('Clarke Transform: \alpha-\beta Frame');
legend;
grid on;

% Plot dq currents (Park Transformation)
subplot(3,1,3);
plot(t, I_d, 'r', 'DisplayName', 'I_d');
hold on;
plot(t, I_q, 'b', 'DisplayName', 'I_q');
xlabel('Time (s)');
ylabel('Current (A)');
title('Park Transform: d-q Frame');
legend;
grid on;

% Plot Inverse Park Transformation
figure;
subplot(2,1,1);
plot(t, I_alpha_reconstructed, 'r', 'DisplayName', 'I_{\alpha}');
hold on;
plot(t, I_beta_reconstructed, 'b', 'DisplayName', 'I_{\beta}');
xlabel('Time (s)');
ylabel('Current (A)');
title('Inverse Park Transform: \alpha-\beta Frame');
legend;
grid on;

% Plot Inverse Clarke Transformation
subplot(2,1,2);
plot(t, I_a_reconstructed, 'r', 'DisplayName', 'I_a');
hold on;
plot(t, I_b_reconstructed, 'g', 'DisplayName', 'I_b');
plot(t, I_c_reconstructed, 'b', 'DisplayName', 'I_c');
xlabel('Time (s)');
ylabel('Current (A)');
title('Inverse Clarke Transform: abc Frame');
legend;
grid on;

