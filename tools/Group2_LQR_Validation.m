%% Group 2 four-state LQR validation
%
% Simulation only. This script does not select Betaflight gains or alter
% firmware. All plant quantities marked "estimate" require identification
% before hardware use.

clear; close all; clc;

%% Estimated vehicle and actuator parameters
massEstimateKg = 0.085;                 % kg, estimate
motorDiagonalSpacingEstimateM = 0.095;  % m, motor-to-motor diagonal estimate
motorRadiusEstimateM = motorDiagonalSpacingEstimateM / 2; % m, geometric estimate

% Four equal point masses at the motor locations give this symmetric,
% deliberately approximate inertia estimate.
inertiaEstimateKgm2 = massEstimateKg * motorRadiusEstimateM^2 / 2; % kg*m^2
inertiaEstimate = [inertiaEstimateKgm2; inertiaEstimateKgm2];

motorLagEstimateS = 0.05;               % s, first-order motor torque lag estimate
torqueLimitEstimateNm = 0.003;          % N*m per roll/pitch axis, estimate
disturbanceEstimateNm = 0.0005;          % N*m roll disturbance, estimate

fprintf('Simulation estimates:\n');
fprintf('  mass: %.6f kg\n', massEstimateKg);
fprintf('  motor diagonal spacing: %.6f m\n', motorDiagonalSpacingEstimateM);
fprintf('  roll/pitch inertia: %.6g kg*m^2\n', inertiaEstimateKgm2);
fprintf('  motor torque lag: %.3f s\n', motorLagEstimateS);
fprintf('  roll/pitch torque limit: +/- %.6f N*m\n', torqueLimitEstimateNm);

%% Four-state continuous-time plant
% x4 = [roll_angle; pitch_angle; roll_rate; pitch_rate]
% Angles are radians, rates are rad/s, and torques are N*m.
A4 = [0 0 1 0;
      0 0 0 1;
      0 0 0 0;
      0 0 0 0];
B4 = [0 0;
      0 0;
      1 / inertiaEstimate(1) 0;
      0 1 / inertiaEstimate(2)];

% Simulation-only design weights. These are not Betaflight flight gains.
Q = diag([1 / deg2rad(10)^2, 1 / deg2rad(10)^2, ...
          1 / deg2rad(100)^2, 1 / deg2rad(100)^2]);
R = diag([1 / torqueLimitEstimateNm^2, 1 / torqueLimitEstimateNm^2]);

if exist('lqr', 'file') ~= 2
    error('This script requires the MATLAB Control System Toolbox (lqr).');
end

[K, ~, eig4] = lqr(A4, B4, Q, R);

%% Six-state validation plant with first-order actual-torque dynamics
% x6 = [roll_angle; pitch_angle; roll_rate; pitch_rate;
%       actual_roll_torque; actual_pitch_torque]
A6 = [A4, zeros(4, 2);
      zeros(2, 4), -eye(2) / motorLagEstimateS];
actualTorqueToAcceleration = [zeros(2, 2);
                              diag(1 ./ inertiaEstimate)];
A6(1:4, 5:6) = actualTorqueToAcceleration;
B6 = [zeros(4, 2);
      eye(2) / motorLagEstimateS];

% The controller observes the four rigid-body states and commands torque.
% This augmented closed loop is used only for the unsaturated stability check.
A6ClosedLoop = A6 - B6 * [K, zeros(2, 2)];
eig6 = eig(A6ClosedLoop);

%% Time response with command saturation and a roll disturbance
dt = 0.001;
t = (0:dt:6).';
sampleCount = numel(t);
x = zeros(sampleCount, 6);
uRaw = zeros(sampleCount, 2);
uCommanded = zeros(sampleCount, 2);
disturbance = zeros(sampleCount, 2);

x(1, :) = [deg2rad(15), deg2rad(-15), 0, 0, 0, 0];
xReference = zeros(4, 1);

for k = 1:sampleCount - 1
    state4 = x(k, 1:4).';
    uRaw(k, :) = (-K * (state4 - xReference)).';
    uCommanded(k, :) = min(max(uRaw(k, :), -torqueLimitEstimateNm), ...
                            torqueLimitEstimateNm);

    if t(k) >= 2.5 && t(k) < 2.55
        disturbance(k, 1) = disturbanceEstimateNm;
    end

    % Explicit Euler is adequate here because dt << motor lag.
    xdot = A6 * x(k, :).';
    xdot = xdot + B6 * uCommanded(k, :).';
    xdot(3:4) = xdot(3:4) + (disturbance(k, :) ./ inertiaEstimate.').';
    x(k + 1, :) = x(k, :) + (dt * xdot).';
end

uRaw(end, :) = (-K * (x(end, 1:4).' - xReference)).';
uCommanded(end, :) = min(max(uRaw(end, :), -torqueLimitEstimateNm), ...
                          torqueLimitEstimateNm);

%% Metrics
angleError = x(:, 1:2) - repmat(xReference(1:2).', sampleCount, 1);
rateError = x(:, 3:4) - repmat(xReference(3:4).', sampleCount, 1);
angleErrorDeg = rad2deg(angleError);
rateErrorDegPerS = rad2deg(rateError);

% Settling means both angle and rate remain inside these bounds thereafter.
angleBandDeg = 1.0;
rateBandDegPerS = 5.0;
settlingTimeS = nan(1, 2);
for axis = 1:2
    inBand = abs(angleErrorDeg(:, axis)) <= angleBandDeg & ...
             abs(rateErrorDegPerS(:, axis)) <= rateBandDegPerS;
    lastOutOfBand = find(~inBand, 1, 'last');
    if isempty(lastOutOfBand)
        settlingTimeS(axis) = 0;
    elseif lastOutOfBand < sampleCount
        settlingTimeS(axis) = t(lastOutOfBand + 1);
    end
end

saturated = any(abs(uRaw) > torqueLimitEstimateNm, 2);
saturationPercent = 100 * mean(saturated);
finalAngleErrorDeg = angleErrorDeg(end, :);
finalRateErrorDegPerS = rateErrorDegPerS(end, :);

fprintf('\nFour-state LQR result (simulation only):\n');
fprintf('K =\n');
disp(K);
fprintf('Four-state closed-loop eigenvalues:\n');
disp(eig4);
fprintf('Six-state actuator-lag closed-loop eigenvalues (unsaturated):\n');
disp(eig6);
fprintf('Closed-loop stable (four-state): %s\n', yesNo(all(real(eig4) < 0)));
fprintf('Closed-loop stable (six-state lag model): %s\n', yesNo(all(real(eig6) < 0)));
fprintf('Settling time, roll (1 deg / 5 deg/s): %.3f s\n', settlingTimeS(1));
fprintf('Settling time, pitch (1 deg / 5 deg/s): %.3f s\n', settlingTimeS(2));
fprintf('Command saturation: %.2f%% of samples\n', saturationPercent);
fprintf('Peak raw roll/pitch command: [%.6f %.6f] N*m\n', max(abs(uRaw), [], 1));
fprintf('Final angle error [roll pitch]: [%.6f %.6f] deg\n', finalAngleErrorDeg);
fprintf('Final rate error [roll pitch]: [%.6f %.6f] deg/s\n', finalRateErrorDegPerS);

%% Plots
figure('Name', 'Group 2 LQR validation');
subplot(3, 1, 1);
plot(t, rad2deg(x(:, 1)), 'b', t, rad2deg(x(:, 2)), 'r', 'LineWidth', 1.1);
grid on;
ylabel('Angle (deg)');
legend('Roll', 'Pitch', 'Location', 'best');
title('Four-state controller on six-state motor-lag plant');

subplot(3, 1, 2);
plot(t, uCommanded(:, 1), 'b', t, uCommanded(:, 2), 'r', ...
     t, torqueLimitEstimateNm * ones(size(t)), 'k--', ...
     t, -torqueLimitEstimateNm * ones(size(t)), 'k--', 'LineWidth', 1.1);
grid on;
ylabel('Commanded torque (N*m)');
legend('Roll', 'Pitch', 'Limits', 'Location', 'best');

subplot(3, 1, 3);
plot(t, x(:, 5), 'b', t, x(:, 6), 'r', ...
     t, disturbance(:, 1), 'k:', 'LineWidth', 1.1);
grid on;
ylabel('Actual torque (N*m)');
xlabel('Time (s)');
legend('Actual roll', 'Actual pitch', 'Roll disturbance', 'Location', 'best');

function text = yesNo(value)
if value
    text = 'yes';
else
    text = 'no';
end
end
