%% software_pll_demo.m
% A small learning simulation for a software phase locked loop (PLL).
%
% The PLL tracks the phase and frequency of a noisy sine wave.  The model is
% intentionally written as a plain sample-by-sample loop because this maps
% well to embedded firmware.

clear; clc; close all;

%% Simulation parameters
fs = 200000;            % Sample rate, Hz
duration = 0.08;        % Simulation time, s
t = 0:1/fs:duration-1/fs;
n = numel(t);

input_freq = 10000;     % Real input frequency, Hz
initial_phase = 1.2;    % Real input initial phase, rad
snr_db = 20;            % Additive noise level, dB

% NCO initial guess.  Make this intentionally wrong so the lock-in process is
% visible.
nco_freq_hz = 9500;     % Initial local oscillator frequency estimate, Hz
nco_phase = 0;          % Initial local oscillator phase, rad

%% Loop filter parameters
% A PLL is a feedback system.  These two gains are the easiest controls:
%   kp: reacts immediately to phase error.  Higher kp locks faster but jitters.
%   ki: corrects steady frequency error. Higher ki removes offset faster but
%       can cause overshoot or oscillation.
%
% The values below are conservative for this example.  Try changing them while
% watching the phase error and frequency estimate plots.
kp = 0.015;
ki = 0.00008;

%% Generate input signal
clean_input = sin(2*pi*input_freq*t + initial_phase);
signal_power = mean(clean_input.^2);
noise_power = signal_power / 10^(snr_db/10);
rx = clean_input + sqrt(noise_power) * randn(size(clean_input));

%% PLL state and history buffers
nco_out = zeros(1, n);
phase_error = zeros(1, n);
freq_est_hz = zeros(1, n);
phase_est = zeros(1, n);

integrator = 0;
nco_omega = 2*pi*nco_freq_hz/fs;      % rad/sample

%% Main software PLL
for k = 1:n
    % Numerically controlled oscillator (NCO).  Firmware usually implements
    % this as a phase accumulator plus a sine lookup table.
    local_sin = sin(nco_phase);
    local_cos = cos(nco_phase);
    nco_out(k) = local_sin;

    % Multiplier phase detector for a sinusoidal input.
    % Near lock:
    %   rx ~= sin(theta_in)
    %   local_cos ~= cos(theta_nco)
    %   rx * local_cos ~= 0.5 * sin(theta_in - theta_nco) + high-frequency term
    %
    % The feedback loop and low-pass behavior of the PI filter suppress the
    % high-frequency term.  The sign below makes positive phase error speed the
    % NCO up.
    err = rx(k) * local_cos;
    phase_error(k) = err;

    % PI loop filter.
    integrator = integrator + ki * err;
    correction = kp * err + integrator;

    % Update NCO frequency and phase.
    nco_omega = 2*pi*nco_freq_hz/fs + correction;
    nco_phase = nco_phase + nco_omega;
    nco_phase = wrapToPiLocal(nco_phase);

    freq_est_hz(k) = nco_omega * fs / (2*pi);
    phase_est(k) = nco_phase;
end

%% Estimate lock quality after initial transient
settle_samples = round(0.04 * fs);
locked_region = settle_samples:n;
avg_freq_est = mean(freq_est_hz(locked_region));
freq_error = avg_freq_est - input_freq;

fprintf('Input frequency:       %.2f Hz\n', input_freq);
fprintf('Average PLL estimate:  %.2f Hz\n', avg_freq_est);
fprintf('Average freq error:    %.2f Hz\n', freq_error);
fprintf('Loop gains:            kp = %.6f, ki = %.6f\n', kp, ki);

%% Plot results
% Plotting every sample of a 10 kHz sine wave over 80 ms makes a solid block.
% These views separate "waveform detail" from "PLL lock trend".

figure('Name', 'Software PLL Demo - Easy View', 'Color', 'w');

% View 1: only show the first 2 ms, so the sine waves are visible.
zoom_ms = 2;
zoom_samples = min(round(zoom_ms * 1e-3 * fs), n);
idx_zoom = 1:zoom_samples;

subplot(3, 1, 1);
plot(t(idx_zoom) * 1000, rx(idx_zoom), 'Color', [0.15 0.15 0.15]); hold on;
plot(t(idx_zoom) * 1000, nco_out(idx_zoom), 'r', 'LineWidth', 1.0);
grid on;
xlabel('Time (ms)');
ylabel('Amplitude');
title('Start-up Waveform, Zoomed to 2 ms');
legend('Noisy input', 'NCO output');

% View 2: show the lock process.  Frequency changes slowly, so plotting all
% points is useful here.
subplot(3, 1, 2);
plot(t * 1000, freq_est_hz, 'b'); hold on;
yline(input_freq, '--r', 'Input frequency');
grid on;
xlabel('Time (ms)');
ylabel('Hz');
title('PLL Frequency Estimate, Full Duration');

% View 3: show the phase detector after lock.  This is where noise/jitter is
% easier to inspect.
lock_view_start_ms = 40;
idx_lock = find(t >= lock_view_start_ms * 1e-3, 1, 'first'):n;
decim = 10;
idx_lock_plot = idx_lock(1:decim:end);

subplot(3, 1, 3);
plot(t(idx_lock_plot) * 1000, phase_error(idx_lock_plot), 'Color', [0.0 0.45 0.2]);
grid on;
xlabel('Time (ms)');
ylabel('Detector output');
title('Phase Detector Output After Lock, Decimated');

figure('Name', 'Software PLL Demo - Locked Waveform', 'Color', 'w');

% View 4: compare input and NCO after the loop has settled.
locked_zoom_ms = 2;
locked_start_ms = 50;
locked_start = find(t >= locked_start_ms * 1e-3, 1, 'first');
locked_stop = min(locked_start + round(locked_zoom_ms * 1e-3 * fs) - 1, n);
idx_locked_zoom = locked_start:locked_stop;

plot(t(idx_locked_zoom) * 1000, rx(idx_locked_zoom), 'Color', [0.15 0.15 0.15]); hold on;
plot(t(idx_locked_zoom) * 1000, nco_out(idx_locked_zoom), 'r', 'LineWidth', 1.0);
grid on;
xlabel('Time (ms)');
ylabel('Amplitude');
title('Locked Waveform, Zoomed to 2 ms');
legend('Noisy input', 'NCO output');

%% Local helper
function y = wrapToPiLocal(x)
% Keep phase bounded without requiring Mapping Toolbox.
y = mod(x + pi, 2*pi) - pi;
end
