function stats = validate_clutter_patch_trace(trace_file, psd_file)
%VALIDATE_CLUTTER_PATCH_TRACE Validate one-patch sea clutter trace export.
%
% Usage:
%   stats = validate_clutter_patch_trace()
%   stats = validate_clutter_patch_trace('out/clutter_patch_trace_trace.txt', ...
%                                       'out/clutter_patch_trace_psd.txt')
%
% Expected trace columns:
%   n time_s real imag amplitude power
%
% Expected PSD columns:
%   bin freq_hz psd_linear psd_db target_shape target_shape_db

if nargin < 1 || isempty(trace_file)
    trace_file = fullfile('out', 'clutter_patch_trace_trace.txt');
end
if nargin < 2 || isempty(psd_file)
    psd_file = fullfile('out', 'clutter_patch_trace_psd.txt');
end

trace = readmatrix(trace_file, 'FileType', 'text', 'CommentStyle', '#');
psd_data = readmatrix(psd_file, 'FileType', 'text', 'CommentStyle', '#');
meta = read_meta_from_comments(trace_file);

n = trace(:, 1);
t = trace(:, 2);
x = trace(:, 3) + 1i * trace(:, 4);
amp = trace(:, 5);
power = trace(:, 6);

freq = psd_data(:, 2);
psd_db = psd_data(:, 4);
target_db = psd_data(:, 6);

sample_count = numel(x);
mean_power = mean(power);
var_power = var(power, 1);
rayleigh_sigma = sqrt(mean_power / 2);
expected_amp_mean = rayleigh_sigma * sqrt(pi / 2);
expected_power_var = mean_power^2;
distribution = get_meta(meta, 'distribution', 0);
weibull_shape = get_meta(meta, 'weibull_shape', 2);
weibull_scale = get_meta(meta, 'weibull_scale', sqrt(2));
lognormal_mu = get_meta(meta, 'lognormal_mu', -1);
lognormal_sigma = get_meta(meta, 'lognormal_sigma', 1);
k_shape_nu = get_meta(meta, 'k_shape_nu', 1);

fit = distribution_fit(amp, distribution, weibull_shape, weibull_scale, ...
    lognormal_mu, lognormal_sigma, k_shape_nu);

stats.sample_count = sample_count;
stats.mean_real = mean(real(x));
stats.mean_imag = mean(imag(x));
stats.var_real = var(real(x), 1);
stats.var_imag = var(imag(x), 1);
stats.mean_power = mean_power;
stats.mean_amplitude = mean(amp);
stats.rayleigh_sigma = rayleigh_sigma;
stats.expected_amplitude_mean = expected_amp_mean;
stats.power_variance = var_power;
stats.expected_power_variance = expected_power_var;
stats.distribution = distribution;
stats.distribution_name = fit.name;
stats.weibull_shape = fit.weibull_shape;
stats.weibull_scale = fit.weibull_scale;
stats.lognormal_mu = fit.lognormal_mu;
stats.lognormal_sigma = fit.lognormal_sigma;
stats.k_shape_nu = fit.k_shape_nu;
stats.k_amplitude_scale = fit.k_amplitude_scale;

fprintf('Single patch clutter validation\n');
fprintf('  samples:              %d\n', stats.sample_count);
fprintf('  mean(real):           %.6g\n', stats.mean_real);
fprintf('  mean(imag):           %.6g\n', stats.mean_imag);
fprintf('  var(real):            %.6g\n', stats.var_real);
fprintf('  var(imag):            %.6g\n', stats.var_imag);
fprintf('  mean(power):          %.6g\n', stats.mean_power);
fprintf('  mean(amplitude):      %.6g\n', stats.mean_amplitude);
fprintf('  var(power):           %.6g\n', stats.power_variance);
fprintf('  distribution:         %s\n', stats.distribution_name);
if distribution == 0
    fprintf('  Rayleigh mean amp:    %.6g\n', stats.expected_amplitude_mean);
    fprintf('  exponential var:      %.6g\n', stats.expected_power_variance);
elseif distribution == 1
    fprintf('  Weibull shape p:      %.6g\n', fit.weibull_shape);
    fprintf('  Weibull scale fit:    %.6g\n', fit.weibull_scale);
elseif distribution == 2
    fprintf('  LogNormal mu:         %.6g\n', fit.lognormal_mu);
    fprintf('  LogNormal sigma:      %.6g\n', fit.lognormal_sigma);
elseif distribution == 3
    fprintf('  K shape nu:           %.6g\n', fit.k_shape_nu);
    fprintf('  K amplitude scale:    %.6g\n', fit.k_amplitude_scale);
end

figure('Name', 'Clutter Patch Time Series');
subplot(2, 1, 1);
plot(t, real(x), 'b');
hold on;
plot(t, imag(x), 'r');
grid on;
xlabel('Time (s)');
ylabel('I/Q');
legend('Real', 'Imag');
title('Single Patch Complex Clutter');

subplot(2, 1, 2);
plot(t, amp, 'k');
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Amplitude Time Series');

figure('Name', 'Amplitude Distribution Check');
histogram(amp, 80, 'Normalization', 'pdf', 'DisplayStyle', 'bar');
hold on;
r_grid = linspace(max(eps, min(amp)), max(amp), 500);
plot(r_grid, fit.pdf(r_grid), 'r', 'LineWidth', 2);
grid on;
xlabel('Amplitude');
ylabel('PDF');
legend('Measured', [fit.name ' fit']);
title(['Amplitude Distribution - ' fit.name]);

if distribution == 0
    figure('Name', 'Exponential Power Check');
    histogram(power, 100, 'Normalization', 'pdf', 'DisplayStyle', 'bar');
    hold on;
    p_grid = linspace(0, prctile(power, 99.5), 500);
    exp_pdf = (1 / mean_power) * exp(-p_grid / mean_power);
    plot(p_grid, exp_pdf, 'r', 'LineWidth', 2);
    grid on;
    xlabel('Power = |z|^2');
    ylabel('PDF');
    legend('Measured', 'Exponential fit');
    title('Power Distribution');
    xlim([0, max(p_grid)]);
end

figure('Name', 'Doppler PSD Check');
plot(freq, psd_db, 'b');
hold on;
plot(freq, target_db, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Normalized PSD (dB)');
legend('Measured Welch PSD', 'Target Gaussian shape');
title('Doppler Spectrum');
ylim([-80, 5]);

figure('Name', 'Doppler PSD Main Lobe');
main_mask = target_db > -60;
plot(freq(main_mask), psd_db(main_mask), 'b');
hold on;
plot(freq(main_mask), target_db(main_mask), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Normalized PSD (dB)');
legend('Measured Welch PSD', 'Target Gaussian shape');
title('Doppler Spectrum Main Region');
ylim([-70, 5]);

end

function meta = read_meta_from_comments(path)
fid = fopen(path, 'r');
if fid < 0
    error('Failed to open %s', path);
end
meta = struct();
while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end
    line = strtrim(line);
    if ~startsWith(line, '#')
        break;
    end
    line = strtrim(extractAfter(line, 1));
    parts = regexp(line, '\s+', 'split');
    if numel(parts) < 2 || strcmp(parts{1}, 'columns:')
        continue;
    end
    key = matlab.lang.makeValidName(parts{1});
    value = str2double(parts{2});
    if isfinite(value)
        meta.(key) = value;
    else
        meta.(key) = parts{2};
    end
end
fclose(fid);
end

function value = get_meta(meta, key, fallback)
if isfield(meta, key)
    value = meta.(key);
else
    value = fallback;
end
end

function fit = distribution_fit(amp, distribution, weibull_shape, weibull_scale, ...
    lognormal_mu, lognormal_sigma, k_shape_nu)
amp = amp(:);
switch distribution
    case 1
        fit.name = 'Weibull';
        fit.weibull_shape = weibull_shape;
        fit.weibull_scale = weibull_scale_mle(amp, fit.weibull_shape);
        fit.weibull_scale_config_eff = weibull_scale / 2^(1 / max(weibull_shape, eps));
        fit.lognormal_mu = NaN;
        fit.lognormal_sigma = NaN;
        fit.k_shape_nu = NaN;
        fit.k_amplitude_scale = NaN;
        fit.pdf = @(r) weibull_pdf(r, fit.weibull_shape, fit.weibull_scale);
    case 2
        positive_amp = amp(amp > 0);
        fit.name = 'LogNormal';
        fit.weibull_shape = NaN;
        fit.weibull_scale = NaN;
        fit.k_shape_nu = NaN;
        fit.k_amplitude_scale = NaN;
        if isempty(positive_amp)
            fit.lognormal_mu = lognormal_mu;
            fit.lognormal_sigma = lognormal_sigma;
        else
            fit.lognormal_mu = mean(log(positive_amp));
            fit.lognormal_sigma = std(log(positive_amp), 1);
        end
        fit.pdf = @(r) lognormal_pdf(r, fit.lognormal_mu, fit.lognormal_sigma);
    case 3
        fit.name = 'K';
        fit.weibull_shape = NaN;
        fit.weibull_scale = NaN;
        fit.lognormal_mu = NaN;
        fit.lognormal_sigma = NaN;
        fit.k_shape_nu = k_shape_nu;
        fit.k_amplitude_scale = sqrt(max(mean(amp.^2), eps));
        fit.pdf = @(r) k_amplitude_pdf(r, fit.k_shape_nu, fit.k_amplitude_scale);
    otherwise
        mean_power = mean(amp.^2);
        sigma = sqrt(mean_power / 2);
        fit.name = 'Rayleigh';
        fit.weibull_shape = NaN;
        fit.weibull_scale = NaN;
        fit.lognormal_mu = NaN;
        fit.lognormal_sigma = NaN;
        fit.k_shape_nu = NaN;
        fit.k_amplitude_scale = NaN;
        fit.pdf = @(r) (r ./ sigma^2) .* exp(-(r.^2) ./ (2 * sigma^2));
end
end

function y = weibull_pdf(r, shape, scale)
r = max(r, 0);
shape = max(shape, eps);
scale = max(scale, eps);
y = (shape ./ scale) .* (r ./ scale).^(shape - 1) .* exp(-(r ./ scale).^shape);
end

function scale = weibull_scale_mle(r, shape)
r = max(r(:), 0);
shape = max(shape, eps);
scale = mean(r.^shape)^(1 / shape);
scale = max(scale, eps);
end

function y = lognormal_pdf(r, mu, sigma)
r = max(r, realmin);
sigma = max(sigma, eps);
y = exp(-((log(r) - mu).^2) ./ (2 * sigma^2)) ./ (r .* sigma .* sqrt(2 * pi));
end

function y = k_amplitude_pdf(r, nu, amp_scale)
r0 = max(r ./ max(amp_scale, eps), realmin);
nu = max(nu, eps);
coef = 4 * nu^((nu + 1) / 2) / gamma(nu);
y0 = coef .* (r0.^nu) .* besselk(nu - 1, 2 * r0 * sqrt(nu));
y = y0 ./ max(amp_scale, eps);
y(~isfinite(y)) = 0;
end
