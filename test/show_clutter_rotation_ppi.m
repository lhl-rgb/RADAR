function meta = show_clutter_rotation_ppi(prefix, dynamic_range_db, range_idx)
%SHOW_CLUTTER_ROTATION_PPI Display exported GPU sea clutter one-rotation data.
%
% Usage:
%   meta = show_clutter_rotation_ppi()
%   meta = show_clutter_rotation_ppi('out/clutter_rotation', 50)
%   meta = show_clutter_rotation_ppi('out/clutter_rotation', 50, 1000)
%
% The C++ exporter writes:
%   <prefix>_power.bin  single precision, layout [range x pulse]
%   <prefix>_meta.txt   key/value metadata

if nargin < 1 || isempty(prefix)
    prefix = fullfile('out', 'clutter_rotation');
end
if nargin < 2 || isempty(dynamic_range_db)
    dynamic_range_db = 50;
end
if nargin < 3 || isempty(range_idx)
    range_idx = 1000;
end

meta_file = [prefix '_meta.txt'];
power_file = [prefix '_power.bin'];
iq_file = [prefix '_iq.txt'];
meta = read_meta(meta_file);

range_cells = meta.range_cells;
pulses = meta.pulses;
min_range_m = meta.min_range_m;
range_bin_size_m = meta.range_bin_size_m;
az_step_deg = meta.az_step_deg;
az_start_deg = meta.az_start_deg;

fid = fopen(power_file, 'rb');
if fid < 0
    error('Failed to open %s', power_file);
end
power = fread(fid, [range_cells, pulses], 'single=>double');
fclose(fid);

if ~isequal(size(power), [range_cells, pulses])
    error('Power matrix size mismatch. Expected [%d, %d], got [%d, %d].', ...
        range_cells, pulses, size(power, 1), size(power, 2));
end

eps_power = max(power(power > 0), [], 'all') * 1e-12;
if isempty(eps_power) || ~isfinite(eps_power)
    eps_power = 1e-30;
end
power_db = 10 * log10(max(power, eps_power));
peak_db = max(power_db, [], 'all');
clim = [peak_db - dynamic_range_db, peak_db];

range_m = min_range_m + (0:range_cells-1).' * range_bin_size_m;
az_deg = az_start_deg + (0:pulses-1) * az_step_deg;
range_idx = max(1, min(range_cells, round(range_idx)));

figure('Name', 'Clutter Range-Azimuth Power');
imagesc(az_deg, range_m / 1000, power_db);
set(gca, 'YDir', 'normal');
axis tight;
colormap turbo;
colorbar;
caxis(clim);
xlabel('Azimuth (deg)');
ylabel('Slant range (km)');
title('GPU Sea Clutter One Rotation: Range-Azimuth Power');

theta = deg2rad(az_deg);
[theta_grid, range_grid] = meshgrid(theta, range_m);
x_km = range_grid .* sin(theta_grid) / 1000;
y_km = range_grid .* cos(theta_grid) / 1000;

figure('Name', 'Clutter PPI Power');
pcolor(x_km, y_km, power_db);
shading flat;
axis equal tight;
colormap turbo;
colorbar;
caxis(clim);
xlabel('East (km)');
ylabel('North (km)');
title('GPU Sea Clutter One Rotation: PPI');

figure('Name', 'Clutter Range Mean');
range_mean_db = 10 * log10(mean(max(power, eps_power), 2));
plot(range_m / 1000, range_mean_db, 'k');
grid on;
xlabel('Slant range (km)');
ylabel('Mean power (dB)');
title('Range Mean Power');

if exist(iq_file, 'file') == 2
    iq_trace = readmatrix(iq_file, 'FileType', 'text', 'CommentStyle', '#');
    iq_pulse = iq_trace(:, 1);
    iq_az_deg = iq_trace(:, 2);
    iq_complex = iq_trace(:, 3).' + 1i * iq_trace(:, 4).';
    iq_amp = iq_trace(:, 5).';
    iq_power = iq_trace(:, 6).';
    if isfield(meta, 'iq_range_idx')
        range_idx = max(1, min(range_cells, round(meta.iq_range_idx)));
    end
    cell_power = iq_power;
    cell_amp = iq_amp;
    cell_complex = iq_complex;
    cell_az_deg = iq_az_deg(:).';
else
    cell_power = power(range_idx, :);
    cell_amp = sqrt(max(cell_power, 0));
    cell_complex = [];
    cell_az_deg = az_deg;
end
cell_power_mean = mean(cell_power);
cell_power_var = var(cell_power, 1);
distribution = get_meta(meta, 'distribution', 0);
weibull_shape = get_meta(meta, 'weibull_shape', 2);
weibull_scale = get_meta(meta, 'weibull_scale', sqrt(2));
lognormal_mu = get_meta(meta, 'lognormal_mu', -1);
lognormal_sigma = get_meta(meta, 'lognormal_sigma', 1);
k_shape_nu = get_meta(meta, 'k_shape_nu', 1);
dist_fit = distribution_fit(cell_amp, distribution, weibull_shape, weibull_scale, ...
    lognormal_mu, lognormal_sigma, k_shape_nu);
cell_expected_amp_mean = dist_fit.expected_amp_mean;
cell_expected_power_var = dist_fit.expected_power_var;

valid_amp = cell_amp(isfinite(cell_amp) & cell_amp >= 0);
if isempty(valid_amp)
    valid_amp = 0;
end
amp_plot_max = prctile(valid_amp, 99.7);
if ~isfinite(amp_plot_max) || amp_plot_max <= 0
    amp_plot_max = max(valid_amp);
end
if ~isfinite(amp_plot_max) || amp_plot_max <= 0
    amp_plot_max = eps;
end
r_grid = linspace(0, amp_plot_max, 600);

figure('Name', 'Selected Range Cell Amplitude Distribution');
tiledlayout(1, 2);
nexttile;
histogram(valid_amp(valid_amp <= amp_plot_max), 100, ...
    'Normalization', 'pdf', 'DisplayStyle', 'bar');
hold on;
plot(r_grid, dist_fit.pdf(r_grid), 'r', 'LineWidth', 2);
grid on;
xlabel('Amplitude');
ylabel('PDF');
legend('Measured main range', [dist_fit.name ' fit']);
title(sprintf('PDF, Range Cell %d, R = %.3f km', ...
    range_idx, range_m(range_idx) / 1000));
xlim([0, amp_plot_max]);

nexttile;
sorted_amp = sort(valid_amp(:));
empirical_cdf = (1:numel(sorted_amp)).' / max(1, numel(sorted_amp));
plot(sorted_amp, empirical_cdf, 'k');
hold on;
plot(r_grid, dist_fit.cdf(r_grid), 'r', 'LineWidth', 2);
grid on;
xlabel('Amplitude');
ylabel('CDF');
legend('Measured empirical CDF', [dist_fit.name ' fit']);
title('CDF Check');
xlim([0, amp_plot_max]);

if distribution == 0
    figure('Name', 'Selected Range Cell Power Distribution');
    histogram(cell_power, 120, 'Normalization', 'pdf', 'DisplayStyle', 'bar');
    hold on;
    p_grid = linspace(0, prctile(cell_power, 99.5), 500);
    exp_pdf = (1 / cell_power_mean) * exp(-p_grid / cell_power_mean);
    plot(p_grid, exp_pdf, 'r', 'LineWidth', 2);
    grid on;
    xlabel('Power = |z|^2');
    ylabel('PDF');
    legend('Measured over one rotation', 'Exponential fit');
    title(sprintf('Range Cell %d Power Distribution, R = %.3f km', ...
        range_idx, range_m(range_idx) / 1000));
    xlim([0, max(p_grid)]);
end

figure('Name', 'Selected Range Cell Power vs Azimuth');
plot(cell_az_deg, 10 * log10(max(cell_power, eps_power)), 'k');
grid on;
xlabel('Azimuth (deg)');
ylabel('Power (dB)');
title(sprintf('Range Cell %d Power Across One Rotation', range_idx));

if ~isempty(cell_complex)
    iq_centered = cell_complex - mean(cell_complex);
    psd_nfft = min(2048, 2^floor(log2(numel(iq_centered))));
    if psd_nfft >= 16
        [iq_psd, freq_hz] = pwelch(iq_centered, ...
            hann(psd_nfft), floor(psd_nfft / 2), psd_nfft, meta.prf_hz, 'centered');
        iq_psd_db = 10 * log10(iq_psd / max(iq_psd));
        sigma_f_hz = meta.doppler_sigma_hz;
        target_shape = exp(-0.5 * (freq_hz / sigma_f_hz).^2);
        target_shape_db = 10 * log10(max(target_shape, realmin));

        figure('Name', 'Selected Range Cell Complex IQ PSD');
        plot(freq_hz, iq_psd_db, 'b');
        hold on;
        plot(freq_hz, target_shape_db, 'r--', 'LineWidth', 1.5);
        grid on;
        xlabel('Doppler frequency (Hz)');
        ylabel('Normalized PSD (dB)');
        legend('Measured complex I/Q PSD', 'Gaussian reference');
        title(sprintf('Range Cell %d Complex I/Q Doppler Spectrum', range_idx));
        ylim([-80, 5]);

        main_mask = target_shape_db > -60;
        figure('Name', 'Selected Range Cell Complex IQ PSD Main Region');
        plot(freq_hz(main_mask), iq_psd_db(main_mask), 'b');
        hold on;
        plot(freq_hz(main_mask), target_shape_db(main_mask), 'r--', 'LineWidth', 1.5);
        grid on;
        xlabel('Doppler frequency (Hz)');
        ylabel('Normalized PSD (dB)');
        legend('Measured complex I/Q PSD', 'Gaussian reference');
        title(sprintf('Range Cell %d Complex I/Q Doppler Main Region', range_idx));
        ylim([-70, 5]);
    end
else
    power_fluctuation = cell_power - mean(cell_power);
    psd_nfft = min(2048, 2^floor(log2(numel(power_fluctuation))));
    if psd_nfft >= 16
        [power_psd, freq_hz] = pwelch(power_fluctuation, ...
            hann(psd_nfft), floor(psd_nfft / 2), psd_nfft, meta.prf_hz, 'centered');
        power_psd_db = 10 * log10(power_psd / max(power_psd));
        sigma_f_hz = meta.doppler_sigma_hz;
        target_shape = exp(-0.5 * (freq_hz / sigma_f_hz).^2);
        target_shape_db = 10 * log10(max(target_shape, realmin));

        figure('Name', 'Selected Range Cell Power-Fluctuation PSD');
        plot(freq_hz, power_psd_db, 'b');
        hold on;
        plot(freq_hz, target_shape_db, 'r--', 'LineWidth', 1.5);
        grid on;
        xlabel('Frequency (Hz)');
        ylabel('Normalized PSD (dB)');
        legend('Measured power-fluctuation PSD', 'Gaussian reference');
        title(sprintf('Range Cell %d Power-Fluctuation Spectrum', range_idx));
        ylim([-80, 5]);
    end
end

if ~isempty(cell_complex)
    meta.selected_mean_real = mean(real(cell_complex));
    meta.selected_mean_imag = mean(imag(cell_complex));
    meta.selected_var_real = var(real(cell_complex), 1);
    meta.selected_var_imag = var(imag(cell_complex), 1);
else
    meta.selected_mean_real = NaN;
    meta.selected_mean_imag = NaN;
    meta.selected_var_real = NaN;
    meta.selected_var_imag = NaN;
end

meta.selected_range_idx = range_idx;
meta.selected_range_m = range_m(range_idx);
meta.selected_mean_power = cell_power_mean;
meta.selected_power_variance = cell_power_var;
meta.selected_expected_power_variance = cell_expected_power_var;
meta.selected_mean_amplitude = mean(cell_amp);
meta.selected_expected_amplitude_mean = cell_expected_amp_mean;
meta.selected_distribution_name = dist_fit.name;
meta.selected_weibull_shape = dist_fit.weibull_shape;
meta.selected_weibull_scale = dist_fit.weibull_scale;
meta.selected_weibull_shape_config = dist_fit.weibull_shape_config;
meta.selected_lognormal_mu = dist_fit.lognormal_mu;
meta.selected_lognormal_sigma = dist_fit.lognormal_sigma;
meta.selected_k_shape_nu = dist_fit.k_shape_nu;
meta.selected_k_amplitude_scale = dist_fit.k_amplitude_scale;

fprintf('Clutter PPI loaded\n');
fprintf('  range_cells:       %d\n', range_cells);
fprintf('  pulses:            %d\n', pulses);
fprintf('  az_step_deg:       %.9g\n', az_step_deg);
fprintf('  dynamic_range_db:  %.3f\n', dynamic_range_db);
fprintf('  peak_power_db:     %.6g\n', peak_db);
fprintf('  mean_power:        %.6g\n', mean(power, 'all'));
fprintf('Selected range cell distribution\n');
fprintf('  range_idx:         %d\n', range_idx);
fprintf('  range_km:          %.6g\n', range_m(range_idx) / 1000);
fprintf('  mean_power:        %.6g\n', cell_power_mean);
fprintf('  var_power:         %.6g\n', cell_power_var);
fprintf('  model_var_expect:  %.6g\n', cell_expected_power_var);
fprintf('  mean_amplitude:    %.6g\n', mean(cell_amp));
fprintf('  model_amp_expect:  %.6g\n', cell_expected_amp_mean);
fprintf('  distribution:      %s\n', dist_fit.name);
if distribution == 0
    fprintf('  rayleigh_amp_mean: %.6g\n', cell_expected_amp_mean);
elseif distribution == 1
    fprintf('  weibull_shape_cfg: %.6g\n', dist_fit.weibull_shape_config);
    fprintf('  weibull_shape_fit: %.6g\n', dist_fit.weibull_shape);
    fprintf('  weibull_scale_fit: %.6g\n', dist_fit.weibull_scale);
elseif distribution == 2
    fprintf('  lognormal_mu_fit:  %.6g\n', dist_fit.lognormal_mu);
    fprintf('  lognormal_sig_fit: %.6g\n', dist_fit.lognormal_sigma);
elseif distribution == 3
    fprintf('  k_shape_nu:        %.6g\n', dist_fit.k_shape_nu);
    fprintf('  k_amp_scale_fit:   %.6g\n', dist_fit.k_amplitude_scale);
end

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
        fit.weibull_shape_config = weibull_shape;
        [fit.weibull_shape, fit.weibull_scale] = weibull_fit_mle(amp, weibull_shape);
        fit.weibull_scale_config_eff = weibull_scale / 2^(1 / max(weibull_shape, eps));
        fit.lognormal_mu = NaN;
        fit.lognormal_sigma = NaN;
        fit.k_shape_nu = NaN;
        fit.k_amplitude_scale = NaN;
        fit.expected_amp_mean = fit.weibull_scale * gamma(1 + 1 / fit.weibull_shape);
        expected_power_mean = fit.weibull_scale^2 * gamma(1 + 2 / fit.weibull_shape);
        expected_power_second = fit.weibull_scale^4 * gamma(1 + 4 / fit.weibull_shape);
        fit.expected_power_var = expected_power_second - expected_power_mean^2;
        fit.pdf = @(r) weibull_pdf(r, fit.weibull_shape, fit.weibull_scale);
        fit.cdf = @(r) weibull_cdf(r, fit.weibull_shape, fit.weibull_scale);
    case 2
        positive_amp = amp(amp > 0);
        fit.name = 'LogNormal';
        fit.weibull_shape_config = NaN;
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
        fit.expected_amp_mean = exp(fit.lognormal_mu + 0.5 * fit.lognormal_sigma^2);
        expected_power_mean = exp(2 * fit.lognormal_mu + 2 * fit.lognormal_sigma^2);
        expected_power_second = exp(4 * fit.lognormal_mu + 8 * fit.lognormal_sigma^2);
        fit.expected_power_var = expected_power_second - expected_power_mean^2;
        fit.pdf = @(r) lognormal_pdf(r, fit.lognormal_mu, fit.lognormal_sigma);
        fit.cdf = @(r) lognormal_cdf(r, fit.lognormal_mu, fit.lognormal_sigma);
    case 3
        mean_power = mean(amp.^2);
        fit.name = 'K';
        fit.weibull_shape_config = NaN;
        fit.weibull_shape = NaN;
        fit.weibull_scale = NaN;
        fit.lognormal_mu = NaN;
        fit.lognormal_sigma = NaN;
        fit.k_shape_nu = k_shape_nu;
        fit.k_amplitude_scale = sqrt(max(mean_power, eps));
        fit.expected_amp_mean = mean(amp);
        fit.expected_power_var = var(amp.^2, 1);
        fit.pdf = @(r) k_amplitude_pdf(r, fit.k_shape_nu, fit.k_amplitude_scale);
        fit.cdf = @(r) k_amplitude_cdf_numeric(r, fit.k_shape_nu, fit.k_amplitude_scale);
    otherwise
        mean_power = mean(amp.^2);
        sigma = sqrt(mean_power / 2);
        fit.name = 'Rayleigh';
        fit.weibull_shape_config = NaN;
        fit.weibull_shape = NaN;
        fit.weibull_scale = NaN;
        fit.lognormal_mu = NaN;
        fit.lognormal_sigma = NaN;
        fit.k_shape_nu = NaN;
        fit.k_amplitude_scale = NaN;
        fit.expected_amp_mean = sigma * sqrt(pi / 2);
        fit.expected_power_var = mean_power^2;
        fit.pdf = @(r) (r ./ sigma^2) .* exp(-(r.^2) ./ (2 * sigma^2));
        fit.cdf = @(r) 1 - exp(-(r.^2) ./ (2 * sigma^2));
end
end

function y = weibull_pdf(r, shape, scale)
r = max(r, 0);
shape = max(shape, eps);
scale = max(scale, eps);
y = (shape ./ scale) .* (r ./ scale).^(shape - 1) .* exp(-(r ./ scale).^shape);
end

function y = weibull_cdf(r, shape, scale)
r = max(r, 0);
shape = max(shape, eps);
scale = max(scale, eps);
y = 1 - exp(-(r ./ scale).^shape);
end

function scale = weibull_scale_mle(r, shape)
r = max(r(:), 0);
shape = max(shape, eps);
scale = mean(r.^shape)^(1 / shape);
scale = max(scale, eps);
end

function [shape, scale] = weibull_fit_mle(r, initial_shape)
r = r(:);
r = r(isfinite(r) & r > 0);
if numel(r) < 2
    shape = max(initial_shape, eps);
    scale = weibull_scale_mle(max(r, eps), shape);
    return;
end

log_r = log(r);
shape = max(initial_shape, 0.1);
for iter = 1:50
    r_pow = exp(shape * log_r);
    sum_pow = sum(r_pow);
    if ~isfinite(sum_pow) || sum_pow <= 0
        break;
    end
    weighted_log = sum(r_pow .* log_r) / sum_pow;
    weighted_log2 = sum(r_pow .* log_r .* log_r) / sum_pow;
    variance_log = max(0, weighted_log2 - weighted_log^2);
    score = 1 / shape + mean(log_r) - weighted_log;
    derivative = -1 / (shape * shape) - variance_log;
    if ~isfinite(score) || ~isfinite(derivative) || derivative == 0
        break;
    end
    next_shape = shape - score / derivative;
    if ~isfinite(next_shape) || next_shape <= 0
        next_shape = shape * 0.5;
    end
    if abs(next_shape - shape) <= 1e-6 * max(1, shape)
        shape = next_shape;
        break;
    end
    shape = max(next_shape, 1e-3);
end

shape = max(shape, eps);
scale = weibull_scale_mle(r, shape);
end

function y = lognormal_pdf(r, mu, sigma)
r = max(r, realmin);
sigma = max(sigma, eps);
y = exp(-((log(r) - mu).^2) ./ (2 * sigma^2)) ./ (r .* sigma .* sqrt(2 * pi));
end

function y = lognormal_cdf(r, mu, sigma)
r = max(r, realmin);
sigma = max(sigma, eps);
y = 0.5 + 0.5 * erf((log(r) - mu) ./ (sigma * sqrt(2)));
end

function y = k_amplitude_pdf(r, nu, amp_scale)
r0 = max(r ./ max(amp_scale, eps), realmin);
nu = max(nu, eps);
coef = 4 * nu^((nu + 1) / 2) / gamma(nu);
y0 = coef .* (r0.^nu) .* besselk(nu - 1, 2 * r0 * sqrt(nu));
y = y0 ./ max(amp_scale, eps);
y(~isfinite(y)) = 0;
end

function y = k_amplitude_cdf_numeric(r, nu, amp_scale)
r = max(r(:), 0);
grid_max = max(r);
if grid_max <= 0
    y = zeros(size(r));
    return;
end
grid = linspace(0, grid_max, 2000).';
pdf_grid = k_amplitude_pdf(grid, nu, amp_scale);
cdf_grid = cumtrapz(grid, pdf_grid);
if cdf_grid(end) > 0
    cdf_grid = cdf_grid ./ cdf_grid(end);
end
y = interp1(grid, cdf_grid, r, 'linear', 'extrap');
y = min(max(y, 0), 1);
end

function meta = read_meta(meta_file)
fid = fopen(meta_file, 'r');
if fid < 0
    error('Failed to open %s', meta_file);
end

meta = struct();
while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end
    line = strtrim(line);
    if isempty(line) || startsWith(line, '#')
        continue;
    end
    parts = regexp(line, '\s+', 'split');
    if numel(parts) < 2
        continue;
    end
    key = matlab.lang.makeValidName(parts{1});
    value_text = parts{2};
    value_num = str2double(value_text);
    if isfinite(value_num)
        meta.(key) = value_num;
    else
        meta.(key) = value_text;
    end
end
fclose(fid);
end
