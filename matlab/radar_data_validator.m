%% radar_data_validator.m
% 雷达仿真数据验证脚本
% 功能：
%   1. 读取导出的 IQ 数据（.dat 二进制格式）
%   2. 读取配置参数（.json 格式）
%   3. 读取目标轨迹（.csv 格式）
%   4. 进行距离 - 多普勒处理，检测目标
%   5. 对比检测结果与设定目标参数

clear; close all; clc;

%% ========== 1. 配置 ==========
OUTPUT_DIR = 'output';
CONFIG_FILE = fullfile(OUTPUT_DIR, 'config_params.json');
SCAN_INDEX = 0;  % 要分析的扫描编号

%% ========== 2. 读取配置参数 ==========
fprintf('=== 读取配置参数 ===\n');
if ~exist(CONFIG_FILE, 'file')
    error('配置文件不存在：%s', CONFIG_FILE);
end

config = loadjson(CONFIG_FILE);
system_params = config.system_params;
target_config = config.target_config;
initial_targets = config.initial_targets;

fprintf('系统参数:\n');
fprintf('  fc = %.2f GHz\n', system_params.fc_hz / 1e9);
fprintf('  PRF = %.0f Hz\n', system_params.prf_hz);
fprintf('  PRI = %.6f us\n', system_params.pri_s * 1e6);
fprintf('  脉冲数/CPI = %d\n', system_params.pulses_per_cpi);
fprintf('  采样数/脉冲 = %d\n', system_params.samples_per_pulse);
fprintf('  波长 = %.2f m\n', system_params.wavelength_m);
fprintf('  最大不模糊距离 = %.1f km\n', system_params.max_unambiguous_range_m / 1000);
fprintf('  距离分辨率 = %.1f m\n', system_params.range_resolution_m);

fprintf('\n设定目标参数:\n');
for i = 1:length(initial_targets)
    t = initial_targets{i};
    fprintf('  目标 %d: RCS=%.1f m^2, 位置=[%.0f, %.0f, %.0f] m, 速度=[%.1f, %.1f, %.1f] m/s\n', ...
        t.id, t.rcs_mean_m2, t.position_m, t.velocity_mps);
end

%% ========== 3. 读取 IQ 数据 ==========
fprintf('\n=== 读取 IQ 数据 ===\n');
iq_file = fullfile(OUTPUT_DIR, sprintf('echo_iq_scan_%d.dat', SCAN_INDEX));
if ~exist(iq_file, 'file')
    error('IQ 数据文件不存在：%s', iq_file);
end

% 读取文件头
fid = fopen(iq_file, 'rb');
if fid < 0
    error('无法打开文件：%s', iq_file);
end

% 文件头格式：[magic(4)] [version(4)] [scan_index(4)] [cpi_count(4)]
%            [pulses_per_cpi(4)] [samples_per_pulse(4)] [reserved(8)]
magic = fread(fid, 1, 'uint32');
version = fread(fid, 1, 'uint32');
scan_idx = fread(fid, 1, 'int32');
cpi_count = fread(fid, 1, 'uint32');
pulses_per_cpi = fread(fid, 1, 'uint32');
samples_per_pulse = fread(fid, 1, 'uint32');
reserved = fread(fid, 1, 'uint64');

fprintf('文件头信息:\n');
fprintf('  Magic: 0x%08X (期望：0x4543484F)\n', magic);
fprintf('  Version: %d\n', version);
fprintf('  Scan Index: %d\n', scan_idx);
fprintf('  CPI 数量：%d\n', cpi_count);
fprintf('  脉冲数/CPI: %d\n', pulses_per_cpi);
fprintf('  采样数/脉冲：%d\n', samples_per_pulse);

if magic ~= hex2dec('4543484F')
    warning('Magic 不匹配，文件格式可能不正确');
end

% 读取 IQ 数据 (float32 复数)
num_samples_total = cpi_count * pulses_per_cpi * samples_per_pulse;
iq_data_raw = fread(fid, num_samples_total * 2, 'float');
fclose(fid);

% 重组为复数矩阵 [samples x pulses x cpi]
iq_complex = iq_data_raw(1:2:end) + 1i * iq_data_raw(2:2:end);
iq_matrix = reshape(iq_complex, [samples_per_pulse, pulses_per_cpi, cpi_count]);

fprintf('IQ 数据大小：%d x %d x %d (距离样点 x 脉冲 x CPI)\n', size(iq_matrix));

%% ========== 4. 读取目标轨迹 ==========
fprintf('\n=== 读取目标轨迹 ===\n');
traj_file = fullfile(OUTPUT_DIR, 'target_trajectory.csv');
if exist(traj_file, 'file')
    trajectory_data = readtable(traj_file);
    fprintf('目标轨迹记录数：%d\n', height(trajectory_data));
else
    warning('目标轨迹文件不存在');
    trajectory_data = [];
end

%% ========== 5. 信号处理与目标检测 ==========
fprintf('\n=== 信号处理与目标检测 ===\n');

% 选择一个 CPI 进行分析（通常选波束指向接近目标方向的）
cpi_idx = 1;  % 第一个 CPI
cpi_data = squeeze(iq_matrix(:, :, cpi_idx));
[num_range_bins, num_pulses] = size(cpi_data);

fprintf('分析 CPI #%d, 数据大小：%d x %d\n', cpi_idx, num_range_bins, num_pulses);

% 5.1 距离脉压（如果有 LFM 调制）
% 这里简化处理，假设已经是基带数据
range_compressed = cpi_data;

% 5.2 多普勒处理（FFT）
% 加窗
window = hann(num_pulses);
doppler_data = zeros(num_range_bins, num_pulses);
for r = 1:num_range_bins
    doppler_data(r, :) = fft(range_compressed(r, :) .* window);
end

% 5.3 计算多普勒频率轴
doppler_freq_axis = (-num_pulses/2:num_pulses/2-1) * (system_params.prf_hz / num_pulses);
range_axis = (0:num_range_bins-1) * system_params.range_resolution_m;

% 5.4 计算检测统计量（幅度）
doppler_magnitude = abs(doppler_data);
doppler_magnitude_shifted = fftshift(doppler_magnitude, 2);

% 5.5 自适应门限检测（简化 CFAR）
% 使用固定门限 + 噪声基底估计
noise_floor = median(doppler_magnitude_shifted(:));
threshold = noise_floor * 10;  % 10 倍噪声基底

% 检测峰值
[peaks, ~] = findpeaks(doppler_magnitude_shifted(:), ...
    'MinPeakHeight', threshold, ...
    'NumPeaks', 10);

fprintf('检测门限：%.2f (噪声基底 x10)\n', threshold);
fprintf('检测到峰值数：%d\n', length(peaks));

%% ========== 6. 可视化 ==========
figure('Name', 'Range-Doppler Map', 'Position', [100, 100, 800, 600]);

% 6.1 距离 - 多普勒图
subplot(2, 2, 1);
imagesc(doppler_freq_axis, range_axis / 1000, doppler_magnitude_shifted);
axis xy;
colorbar;
title('Range-Doppler Map');
xlabel('Doppler Frequency (Hz)');
ylabel('Range (km)');

% 6.2 距离向平均功率
subplot(2, 2, 2);
range_power = mean(abs(cpi_data).^2, 2);
plot(range_axis / 1000, 10*log10(range_power + 1e-20));
xlabel('Range (km)');
ylabel('Power (dB)');
title('Range Profile');
grid on;

% 6.3 多普勒谱（某距离单元）
subplot(2, 2, 3);
% 选择能量最强的距离单元
[~, max_range_idx] = max(range_power);
plot(doppler_freq_axis, 10*log10(abs(doppler_data(max_range_idx, :)) + 1e-20));
xlabel('Doppler Frequency (Hz)');
ylabel('Power (dB)');
title(sprintf('Doppler Spectrum (Range bin %d)', max_range_idx));
grid on;

% 6.4 原始 IQ 幅度（距离 - 脉冲图）
subplot(2, 2, 4);
imagesc(abs(cpi_data));
axis xy;
colorbar;
title('Range-Pulse Map');
xlabel('Pulse Index');
ylabel('Range Bin');

sgtitle(sprintf('Radar Data Validation - Scan %d, CPI %d', SCAN_INDEX, cpi_idx));

%% ========== 7. 目标参数验证 ==========
fprintf('\n=== 目标参数验证 ===\n');

% 对于每个设定的目标，计算期望的距离和多普勒
fprintf('\n期望目标位置（基于设定参数）:\n');
for i = 1:length(initial_targets)
    t = initial_targets{i};
    if ~t.enabled
        continue;
    end

    % 计算斜距
    range_m = norm(t.position_m);

    % 计算径向速度
    los = t.position_m / range_m;
    radial_vel = dot(los, t.velocity_mps);

    % 计算期望的多普勒频率
    doppler_freq = -2 * radial_vel / system_params.wavelength_m;

    % 计算期望的距离单元
    range_bin = round(range_m / system_params.range_resolution_m);

    % 计算期望的多普勒单元
    doppler_resolution = system_params.prf_hz / num_pulses;
    doppler_bin = round(doppler_freq / doppler_resolution);
    if doppler_bin < 0
        doppler_bin = doppler_bin + num_pulses;  % 折叠到正索引
    end

    fprintf('  目标 %d: 距离=%.1f km (单元%d), 径向速度=%.1f m/s, 多普勒=%.1f Hz (单元%d)\n', ...
        t.id, range_m/1000, range_bin, radial_vel, doppler_freq, doppler_bin);
end

% 在 RD 图上标记期望位置
hold on;
for i = 1:length(initial_targets)
    t = initial_targets{i};
    if ~t.enabled
        continue;
    end

    range_m = norm(t.position_m);
    los = t.position_m / range_m;
    radial_vel = dot(los, t.velocity_mps);
    doppler_freq = -2 * radial_vel / system_params.wavelength_m;

    range_bin = round(range_m / system_params.range_resolution_m);
    doppler_bin = round(doppler_freq / doppler_resolution);

    % 在图上标记
    plot(doppler_freq, range_m/1000, 'r+', 'MarkerSize', 15, 'LineWidth', 2);
end
hold off;

fprintf('\n=== 验证完成 ===\n');
fprintf('请在图中检查红色+标记是否与能量峰值位置吻合\n');
