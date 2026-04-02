function data = loadjson(filepath)
% LOADJSON 读取 JSON 文件
% 兼容旧版本 MATLAB（没有内置 jsondecode）

if exist('jsondecode', 'file')
    % MATLAB R2016b 及以后版本
    fid = fopen(filepath, 'r');
    if fid < 0
        error('无法打开文件：%s', filepath);
    end
    json_str = fread(fid, '*char')';
    fclose(fid);
    data = jsondecode(json_str);
else
    % 旧版本：简单 JSON 解析器
    data = parse_simple_json(filepath);
end
end

function data = parse_simple_json(filepath)
% 简单 JSON 解析器（支持基本结构）

fid = fopen(filepath, 'r');
if fid < 0
    error('无法打开文件：%s', filepath);
end
content = fread(fid, '*char')';
fclose(fid);

% 移除空白
content = regexprep(content, '\s+', ' ');

% 解析为结构体（简化实现）
data = struct();

% 提取 system_params
match = regexp(content, '"system_params":\s*\{([^}]+)\}', 'tokens');
if ~isempty(match)
    params_str = match{1}{1};
    data.system_params = parse_key_value_pairs(params_str);
end

% 提取 target_config
match = regexp(content, '"target_config":\s*\{([^}]+)\}', 'tokens');
if ~isempty(match)
    params_str = match{1}{1};
    data.target_config = parse_key_value_pairs(params_str);
end

% 提取 initial_targets
match = regexp(content, '"initial_targets":\s*\[([^\]]+)\]', 'tokens');
if ~isempty(match)
    targets_str = match{1}{1};
    data.initial_targets = parse_targets_array(targets_str);
end
end

function obj = parse_key_value_pairs(str)
% 解析键值对
obj = struct();

% 提取每个键值对
pattern = '"(\w+)"\s*:\s*([^,}]+)';
tokens = regexp(str, pattern, 'tokens');

for i = 1:length(tokens)
    key = tokens{i}{1};
    value_str = strtrim(tokens{i}{2});

    if strcmp(value_str, 'true')
        value = true;
    elseif strcmp(value_str, 'false')
        value = false;
    elseif isstrprop(value_str(1), 'digit') || value_str(1) == '-'
        value = str2double(value_str);
    else
        value = value_str;
    end

    obj.(key) = value;
end
end

function targets = parse_targets_array(str)
% 解析目标数组
targets = {};

% 提取每个目标对象
pattern = '\{([^}]+)\}';
matches = regexp(str, pattern, 'tokens');

for i = 1:length(matches)
    obj_str = matches{i}{1};
    obj = parse_key_value_pairs(obj_str);

    % 解析位置数组
    if isfield(obj, 'position_m')
        pos_str = obj.position_m;
        pos_str = regexprep(pos_str, '[\[\]]', '');
        pos = str2num(pos_str);
        obj.position_m = pos;
    end

    % 解析速度数组
    if isfield(obj, 'velocity_mps')
        vel_str = obj.velocity_mps;
        vel_str = regexprep(vel_str, '[\[\]]', '');
        vel = str2num(vel_str);
        obj.velocity_mps = vel;
    end

    targets{end+1} = obj;
end
end
