function cmd = parseNaturalLanguage(input_str)
% parseNaturalLanguage 將自然語言字符串解析為結構化指令
%   input_str: 用戶輸入的字符串
%   cmd:       結構化指令 struct
%
%   支持的輸入示例：
%       "home" / "回原位" / "歸零"
%       "走到 500 0 800"
%       "move to (500, 0, 800)"
%       "走到 500 0 800 用 3 秒"
%       "關節1轉90度" / "joint 1 90 deg"
%       "畫圓半徑200" / "circle radius 200"
%       "現在姿態怎樣" / "status"

    input_str = strtrim(lower(input_str));
    
    % 默認 duration
    default_duration = 5;
    
    %% home / 回原位 / 歸零
    if ismatch(input_str, {'^home$', '^回到?原位$', '^回零位$', '^歸零$'})
        cmd = struct('cmd', 'home', 'duration', default_duration);
        return;
    end
    
    %% get_status / 姿態 / 狀態
    if ismatch(input_str, {'^status$', '^現在姿態', '^狀態', '^get.status'})
        cmd = struct('cmd', 'get_status');
        return;
    end
    
    %% move_to: 走到 / move to
    move_to_patterns = {'走到\s+([-\d\.\s,]+)', 'move\s+to\s*\(?([-\d\.\s,]+)\)?'};
    for p = 1:length(move_to_patterns)
        tokens = regexp(input_str, move_to_patterns{p}, 'tokens', 'once');
        if ~isempty(tokens)
            pos = parseNumbers(tokens{1});
            if length(pos) >= 3
                duration = parseDuration(input_str);
                cmd = struct('cmd', 'move_to', 'position', pos(1:3), 'duration', duration);
                return;
            end
        end
    end
    
    %% joint_move: 關節 / joint
    joint_patterns = {
        '關節(\d+)\s*轉\s*(-?[\d\.]+)\s*度',
        'joint\s+(\d+)\s+(-?[\d\.]+)\s*(deg|degree|度)?'
    };
    for p = 1:length(joint_patterns)
        tokens = regexp(input_str, joint_patterns{p}, 'tokens', 'once');
        if ~isempty(tokens)
            joint_idx = str2double(tokens{1});
            angle_val = str2double(tokens{2});
            has_deg = true;  % 默認為角度（度）
            duration = parseDuration(input_str);
            cmd = struct('cmd', 'joint_move', 'joint', joint_idx, 'angle', angle_val, 'angle_deg', has_deg, 'duration', duration);
            return;
        end
    end
    
    %% trajectory: 畫圓 / circle
    circle_patterns = {'畫圓.*半徑\s*([\d\.]+)', 'circle.*radius\s+([\d\.]+)'};
    for p = 1:length(circle_patterns)
        tokens = regexp(input_str, circle_patterns{p}, 'tokens', 'once');
        if ~isempty(tokens)
            radius = str2double(tokens{1});
            duration = parseDuration(input_str);
            cmd = struct('cmd', 'trajectory', 'type', 'circle', 'radius', radius, 'duration', duration);
            return;
        end
    end
    
    %% trajectory: 直線 / line
    line_patterns = {'直線到\s*\(?([-\d\.\s,]+)\)?', 'line\s+to\s*\(?([-\d\.\s,]+)\)?'};
    for p = 1:length(line_patterns)
        tokens = regexp(input_str, line_patterns{p}, 'tokens', 'once');
        if ~isempty(tokens)
            target = parseNumbers(tokens{1});
            if length(target) >= 3
                duration = parseDuration(input_str);
                cmd = struct('cmd', 'trajectory', 'type', 'line', 'target', target(1:3), 'duration', duration);
                return;
            end
        end
    end
    
    %% relative_move: Z向移動 / 向上向下 / x向移動 / 在負z方向移動
    % 預處理：將「負x」「負z」等轉為「-x」「-z」
    processed = regexprep(input_str, '[負负]([xyz])', '-$1');
    rel_patterns = {
        '([xyz])[軸]?向[移動动]+\s*(-?[\d\.]+)',
        '([上下])[移動动]+\s*([\d\.]+)',
        'move\s+([xyz])\s*by\s*(-?[\d\.]+)',
        'move\s+(up|down)\s*by\s*([\d\.]+)',
        '[在朝往沿]\s*(-?[xyz])\s*[方向軸]?[移動动]+\s*(-?[\d\.]+)'
    };
    for p = 1:length(rel_patterns)
        tokens = regexp(processed, rel_patterns{p}, 'tokens', 'once');
        if ~isempty(tokens)
            axis_token = tokens{1};
            dist = str2double(tokens{2});
            if strcmp(axis_token, '上')
                axis = 'z'; dist = abs(dist);
            elseif strcmp(axis_token, '下')
                axis = 'z'; dist = -abs(dist);
            elseif strcmp(axis_token, 'up')
                axis = 'z'; dist = abs(dist);
            elseif strcmp(axis_token, 'down')
                axis = 'z'; dist = -abs(dist);
            elseif startsWith(axis_token, '-')
                axis = axis_token(2);
                dist = -abs(dist);
            else
                axis = axis_token;  % x, y, z
            end
            duration = parseDuration(input_str);
            cmd = struct('cmd', 'relative_move', 'axis', axis, 'distance', dist, 'duration', duration);
            return;
        end
    end
    
    % 無法識別
    error('parseNaturalLanguage: 無法識別輸入: "%s"', input_str);
end

%% 輔助函數

function match = ismatch(str, patterns)
    match = false;
    for i = 1:length(patterns)
        if ~isempty(regexp(str, patterns{i}, 'once'))
            match = true;
            return;
        end
    end
end

function nums = parseNumbers(str)
    % 從字符串中提取數字，支持逗號、空格分隔
    str = regexprep(str, '[,;]', ' ');
    tokens = regexp(str, '[-\d\.]+', 'match');
    nums = cellfun(@str2double, tokens);
end

function dur = parseDuration(str)
    % 從字符串中提取時間（秒）
    dur = 5;  % 默認
    tokens = regexp(str, '(\d+(?:\.\d+)?)\s*秒', 'tokens', 'once');
    if isempty(tokens)
        tokens = regexp(str, '(\d+(?:\.\d+)?)\s*s(?:ec)?', 'tokens', 'once');
    end
    if ~isempty(tokens)
        dur = str2double(tokens{1});
    end
end
