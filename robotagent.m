%% robotagent.m — 機械臂可視化與指令監聽啟動腳本
%   使用方式：在 MATLAB Editor 中直接點擊「運行」按鈕，或雙擊此文件
%   效果：自動添加路徑 → 初始化 Figure → 啟動文件監聽 timer

% 獲取本腳本所在目錄，自動添加 src 路徑
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'src'));

% 初始化機械臂與 Figure
arm = Arm7R();
current_q = zeros(1, 7);

% 用戶可手動設定是否啟用避障規劃
%   true  : 啟用（需要 Robotics System Toolbox）
%   false : 禁用（默認）
enable_obstacle_avoidance = true;
%enable_obstacle_avoidance = false;
% 障礙物默認配置（可視化開關與避障開關分離）
obstacle = struct('center', [400; 0; 500], ...
                  'radius', 100, ...
                  'safety_margin', 50, ...
                  'enabled', true);

fig = initRobotFigure(arm, current_q, obstacle);
% initRobotFigure 已存儲圖形句柄（ax, h_link 等），只需補充狀態字段
fig.UserData.arm = arm;
fig.UserData.current_q = current_q;
fig.UserData.is_busy = false;

% 根據用戶設定構建 rigidBodyTree 以支持避障
if enable_obstacle_avoidance
    v = ver;
    has_robotics_toolbox = any(strcmpi({v.Name}, 'Robotics System Toolbox'));
    if has_robotics_toolbox
        fig.UserData.robot_tree = buildRobotTree(arm);
        fig.UserData.obstacle_avoidance_enabled = true;
    else
        fig.UserData.robot_tree = [];
        fig.UserData.obstacle_avoidance_enabled = false;
        warning('未檢測到 Robotics System Toolbox，避障規劃功能已禁用。');
    end
else
    fig.UserData.robot_tree = [];
    fig.UserData.obstacle_avoidance_enabled = false;
end

% 創建指令監聽目錄
incoming_dir = fullfile(scriptDir, 'incoming');
if ~exist(incoming_dir, 'dir')
    mkdir(incoming_dir);
end

% 啟動定時器監聽
t = timer('ExecutionMode', 'fixedRate', 'Period', 0.5, ...
          'TimerFcn', @(~,~) processIncomingCommands(incoming_dir, fig));
start(t);

fprintf('========================================\n');
fprintf('RobotAgent started.\n');
fprintf('Figure: RobotAgent - 7R Arm\n');
fprintf('Watching: %s\n', incoming_dir);
if fig.UserData.obstacle_avoidance_enabled
    fprintf('Obstacle avoidance: enabled\n');
else
    fprintf('Obstacle avoidance: disabled (set enable_obstacle_avoidance=true to enable)\n');
end
fprintf('========================================\n');
