%% robotagent.m — 機械臂可視化與指令監聽啟動腳本
%   使用方式：在 MATLAB Editor 中直接點擊「運行」按鈕，或雙擊此文件
%   效果：自動添加路徑 → 初始化 Figure → 啟動文件監聽 timer

% 獲取本腳本所在目錄，自動添加 src 路徑
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'src'));

% 初始化機械臂與 Figure
arm = Arm7R();
current_q = zeros(1, 7);
fig = initRobotFigure(arm, current_q);
% initRobotFigure 已存儲圖形句柄（ax, h_link 等），只需補充狀態字段
fig.UserData.is_busy = false;

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
fprintf('========================================\n');
