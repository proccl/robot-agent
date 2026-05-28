function run_robot_agent(port)
    % RUN_ROBOT_AGENT 一鍵啟動 RobotAgent TCP 服務器
    %
    %   Usage:
    %       run_robot_agent;       % 默認端口 12345
    %       run_robot_agent(12346); % 指定端口
    %
    %   啟動後 Figure 窗口將彈出，TCP 服務器在後台監聽，
    %   可通過 JSON 指令遠程控制機械臂。

    if nargin < 1 || isempty(port)
        port = 12345;
    end

    % 添加 src 路徑
    scriptPath = fileparts(mfilename('fullpath'));
    srcPath = fullfile(scriptPath, '..', 'src');
    addpath(srcPath);

    % 創建並啟動
    agent = RobotAgent();
    agent.start(port);
    agent.startRenderLoop();

    fprintf('\n');
    fprintf('RobotAgent is running. Close the Figure window to stop.\n');
    fprintf('Send commands via TCP to 127.0.0.1:%d\n', agent.port);
    fprintf('Example: {"cmd":"home","duration":2}\n');
end
