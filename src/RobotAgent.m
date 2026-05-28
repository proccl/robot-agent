classdef RobotAgent < handle
    % RobotAgent 機械臂智能代理服務端
    %   基於 Arm7R 的 TCP 遠程控制服務端，支持 JSON 指令執行與實時動畫更新
    %
    %   啟動方式:
    %       agent = RobotAgent();
    %       agent.start(12345);
    
    properties
        arm             % Arm7R 對象
        fig             % Figure 句柄
        ax              % Axes 句柄
        h_link          % 連桿 plot3 句柄
        h_joints        % 關節 scatter3 句柄
        h_labels        % 關節標註 text 句柄
        h_axes_base     % Base 坐標軸 line 句柄 (3x1)
        h_axes_ee       % 末端坐標軸 line 句柄 (3x1)
        
        server          % tcpserver 對象
        port            % 當前監聽端口
        is_running      % 服務器運行標誌
        is_busy         % 動畫/計算忙碌標誌
        anim_speed      % 動畫倍速
        
        current_q       % 當前關節角 (1x7)
        trajectory_queue % 軌跡隊列 (Nx7，每行為一幀)
        queue_idx       % 當前播放到的隊列索引
        
        render_timer    % 渲染循環 timer 對象
        target_fps      % 目標幀率
        render_frame_count  % 渲染幀數統計（調試/測試用）
    end
    
    methods
        %% 構造函數
        function obj = RobotAgent()
            obj.arm = Arm7R();
            obj.is_running = false;
            obj.is_busy = false;
            obj.anim_speed = 1.0;
            obj.current_q = zeros(1, 7);
            obj.trajectory_queue = [];
            obj.queue_idx = 1;
            obj.target_fps = 30;
            obj.render_frame_count = 0;
            obj.initFigure();
            fprintf('RobotAgent initialized. Call start(port) to begin server.\n');
        end
        
        %% 析構函數
        function delete(obj)
            obj.stop();
            if ~isempty(obj.render_timer) && isvalid(obj.render_timer)
                stop(obj.render_timer);
                delete(obj.render_timer);
            end
            if ~isempty(obj.fig) && isvalid(obj.fig)
                close(obj.fig);
            end
        end
        
        %% Figure 初始化
        function initFigure(obj)
            % initFigure 初始化 Figure，創建所有圖形對象並保留句柄
            
            if ~isempty(obj.fig) && isvalid(obj.fig)
                close(obj.fig);
            end
            
            obj.fig = figure('Color', 'white', ...
                             'Name', 'RobotAgent - 7R Arm', ...
                             'NumberTitle', 'off', ...
                             'Position', [100, 100, 900, 700]);
            obj.ax = axes('Parent', obj.fig);
            hold(obj.ax, 'on');
            grid(obj.ax, 'on');
            box(obj.ax, 'on');
            
            % 計算初始位置
            points = obj.arm.getJointPositions(obj.current_q);
            
            % 繪製連桿
            obj.h_link = plot3(obj.ax, points(:,1), points(:,2), points(:,3), ...
                               'b-o', 'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
            
            % 繪製關節點
            obj.h_joints = scatter3(obj.ax, points(:,1), points(:,2), points(:,3), ...
                                    100, 'b', 'filled');
            
            % 關節標註
            labels = {'Base', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'EE'};
            obj.h_labels = gobjects(length(labels), 1);
            for i = 1:length(labels)
                obj.h_labels(i) = text(obj.ax, points(i,1), points(i,2), points(i,3), ...
                                       sprintf('  %s', labels{i}), ...
                                       'FontSize', 9, 'FontWeight', 'bold');
            end
            
            % Base 坐標軸
            obj.h_axes_base = gobjects(3, 1);
            colors = {'r', 'g', 'b'};
            axis_len_base = 50;
            for j = 1:3
                dir = eye(3);
                obj.h_axes_base(j) = plot3(obj.ax, ...
                    [0, axis_len_base*dir(1,j)], ...
                    [0, axis_len_base*dir(2,j)], ...
                    [0, axis_len_base*dir(3,j)], ...
                    colors{j}, 'LineWidth', 1.5);
            end
            
            % 末端坐標軸（初始為零，在 updatePlot 中更新）
            obj.h_axes_ee = gobjects(3, 1);
            axis_len_ee = 80;
            for j = 1:3
                obj.h_axes_ee(j) = plot3(obj.ax, [0, 0], [0, 0], [0, 0], ...
                                         colors{j}, 'LineWidth', 2);
            end
            
            % 坐標軸設置
            xlabel(obj.ax, 'X (mm)', 'FontSize', 12);
            ylabel(obj.ax, 'Y (mm)', 'FontSize', 12);
            zlabel(obj.ax, 'Z (mm)', 'FontSize', 12);
            
            axis(obj.ax, [-1500, 1500, -1500, 1500, -500, 1500]);
            view(obj.ax, 45, 25);
            camproj('perspective');
            rotate3d(obj.ax, 'on');
            
            title(obj.ax, 'RobotAgent - 7R Manipulator', 'FontSize', 14);
            legend(obj.ax, 'Links', 'Location', 'best');
            
            hold(obj.ax, 'off');
            
            % 初始化時更新一次，確保 EE 坐標軸等正確定位
            obj.updatePlot(obj.current_q);
        end
        
        %% 高效更新圖形
        function updatePlot(obj, q)
            % updatePlot 高效更新圖形（不重新創建對象）
            
            if ~isvalid(obj.fig) || ~isvalid(obj.ax)
                obj.initFigure();
                return;
            end
            
            points = obj.arm.getJointPositions(q);
            
            % 更新連桿
            set(obj.h_link, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
            
            % 更新關節點
            set(obj.h_joints, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
            
            % 更新標註位置
            labels = {'Base', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'EE'};
            for i = 1:length(labels)
                if isvalid(obj.h_labels(i))
                    set(obj.h_labels(i), 'Position', points(i,:));
                end
            end
            
            % 更新末端坐標軸
            T_all = obj.getAllTransforms(q);
            T_ee = T_all{end};
            p_ee = T_ee(1:3, 4);
            axis_len_ee = 80;
            colors = {'r', 'g', 'b'};
            for j = 1:3
                if isvalid(obj.h_axes_ee(j))
                    dir = T_ee(1:3, j);
                    set(obj.h_axes_ee(j), ...
                        'XData', [p_ee(1), p_ee(1)+axis_len_ee*dir(1)], ...
                        'YData', [p_ee(2), p_ee(2)+axis_len_ee*dir(2)], ...
                        'ZData', [p_ee(3), p_ee(3)+axis_len_ee*dir(3)], ...
                        'Color', colors{j});
                end
            end
            
            drawnow limitrate;
        end
        
        %% 計算所有關節變換矩陣
        function T_all = getAllTransforms(obj, q)
            t1 = q(1); t2 = q(2); t3 = q(3); t4 = q(4);
            t5 = q(5); t6 = q(6); t7 = q(7);
            e = obj.arm.e; k = obj.arm.k; i = obj.arm.i; l = obj.arm.l;
            m = obj.arm.m; n = obj.arm.n; j = obj.arm.j; b = obj.arm.b;
            
            T01 = [-sin(t1), 0,  cos(t1), 0;
                    cos(t1), 0,  sin(t1), 0;
                    0,       1,  0,       e;
                    0,       0,  0,       1];
            T12 = [-sin(t2), 0,  cos(t2), 0;
                    cos(t2), 0,  sin(t2), 0;
                    0,       1,  0,       k;
                    0,       0,  0,       1];
            T23 = [cos(t3), -sin(t3), 0,  l*cos(t3);
                    sin(t3),  cos(t3), 0,  l*sin(t3);
                    0,        0,       1,  i;
                    0,        0,       0,  1];
            T34 = [cos(t4), -sin(t4), 0,  n*cos(t4);
                    sin(t4),  cos(t4), 0,  n*sin(t4);
                    0,        0,       1, -m;
                    0,        0,       0,  1];
            T45 = [cos(t5), 0, -sin(t5), 0;
                    sin(t5), 0,  cos(t5), 0;
                    0,      -1,  0,       0;
                    0,       0,  0,       1];
            T56 = [sin(t6), 0,  cos(t6), 0;
                   -cos(t6), 0,  sin(t6), 0;
                    0,      -1,  0,       j;
                    0,       0,  0,       1];
            T67 = [cos(t7),  sin(t7), 0,  0;
                    sin(t7), -cos(t7), 0,  0;
                    0,        0,      -1, -b;
                    0,        0,       0,  1];
            
            T02 = T01 * T12;
            T03 = T02 * T23;
            T04 = T03 * T34;
            T05 = T04 * T45;
            T06 = T05 * T56;
            T07 = T06 * T67;
            
            T_all = {eye(4), T01, T02, T03, T04, T05, T06, T07};
        end
        
        %% ===== Phase 2: TCP 服務器 =====
        
        function start(obj, port)
            % start 啟動 TCP 服務器
            if nargin < 2 || isempty(port)
                port = 12345;
            end
            
            max_attempts = 10;
            for attempt = 1:max_attempts
                try
                    obj.server = tcpserver('127.0.0.1', port);
                    break;
                catch ME
                    % 兼容中英文錯誤信息
                    is_port_in_use = strcmp(ME.identifier, 'instrument:tcpserver:connectError') || ...
                                     contains(ME.message, 'already in use', 'IgnoreCase', true) || ...
                                     contains(ME.message, 'socket', 'IgnoreCase', true) || ...
                                     contains(ME.message, '端口', 'IgnoreCase', true) || ...
                                     contains(ME.message, 'address', 'IgnoreCase', true);
                    if is_port_in_use
                        fprintf('Port %d in use, trying %d...\n', port, port+1);
                        port = port + 1;
                    else
                        rethrow(ME);
                    end
                end
            end
            
            obj.port = port;
            configureCallback(obj.server, 'terminator', @(src, evt) obj.dataCallback(src, evt));
            configureTerminator(obj.server, 'LF');
            obj.is_running = true;
            
            fprintf('========================================\n');
            fprintf('RobotAgent server started on port %d\n', port);
            fprintf('========================================\n');
        end
        
        function dataCallback(obj, src, ~)
            try
                data = readline(src);
                data = strtrim(char(data));
                if isempty(data)
                    return;
                end
                
                fprintf('[RX] %s\n', data);
                cmd = jsondecode(data);
                response = obj.executeCommand(cmd);
                
                if isvalid(src)
                    writeline(src, jsonencode(response));
                end
            catch ME
                fprintf('Error: %s\n', ME.message);
                if isvalid(src)
                    err_resp = struct('status', 'error', 'message', ME.message);
                    writeline(src, jsonencode(err_resp));
                end
            end
        end
        
        function response = executeCommand(obj, cmd)
            response = struct('status', 'ok', 'message', '');
            
            if ~isstruct(cmd) || ~isfield(cmd, 'cmd')
                response.status = 'error';
                response.message = 'Invalid command: missing "cmd" field';
                return;
            end
            
            try
                if strcmp(cmd.cmd, 'home')
                    if isfield(cmd, 'duration'), duration = cmd.duration; else, duration = 2; end
                    obj.computeTrajectoryAsync(struct('cmd', 'home', 'duration', duration));
                    response.message = 'Moving to home';
                elseif strcmp(cmd.cmd, 'move_to')
                    if ~isfield(cmd, 'position')
                        error('move_to requires "position" [x,y,z]');
                    end
                    if isfield(cmd, 'duration'), duration = cmd.duration; else, duration = 3; end
                    obj.computeTrajectoryAsync(struct('cmd', 'move_to', ...
                        'position', cmd.position, 'duration', duration));
                    response.message = 'Moving to target position';
                elseif strcmp(cmd.cmd, 'joint_move')
                    if ~isfield(cmd, 'joint') || ~isfield(cmd, 'angle')
                        error('joint_move requires "joint" and "angle"');
                    end
                    if isfield(cmd, 'duration'), duration = cmd.duration; else, duration = 2; end
                    obj.computeTrajectoryAsync(struct('cmd', 'joint_move', ...
                        'joint', cmd.joint, 'angle', cmd.angle, ...
                        'angle_deg', isfield(cmd, 'angle_deg') && cmd.angle_deg, ...
                        'duration', duration));
                    response.message = sprintf('Joint %d moving', cmd.joint);
                elseif strcmp(cmd.cmd, 'pose')
                    if isfield(cmd, 'T')
                        T_target = cmd.T;
                    elseif isfield(cmd, 'position')
                        T_current = obj.arm.forwardKinematics(obj.current_q);
                        R_current = T_current(1:3, 1:3);
                        p_target = cmd.position(:);
                        T_target = [R_current, p_target; 0, 0, 0, 1];
                    else
                        error('pose requires "T" (4x4) or "position" [x,y,z]');
                    end
                    if isfield(cmd, 'duration'), duration = cmd.duration; else, duration = 3; end
                    obj.computeTrajectoryAsync(struct('cmd', 'move_to', ...
                        'position', T_target(1:3, 4)', 'duration', duration));
                    response.message = 'Moving to pose';
                elseif strcmp(cmd.cmd, 'trajectory')
                    if ~isfield(cmd, 'type')
                        error('trajectory requires "type" field');
                    end
                    if isfield(cmd, 'duration'), duration = cmd.duration; else, duration = 5; end
                    if isfield(cmd, 'radius'), radius = cmd.radius; else, radius = 200; end
                    if isfield(cmd, 'center'), center = cmd.center; else, center = []; end
                    if isfield(cmd, 'target'), target = cmd.target; else, target = []; end
                    obj.computeTrajectoryAsync(struct('cmd', 'trajectory', ...
                        'type', cmd.type, 'duration', duration, ...
                        'radius', radius, 'center', center, 'target', target));
                    response.message = sprintf('Executing %s trajectory', cmd.type);
                elseif strcmp(cmd.cmd, 'set_speed')
                    if isfield(cmd, 'factor')
                        obj.anim_speed = max(0.1, min(5.0, cmd.factor));
                        response.message = sprintf('Speed set to %.1fx', obj.anim_speed);
                    else
                        error('set_speed requires "factor"');
                    end
                elseif strcmp(cmd.cmd, 'get_status')
                    response = obj.cmdGetStatus();
                elseif strcmp(cmd.cmd, 'plot')
                    obj.updatePlot(obj.current_q);
                    response.message = 'Plot refreshed';
                else
                    response.status = 'error';
                    response.message = sprintf('Unknown command: %s', cmd.cmd);
                end
            catch ME
                response.status = 'error';
                response.message = ME.message;
            end
        end
        
        function response = cmdGetStatus(obj)
            T_ee = obj.arm.forwardKinematics(obj.current_q);
            response = struct();
            response.status = 'ok';
            response.message = 'Status retrieved';
            response.joint_angles_rad = obj.current_q;
            response.end_effector_position = T_ee(1:3, 4)';
            response.end_effector_rotation = T_ee(1:3, 1:3);
            response.anim_speed = obj.anim_speed;
            response.is_busy = obj.is_busy;
        end
        
        %% 停止服務器
        function stop(obj)
            obj.is_running = false;
            if ~isempty(obj.server) && isvalid(obj.server)
                delete(obj.server);
            end
            fprintf('RobotAgent server stopped.\n');
        end
        
        %% ===== Phase 3: 渲染循環 =====
        
        function startRenderLoop(obj)
            % startRenderLoop 啟動 30fps 渲染定時器
            if ~isempty(obj.render_timer) && isvalid(obj.render_timer)
                stop(obj.render_timer);
                delete(obj.render_timer);
            end
            obj.render_timer = timer('ExecutionMode', 'fixedRate', ...
                                      'Period', 1/obj.target_fps, ...
                                      'TimerFcn', @(~,~) obj.renderStep());
            start(obj.render_timer);
            fprintf('Render loop started at %d fps\n', obj.target_fps);
        end
        
        function renderStep(obj)
            % renderStep 渲染單幀，播放軌跡隊列
            obj.render_frame_count = obj.render_frame_count + 1;
            
            % 播放軌跡隊列
            if ~isempty(obj.trajectory_queue) && obj.queue_idx <= size(obj.trajectory_queue, 1)
                obj.current_q = obj.trajectory_queue(obj.queue_idx, :);
                obj.queue_idx = obj.queue_idx + 1;
                obj.updatePlot(obj.current_q);
            elseif ~isempty(obj.trajectory_queue) && obj.queue_idx > size(obj.trajectory_queue, 1)
                % 隊列播放完成，釋放忙碌狀態（保留隊列供查詢）
                obj.is_busy = false;
            end
            % 注意：隊列為空時不修改 is_busy，避免與 computeTrajectoryAsync 競爭
        end
        
        function animateTo(obj, q_target, duration)
            % animateTo 生成關節空間軌跡並推入隊列（前台直接計算）
            steps = max(15, round(duration * obj.target_fps * obj.anim_speed));
            q_traj = zeros(steps, 7);
            for i = 1:7
                q_traj(:, i) = linspace(obj.current_q(i), q_target(i), steps);
            end
            obj.trajectory_queue = q_traj;
            obj.queue_idx = 1;
        end
        
        %% ===== Phase 4: 計算循環（後台） =====
        
        function computeTrajectoryAsync(obj, cmd)
            % computeTrajectoryAsync 同步計算軌跡並推入隊列
            if obj.is_busy
                fprintf('Already computing/animating, ignoring new command\n');
                return;
            end
            obj.is_busy = true;
            try
                q_traj = RobotAgent.generateTrajectory(cmd, obj.current_q, obj.arm.P_DH);
                if ~isempty(q_traj) && ~all(isnan(q_traj(:)))
                    obj.trajectory_queue = q_traj;
                    obj.queue_idx = 1;
                end
            catch ME
                fprintf('Trajectory generation failed: %s\n', ME.message);
                obj.is_busy = false;
            end
        end
    end
    
    methods (Static)
        %% ===== 靜態軌跡生成器 =====
        
        function q_traj = generateTrajectory(cmd, current_q, P_DH)
            % generateTrajectory 根據指令生成關節角軌跡（靜態方法）
            %   P_DH: Arm7R 的 DH 參數向量，後台 worker 重新創建對象
            q_traj = [];
            steps = 60;
            if isfield(cmd, 'duration')
                steps = max(20, round(cmd.duration * 30));
            end
            
            % 在後台 worker 重新創建 Arm7R 對象
            arm = Arm7R(P_DH);
            R_current = eye(3); % 默認旋轉矩陣，用於 circle
            
            switch cmd.cmd
                case 'home'
                    q_target = zeros(1, 7);
                    q_traj = zeros(steps, 7);
                    for i = 1:7
                        q_traj(:, i) = linspace(current_q(i), q_target(i), steps);
                    end
                    
                case 'joint_move'
                    q_target = current_q;
                    if isfield(cmd, 'joint') && isfield(cmd, 'angle')
                        angle = cmd.angle;
                        if isfield(cmd, 'angle_deg') && cmd.angle_deg
                            angle = deg2rad(angle);
                        end
                        q_target(cmd.joint) = angle;
                    end
                    q_traj = zeros(steps, 7);
                    for i = 1:7
                        q_traj(:, i) = linspace(current_q(i), q_target(i), steps);
                    end
                    
                case 'move_to'
                    if ~isfield(cmd, 'position')
                        error('move_to requires "position" field');
                    end
                    T_current = arm.forwardKinematics(current_q);
                    R_current = T_current(1:3, 1:3);
                    P_target = cmd.position(:);
                    T_target = [R_current, P_target; 0, 0, 0, 1];
                    [q_target, err] = arm.inverseKinematics(T_target);
                    if err ~= 0
                        error('move_to: unreachable target');
                    end
                    q_traj = zeros(steps, 7);
                    for i = 1:7
                        q_traj(:, i) = linspace(current_q(i), q_target(i), steps);
                    end
                    
                case 'trajectory'
                    if ~isfield(cmd, 'type')
                        error('trajectory requires "type" field');
                    end
                    switch cmd.type
                        case 'circle'
                            radius = 200;
                            if isfield(cmd, 'radius'), radius = cmd.radius; end
                            T_cur = arm.forwardKinematics(current_q);
                            R_current = T_cur(1:3, 1:3);
                            center = T_cur(1:3, 4);
                            if isfield(cmd, 'center') && ~isempty(cmd.center)
                                center = cmd.center(:);
                            end
                            theta = linspace(0, 2*pi, steps);
                            q_traj = zeros(steps, 7);
                            valid_count = 0;
                            for i = 1:steps
                                p_i = center + radius * [cos(theta(i)); sin(theta(i)); 0];
                                T_i = [R_current, p_i; 0, 0, 0, 1];
                                [q_i, err] = arm.inverseKinematics(T_i);
                                if err == 0
                                    valid_count = valid_count + 1;
                                    q_traj(valid_count, :) = q_i;
                                end
                            end
                            q_traj = q_traj(1:valid_count, :);
                            if valid_count == 0
                                error('circle trajectory: no valid IK');
                            end
                            
                        case 'line'
                            if ~isfield(cmd, 'target')
                                error('line trajectory requires "target"');
                            end
                            T_current = arm.forwardKinematics(current_q);
                            p_target = cmd.target(:);
                            R_current = T_current(1:3, 1:3);
                            T_target = [R_current, p_target; 0, 0, 0, 1];
                            [~, q_traj, ~] = arm.planTrajectoryCartesian(T_current, T_target, steps, 0, cmd.duration);
                            nan_rows = any(isnan(q_traj), 2);
                            q_traj = q_traj(~nan_rows, :);
                            if isempty(q_traj)
                                error('line trajectory: unreachable target');
                            end
                            % Ensure final frame reaches target precisely
                            [q_final, err] = arm.inverseKinematics(T_target);
                            if err == 0
                                q_traj(end, :) = q_final;
                            end
                            
                        otherwise
                            error('Unknown trajectory type: %s', cmd.type);
                    end
                    
                otherwise
                    error('Unknown command for trajectory generation: %s', cmd.cmd);
            end
        end
        
        %% 工具函數
        function val = getField(~, s, field, default_val)
            if isfield(s, field)
                val = s.(field);
            else
                val = default_val;
            end
        end
    end
end
