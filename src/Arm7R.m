classdef Arm7R < handle
    % Arm7R 7自由度機械臂運動學類
    %   封裝了7自由度機械臂的正運動學、逆運動學、軌跡規劃等功能
    %
    %   使用示例:
    %       % 創建機械臂對象(使用默認DH參數)
    %       arm = Arm7R();
    %       
    %       % 或使用自定義DH參數
    %       P_DH = [149.438, 147.9, 0, 458.09, 93.5, 360.71, 118.27, 272.42];
    %       arm = Arm7R(P_DH);
    %       
    %       % 正向運動學
    %       q = [0, 0, 0, 0, 0, 0, 0];
    %       T = arm.forwardKinematics(q);
    %       
    %       % 逆向運動學
    %       T_target = [0, 1, 0, 266.17; 1, 0, 0, -93.5; 0, 0, -1, 400.818; 0, 0, 0, 1];
    %       [q_sol, err] = arm.inverseKinematics(T_target);
    %       
    %       % 軌跡規劃
    %       q_start = [0, 0, 0, 0, 0, 0, 0];
    %       q_end = [pi/4, pi/4, 0, 0, 0, -pi/4, 0];
    %       traj = arm.planTrajectory(q_start, q_end, 100);
    %
    %   DH參數說明:
    %       P_DH = [e, k, i, l, m, n, j, b]
    %       - e: 關節1偏置 (149.438 mm)
    %       - k: 關節2偏置 (147.9 mm)
    %       - i: 關節3偏置 (0 mm)
    %       - l: 連桿3長度 (458.09 mm)
    %       - m: 關節4偏置 (93.5 mm)
    %       - n: 連桿4長度 (360.71 mm)
    %       - j: 關節6偏置 (118.27 mm)
    %       - b: 末端偏置 (272.42 mm)
    %
    %   依賴:
    %       - 核心功能: 純MATLAB，無額外依賴
    %       - 可視化(plot/teach): 需要Robotics Toolbox
    %       - 姿態插值: 需要Robotics System Toolbox (rotm2quat, quat2rotm, quatinterp)
    
    properties (Constant)
        % 默認DH參數 [e, k, i, l, m, n, j, b] (單位: mm)
        DEFAULT_DH = [149.438, 147.9, 0, 458.09, 93.5, 360.71, 118.27, 272.42];
        
        % 默認初始關節角 (用於顯示的豎直狀態)
        DEFAULT_Q0 = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
        
        % 初始偏移
        DEFAULT_Q_START = [0, 0, 0, 0, 0, 0, 0];
    end
    
    properties
        % DH參數
        P_DH          % [e, k, i, l, m, n, j, b]
        
        % 各DH參數分量
        e             % 關節1偏置
        k             % 關節2偏置
        i             % 關節3偏置
        l             % 連桿3長度
        m             % 關節4偏置
        n             % 連桿4長度
        j             % 關節6偏置
        b             % 末端偏置
        
        % 初始狀態
        q0            % 顯示用的初始姿態
        q_start       % 初始偏移
        
        % SerialLink對象 (Robotics Toolbox，可選)
        robot
        hasRoboticsToolbox  % 標誌是否安裝Robotics Toolbox
        hasRoboticsSystemToolbox  % 標誌是否安裝Robotics System Toolbox
        
        % 關節限制
        q_limit_lower % 關節下限 [-pi, -pi, ...]
        q_limit_upper % 關節上限 [pi, pi, ...]
    end
    
    methods
        %% 構造函數
        function obj = Arm7R(P_DH, q0, q_start)
            % Arm7R 構造函數
            %   obj = Arm7R() - 使用默認DH參數
            %   obj = Arm7R(P_DH) - 使用自定義DH參數
            %   obj = Arm7R(P_DH, q0) - 指定顯示初始姿態
            %   obj = Arm7R(P_DH, q0, q_start) - 指定所有參數
            
            % 檢查工具箱
            obj.hasRoboticsToolbox = obj.checkRoboticsToolbox();
            obj.hasRoboticsSystemToolbox = obj.checkRoboticsSystemToolbox();
            
            % 設置DH參數
            if nargin < 1 || isempty(P_DH)
                obj.P_DH = obj.DEFAULT_DH;
            else
                obj.P_DH = P_DH;
            end
            
            % 解包DH參數
            obj.e = obj.P_DH(1);
            obj.k = obj.P_DH(2);
            obj.i = obj.P_DH(3);
            obj.l = obj.P_DH(4);
            obj.m = obj.P_DH(5);
            obj.n = obj.P_DH(6);
            obj.j = obj.P_DH(7);
            obj.b = obj.P_DH(8);
            
            % 設置初始姿態
            if nargin < 2 || isempty(q0)
                obj.q0 = obj.DEFAULT_Q0;
            else
                obj.q0 = q0;
            end
            
            if nargin < 3 || isempty(q_start)
                obj.q_start = obj.DEFAULT_Q_START;
            else
                obj.q_start = q_start;
            end
            
            % 默認關節限制
            obj.q_limit_lower = -pi * ones(1, 7);
            obj.q_limit_upper = pi * ones(1, 7);
            
            % 創建SerialLink對象 (如果工具箱可用)
            if obj.hasRoboticsToolbox
                obj.createRobotModel();
            end
        end
        
        %% 檢查Robotics Toolbox
        function hasToolbox = checkRoboticsToolbox(obj)
            try
                Link([0, 0, 0, 0, 0]);
                hasToolbox = true;
            catch
                hasToolbox = false;
            end
        end
        
        %% 檢查Robotics System Toolbox
        function hasToolbox = checkRoboticsSystemToolbox(obj)
            try
                rotm2quat(eye(3));
                hasToolbox = true;
            catch
                hasToolbox = false;
            end
        end
        
        %% 創建機械臂模型
        function createRobotModel(obj)
            % 使用Robotics Toolbox創建機械臂模型
            % DH參數: [theta, d, a, alpha, sigma(0=旋轉,1=平移)]
            
            L1 = Link([pi/2,   obj.e,  0,      pi/2,   0], 'standard');
            L2 = Link([pi/2,   obj.k,  0,      pi/2,   0], 'standard');
            L3 = Link([0,      obj.i,  obj.l,  0,      0], 'standard');
            L4 = Link([0,     -obj.m,  obj.n,  0,      0], 'standard');
            L5 = Link([0,      0,      0,     -pi/2,   0], 'standard');
            L6 = Link([-pi/2,  obj.j,  0,     -pi/2,   0], 'standard');
            L7 = Link([0,     -obj.b,  0,      pi,     0], 'standard');
            
            obj.robot = SerialLink([L1, L2, L3, L4, L5, L6, L7], 'name', '7RArm');
        end
        
        %% 正向運動學
        function T07 = forwardKinematics(obj, q)
            % forwardKinematics 正向運動學
            %   T = arm.forwardKinematics(q) 計算關節角q對應的末端位姿
            %   
            %   輸入:
            %       q - 1x7 關節角向量 (rad)
            %   輸出:
            %       T - 4x4 齊次變換矩陣
            
            if nargin < 2
                q = zeros(1, 7);
            end
            
            t1 = q(1); t2 = q(2); t3 = q(3); t4 = q(4);
            t5 = q(5); t6 = q(6); t7 = q(7);
            
            % 計算各關節變換矩陣
            T01 = [-sin(t1), 0,  cos(t1), 0;
                    cos(t1), 0,  sin(t1), 0;
                    0,       1,  0,       obj.e;
                    0,       0,  0,       1];
                    
            T12 = [-sin(t2), 0,  cos(t2), 0;
                    cos(t2), 0,  sin(t2), 0;
                    0,       1,  0,       obj.k;
                    0,       0,  0,       1];
                    
            T23 = [cos(t3), -sin(t3), 0,  obj.l*cos(t3);
                    sin(t3),  cos(t3), 0,  obj.l*sin(t3);
                    0,        0,       1,  obj.i;
                    0,        0,       0,  1];
                    
            T34 = [cos(t4), -sin(t4), 0,  obj.n*cos(t4);
                    sin(t4),  cos(t4), 0,  obj.n*sin(t4);
                    0,        0,       1, -obj.m;
                    0,        0,       0,  1];
                    
            T45 = [cos(t5), 0, -sin(t5), 0;
                    sin(t5), 0,  cos(t5), 0;
                    0,      -1,  0,       0;
                    0,       0,  0,       1];
                    
            T56 = [sin(t6), 0,  cos(t6), 0;
                   -cos(t6), 0,  sin(t6), 0;
                    0,      -1,  0,       obj.j;
                    0,       0,  0,       1];
                    
            T67 = [cos(t7),  sin(t7), 0,  0;
                    sin(t7), -cos(t7), 0,  0;
                    0,        0,      -1, -obj.b;
                    0,        0,       0,  1];
            
            T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67;
        end
        
        %% 逆向運動學
        function [t, err] = inverseKinematics(obj, T)
            % inverseKinematics 逆向運動學
            %   [q, err] = arm.inverseKinematics(T) 計算位姿T對應的關節角
            %   
            %   輸入:
            %       T - 4x4 齊次變換矩陣
            %   輸出:
            %       q  - 1x7 關節角向量 (rad)，若無解則返回0
            %       err - 錯誤標誌，0=成功，1=無解
            
            R11 = T(1,1); R12 = T(1,2); R13 = T(1,3); Px = T(1,4);
            R21 = T(2,1); R22 = T(2,2); R23 = T(2,3); Py = T(2,4);
            R31 = T(3,1); R32 = T(3,2); R33 = T(3,3); Pz = T(3,4);
            
            t1 = 0;
            
            % 計算t2
            phi1c = -R33 * obj.b + Pz - obj.e;
            phi1s = -Px * sin(t1) + Py * cos(t1) - (-sin(t1) * R13 + R23 * cos(t1)) * obj.b;
            rou1 = sqrt(phi1c^2 + phi1s^2);
            phi1 = atan2(phi1s, phi1c);
            
            if (1 - ((-obj.m + obj.i) / rou1)^2) < 0
                err = 1;
                t = 0;
                return;
            end
            
            t2 = atan2((-obj.m + obj.i) / rou1, sqrt(max(0, 1 - ((-obj.m + obj.i) / rou1)^2))) - phi1;
        %    t2 = obj.angleRound(t2, 2);
            
            % 計算t6
            t6 = atan2(-sin(t1)*cos(t2)*R13 + cos(t1)*cos(t2)*R23 + sin(t2)*R33, ...
                       sqrt(max(0, 1 - (-sin(t1)*cos(t2)*R13 + cos(t1)*cos(t2)*R23 + sin(t2)*R33)^2)));
            %t6 = obj.angleRound(t6, 6);
            
            % 計算t7
            t7 = atan2(-sin(t1)*cos(t2)*R12 + cos(t1)*cos(t2)*R22 + sin(t2)*R32, ...
                       -sin(t1)*cos(t2)*R11 + cos(t1)*cos(t2)*R21 + sin(t2)*R31);
           % t7 = obj.angleRound(t7, 7);
            
            % 計算t3, t4, t5
            t345 = atan2(-(cos(t1)*R13 + sin(t1)*R23), ...
                         -(sin(t1)*sin(t2)*R13 - cos(t1)*sin(t2)*R23 + cos(t2)*R33));
            
            A1 = (Pz - obj.e)*cos(t2) + sin(t2)*(sin(t1)*Px - cos(t1)*Py) + cos(t6)*obj.b*cos(t345) + obj.j*sin(t345);
            B1 = sin(t1)*Py + cos(t1)*Px - obj.k + cos(t6)*obj.b*sin(t345) - obj.j*cos(t345);
            
            phi2s = A1;
            phi2c = B1;
            rou2 = sqrt(phi2c^2 + phi2s^2);
            phi2 = atan2(phi2s, phi2c);
            
            E = (obj.n^2 - obj.l^2 + A1^2 + B1^2) / (2 * obj.n);
            
            if (1 - (E/rou2)^2) < 0
                err = 1;
                t = 0;
                return;
            end
            
            t34 = -phi2 + atan2(E/rou2, -sqrt(max(0, 1 - (E/rou2)^2)));
            t5 = t345 - t34;
           % t5 = obj.angleRound(t5, 5);
            
            t3 = atan2(cos(t1)*Px + sin(t1)*Py - obj.k + cos(t6)*obj.b*sin(t345) - obj.j*cos(t345) - obj.n*sin(t34), ...
                       (Pz - obj.e)*cos(t2) + sin(t2)*(Px*sin(t1) - Py*cos(t1)) + cos(t6)*obj.b*cos(t345) + obj.j*sin(t345) - obj.n*cos(t34));
         %   t3 = obj.angleRound(t3, 3);
            
            t4 = t34 - t3;
           % t4 = obj.angleRound(t4, 4);
            
            t = [t1, t2, t3, t4, t5, t6, t7];
            err = 0;
        end
        
        %% 角度規範化
        function angle_out = angleRound(obj, angle_in, axis_idx)
            % angleRound 將角度規範化到[-pi, pi]並檢查關節限制
            %   angle = arm.angleRound(angle_in, axis_idx)
            %   
            %   輸入:
            %       angle_in - 輸入角度 (rad)
            %       axis_idx - 軸索引 (1-7)
            %   輸出:
            %       angle_out - 規範化後的角度，若超出限制則返回100*pi
            
            % 規範化到 [-pi, pi]
            mark_pi = fix(angle_in / pi);
            if mod(mark_pi, 2) == 1
                angle_out = -(mark_pi + sign(mark_pi)) * pi + angle_in;
            else
                angle_out = -mark_pi * pi + angle_in;
            end
            
            % 檢查關節限制
            if angle_out < obj.q_limit_lower(axis_idx) || angle_out > obj.q_limit_upper(axis_idx)
                angle_out = 100 * pi;  % 超出範圍標記
            end
        end
        
        %% 軌跡規劃 (位姿空間)
        function [T_traj, q_traj, simin] = planTrajectoryCartesian(obj, T_start, T_end, steps, t_start, t_end)
            % planTrajectoryCartesian 笛卡爾空間軌跡規劃
            %   [T_traj, q_traj, simin] = arm.planTrajectoryCartesian(T_start, T_end, steps, t_start, t_end)
            %   
            %   輸入:
            %       T_start - 起始位姿 (4x4矩陣)
            %       T_end   - 終止位姿 (4x4矩陣)
            %       steps   - 插值點數 (默認100)
            %       t_start - 起始時間 (s, 默認0)
            %       t_end   - 終止時間 (s, 默認1)
            %   輸出:
            %       T_traj - 4x4xsteps 軌跡位姿
            %       q_traj - stepsx7 關節角軌跡
            %       simin  - stepsx8 矩陣 [time, q1, q2, q3, q4, q5, q6, q7]，Simulink可用
            %
            %   注意: 需要Robotics System Toolbox進行姿態插值
            %         如果沒有，使用軸角插值代替
            
            if nargin < 4 || isempty(steps)
                steps = 100;
            end
            if nargin < 5 || isempty(t_start)
                t_start = 0;
            end
            if nargin < 6 || isempty(t_end)
                t_end = 1;
            end
            
            % 時間向量
            time_vec = linspace(t_start, t_end, steps)';
            
            % 位置線性插值
            P_traj = zeros(3, steps);
            P_traj(1,:) = linspace(T_start(1,4), T_end(1,4), steps);
            P_traj(2,:) = linspace(T_start(2,4), T_end(2,4), steps);
            P_traj(3,:) = linspace(T_start(3,4), T_end(3,4), steps);
            
            % 初始化輸出
            simin = zeros(steps, 8);
            simin(:,1) = time_vec;
            
            T_traj = zeros(4, 4, steps);
            q_traj = zeros(steps, 7);
            
            if obj.hasRoboticsSystemToolbox
                % 使用球面線性插值 (SLERP)
                quat_start = rotm2quat(T_start(1:3,1:3));
                quat_end = rotm2quat(T_end(1:3,1:3));
                
                for i = 1:steps
                    s = (i-1) / (steps-1);
                    quat_i = quatinterp(quat_start, quat_end, s, 'slerp');
                    R_i = quat2rotm(quat_i);
                    T_traj(:,:,i) = [R_i, P_traj(:,i); 0, 0, 0, 1];
                    
                    % 逆向運動學
                    [q_i, err] = obj.inverseKinematics(T_traj(:,:,i));
                    if err == 0
                        q_traj(i,:) = q_i;
                        simin(i,2:8) = q_i;
                    else
                        q_traj(i,:) = nan(1, 7);
                        simin(i,2:8) = nan(1, 7);
                    end
                end
            else
                % 使用簡單的軸角線性插值
                warning('Arm7R: 未檢測到Robotics System Toolbox，使用軸角線性插值代替SLERP');
                
                % 將旋轉矩陣轉換為軸角表示
                [axis_start, angle_start] = obj.rotm2axang(T_start(1:3,1:3));
                [axis_end, angle_end] = obj.rotm2axang(T_end(1:3,1:3));
                
                for i = 1:steps
                    s = (i-1) / (steps-1);
                    angle_i = angle_start + s * (angle_end - angle_start);
                    axis_i = axis_start + s * (axis_end - axis_start);
                    axis_i = axis_i / norm(axis_i);
                    R_i = obj.axang2rotm(axis_i, angle_i);
                    T_traj(:,:,i) = [R_i, P_traj(:,i); 0, 0, 0, 1];
                    
                    % 逆向運動學
                    [q_i, err] = obj.inverseKinematics(T_traj(:,:,i));
                    if err == 0
                        q_traj(i,:) = q_i;
                        simin(i,2:8) = q_i;
                    else
                        q_traj(i,:) = nan(1, 7);
                        simin(i,2:8) = nan(1, 7);
                    end
                end
            end
        end
        
        %% 旋轉矩陣轉軸角 (輔助函數)
        function [axis, angle] = rotm2axang(obj, R)
            % rotm2axang 旋轉矩陣轉軸角表示
            angle = acos((trace(R) - 1) / 2);
            if abs(angle) < 1e-6
                axis = [1, 0, 0];
            else
                axis = [R(3,2) - R(2,3), R(1,3) - R(3,1), R(2,1) - R(1,2)] / (2*sin(angle));
            end
        end
        
        %% 軸角轉旋轉矩陣 (輔助函數)
        function R = axang2rotm(obj, axis, angle)
            % axang2rotm 軸角轉旋轉矩陣 (Rodrigues公式)
            K = [0, -axis(3), axis(2);
                 axis(3), 0, -axis(1);
                 -axis(2), axis(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*K*K;
        end
        
        %% 軌跡規劃 (關節空間)
        function simin = planTrajectoryJoint(obj, q_start, q_end, t_start, t_end)
            % planTrajectoryJoint 關節空間軌跡規劃
            %   simin = arm.planTrajectoryJoint(q_start, q_end, t_start, t_end)
            %   生成Simulink可用的軌跡數據
            %   
            %   輸入:
            %       q_start - 起始關節角 (1x7)
            %       q_end   - 終止關節角 (1x7)
            %       t_start - 起始時間 (s)
            %       t_end   - 終止時間 (s)
            %   輸出:
            %       simin - Nx8 矩陣 [time, q1, q2, q3, q4, q5, q6, q7]
            
            if nargin < 5
                t_start = 0;
                t_end = 20;
            end
            
            num_points = round((t_end - t_start) * 1000);
            simin = zeros(num_points, 8);
            simin(:,1) = linspace(t_start, t_end, num_points);
            
            for i = 2:8
                simin(:,i) = linspace(q_start(i-1), q_end(i-1), num_points);
            end
        end
        
        %% 計算雅可比矩陣
        function J = jacobian(obj, q)
            % jacobian 計算雅可比矩陣
            %   J = arm.jacobian(q) 使用數值方法計算雅可比
            %   
            %   輸入:
            %       q - 1x7 關節角向量
            %   輸出:
            %       J - 6x7 雅可比矩陣 (前3行為線速度，後3行為角速度)
            %
            %   注意: 如果有Robotics Toolbox，使用其jacob0方法
            %         否則使用數值微分
            
            if obj.hasRoboticsToolbox
                q_display = obj.q0 + obj.q_start + q;
                J = obj.robot.jacob0(q_display);
            else
                % 數值微分計算雅可比
                J = zeros(6, 7);
                T0 = obj.forwardKinematics(q);
                p0 = T0(1:3, 4);
                delta = 1e-6;
                
                for i = 1:7
                    q_perturb = q;
                    q_perturb(i) = q_perturb(i) + delta;
                    T_perturb = obj.forwardKinematics(q_perturb);
                    p_perturb = T_perturb(1:3, 4);
                    
                    % 線速度部分
                    J(1:3, i) = (p_perturb - p0) / delta;
                    
                    % 角速度部分 (簡化計算)
                    dR = (T_perturb(1:3, 1:3) - T0(1:3, 1:3)) / delta;
                    % 從旋轉矩陣導數提取角速度
                    w = [dR(3,2) - dR(2,3); dR(1,3) - dR(3,1); dR(2,1) - dR(1,2)] / 2;
                    J(4:6, i) = w;
                end
            end
        end
        
        %% 計算條件數
        function kappa = conditionNumber(obj, q)
            % conditionNumber 計算雅可比條件數
            %   kappa = arm.conditionNumber(q)
            %   用於評估奇異點接近程度
            
            J = obj.jacobian(q);
            J(1:3,:) = J(1:3,:) / 1000;  % 轉換為米
            kappa = cond(J);
        end
        
        %% 可視化
        function plot(obj, q)
            % plot 繪製機械臂
            %   arm.plot(q) 繪製機械臂在關節角q時的姿態
            %
            %   注意: 需要Robotics Toolbox
            
            if nargin < 2
                q = zeros(1, 7);
            end
            
            if obj.hasRoboticsToolbox
                q_display = obj.q0 + obj.q_start + q;
                obj.robot.plot(q_display);
            else
                error('Arm7R: plot功能需要Robotics Toolbox。請安裝Peter Corke的Robotics Toolbox。');
            end
        end
        
        function teach(obj)
            % teach 啟動交互式示教界面
            %
            %   注意: 需要Robotics Toolbox
            
            if obj.hasRoboticsToolbox
                obj.robot.teach();
            else
                error('Arm7R: teach功能需要Robotics Toolbox。請安裝Peter Corke的Robotics Toolbox。');
            end
        end
        
        %% 簡單可視化 (不依賴工具箱)
        function simplePlot(obj, q)
            % simplePlot 簡單繪製機械臂連桿
            %   arm.simplePlot(q) 不使用Robotics Toolbox繪製機械臂骨架
            
            if nargin < 2
                q = zeros(1, 7);
            end
            
            % 計算各關節位置
            t1 = q(1); t2 = q(2); t3 = q(3); t4 = q(4);
            t5 = q(5); t6 = q(6); t7 = q(7);
            
            % 計算各變換矩陣
            T01 = [-sin(t1), 0,  cos(t1), 0;
                    cos(t1), 0,  sin(t1), 0;
                    0,       1,  0,       obj.e;
                    0,       0,  0,       1];
                    
            T12 = [-sin(t2), 0,  cos(t2), 0;
                    cos(t2), 0,  sin(t2), 0;
                    0,       1,  0,       obj.k;
                    0,       0,  0,       1];
                    
            T23 = [cos(t3), -sin(t3), 0,  obj.l*cos(t3);
                    sin(t3),  cos(t3), 0,  obj.l*sin(t3);
                    0,        0,       1,  obj.i;
                    0,        0,       0,  1];
                    
            T34 = [cos(t4), -sin(t4), 0,  obj.n*cos(t4);
                    sin(t4),  cos(t4), 0,  obj.n*sin(t4);
                    0,        0,       1, -obj.m;
                    0,        0,       0,  1];
                    
            T45 = [cos(t5), 0, -sin(t5), 0;
                    sin(t5), 0,  cos(t5), 0;
                    0,      -1,  0,       0;
                    0,       0,  0,       1];
                    
            T56 = [sin(t6), 0,  cos(t6), 0;
                   -cos(t6), 0,  sin(t6), 0;
                    0,      -1,  0,       obj.j;
                    0,       0,  0,       1];
                    
            T67 = [cos(t7),  sin(t7), 0,  0;
                    sin(t7), -cos(t7), 0,  0;
                    0,        0,      -1, -obj.b;
                    0,        0,       0,  1];
            
            % 計算各關節在世界坐標系中的位置
            T02 = T01 * T12;
            T03 = T02 * T23;
            T04 = T03 * T34;
            T05 = T04 * T45;
            T06 = T05 * T56;
            T07 = T06 * T67;
            
            points = [0, 0, 0;              % Base
                      T01(1:3,4)';           % Joint 1
                      T02(1:3,4)';           % Joint 2
                      T03(1:3,4)';           % Joint 3
                      T04(1:3,4)';           % Joint 4
                      T05(1:3,4)';           % Joint 5
                      T06(1:3,4)';           % Joint 6
                      T07(1:3,4)'];          % End-effector
            
            % 繪製
            if isempty(get(groot, 'CurrentFigure'))
                % 沒有figure時創建，保持白色背景
                figure('Color', 'white');
            else
                % 已有figure，設置當前figure背景為白色
                set(gcf, 'Color', 'white');
            end
            hold on;
            
            % 繪製連桿
            plot3(points(:,1), points(:,2), points(:,3), 'b-o', 'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
            
            % 標註各關節
            labels = {'Base', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'EE'};
            for i = 1:size(points, 1)
                text(points(i,1), points(i,2), points(i,3), sprintf('  %s', labels{i}), 'FontSize', 9, 'FontWeight', 'bold');
            end
            
            % 添加坐標系標註 (在每個關節處繪製小坐標軸)
            axis_length_base = 50;  % Base坐標軸長度
            axis_length_end = 80;   % 末端坐標軸長度
            T_all = {eye(4), T01, T02, T03, T04, T05, T06, T07};
            colors = {'r', 'g', 'b'};  % X紅, Y綠, Z藍
            for i = 1:length(T_all)
                T_i = T_all{i};
                p = T_i(1:3, 4);
                % 只在Base和末端標註坐標系
                if i == 1
                    % Base坐標系
                    for j = 1:3
                        dir = T_i(1:3, j);
                        plot3([p(1), p(1)+axis_length_base*dir(1)], ...
                              [p(2), p(2)+axis_length_base*dir(2)], ...
                              [p(3), p(3)+axis_length_base*dir(3)], ...
                              colors{j}, 'LineWidth', 1.5);
                    end
                elseif i == length(T_all)
                    % 末端坐標系 (更長的軸，更粗的線)
                    for j = 1:3
                        dir = T_i(1:3, j);
                        plot3([p(1), p(1)+axis_length_end*dir(1)], ...
                              [p(2), p(2)+axis_length_end*dir(2)], ...
                              [p(3), p(3)+axis_length_end*dir(3)], ...
                              colors{j}, 'LineWidth', 2);
                    end
                end
            end
            
            % 設置坐標軸
            xlabel('X (mm)', 'FontSize', 12);
            ylabel('Y (mm)', 'FontSize', 12);
            zlabel('Z (mm)', 'FontSize', 12);

            
            grid on;
            box on;
            
            % 自動設置合適的坐標範圍
            margin = 200;
            x_min = min(points(:,1)) - margin; x_max = max(points(:,1)) + margin;
            y_min = min(points(:,2)) - margin; y_max = max(points(:,2)) + margin;
            z_min = min(points(:,3)) - margin; z_max = max(points(:,3)) + margin;
            
            % 確保範圍合理
            if x_min >= x_max, x_min = -500; x_max = 500; end
            if y_min >= y_max, y_min = -500; y_max = 500; end
            if z_min >= z_max, z_min = -100; z_max = 1000; end
            
            axis([x_min, x_max, y_min, y_max, z_min, z_max]);
            
            % 設置視角 (默認視角，可交互旋轉)
            view(45, 25);  % 初始視角
            
            % 設置透視
            camproj('perspective');
            
            % 啟用滑鼠左鍵旋轉
            rotate3d on;
            
            % 添加圖例
            legend('連桿', 'Location', 'best');
            
            hold off;
            
            % 輸出調試信息
            fprintf('simplePlot: 繪製了 %d 個關節點\n', size(points, 1));
            fprintf('  X範圍: [%.1f, %.1f]\n', min(points(:,1)), max(points(:,1)));
            fprintf('  Y範圍: [%.1f, %.1f]\n', min(points(:,2)), max(points(:,2)));
            fprintf('  Z範圍: [%.1f, %.1f]\n', min(points(:,3)), max(points(:,3)));
        end
        
        %% 獲取機械臂各關節位置
        function points = getJointPositions(obj, q)
            % getJointPositions 計算機械臂各關節在世界坐標系中的位置
            %   points = arm.getJointPositions(q) 返回關節位置矩陣
            %
            %   輸入:
            %       q - 1x7 關節角向量 (rad)，默認為零位
            %   輸出:
            %       points - 9x3 矩陣，每行為一個點的 [x, y, z] 坐標
            %                [基座; 關節1; 關節2; ...; 關節7/末端]
            
            if nargin < 2 || isempty(q)
                q = zeros(1, 7);
            end
            
            t1 = q(1); t2 = q(2); t3 = q(3); t4 = q(4);
            t5 = q(5); t6 = q(6); t7 = q(7);
            
            % 計算各變換矩陣
            T01 = [-sin(t1), 0,  cos(t1), 0;
                    cos(t1), 0,  sin(t1), 0;
                    0,       1,  0,       obj.e;
                    0,       0,  0,       1];
                    
            T12 = [-sin(t2), 0,  cos(t2), 0;
                    cos(t2), 0,  sin(t2), 0;
                    0,       1,  0,       obj.k;
                    0,       0,  0,       1];
                    
            T23 = [cos(t3), -sin(t3), 0,  obj.l*cos(t3);
                    sin(t3),  cos(t3), 0,  obj.l*sin(t3);
                    0,        0,       1,  obj.i;
                    0,        0,       0,  1];
                    
            T34 = [cos(t4), -sin(t4), 0,  obj.n*cos(t4);
                    sin(t4),  cos(t4), 0,  obj.n*sin(t4);
                    0,        0,       1, -obj.m;
                    0,        0,       0,  1];
                    
            T45 = [cos(t5), 0, -sin(t5), 0;
                    sin(t5), 0,  cos(t5), 0;
                    0,      -1,  0,       0;
                    0,       0,  0,       1];
                    
            T56 = [sin(t6), 0,  cos(t6), 0;
                   -cos(t6), 0,  sin(t6), 0;
                    0,      -1,  0,       obj.j;
                    0,       0,  0,       1];
                    
            T67 = [cos(t7),  sin(t7), 0,  0;
                    sin(t7), -cos(t7), 0,  0;
                    0,        0,      -1, -obj.b;
                    0,        0,       0,  1];
            
            % 計算累積變換矩陣
            T02 = T01 * T12;
            T03 = T02 * T23;
            T04 = T03 * T34;
            T05 = T04 * T45;
            T06 = T05 * T56;
            T07 = T06 * T67;
            
            % 提取各關節位置
            points = [
                0, 0, 0;           % 基座 (關節0)
                T01(1:3, 4)';      % 關節1
                T02(1:3, 4)';      % 關節2
                T03(1:3, 4)';      % 關節3
                T04(1:3, 4)';      % 關節4
                T05(1:3, 4)';      % 關節5
                T06(1:3, 4)';      % 關節6
                T07(1:3, 4)';      % 關節7/末端
            ];
        end
        
        %% DH變換矩陣
        function T = dhTransform(obj, theta, d, a, alpha)
            % dhTransform 計算單個DH變換矩陣
            %   T = arm.dhTransform(theta, d, a, alpha)
            %   
            %   使用標準DH參數約定
            
            T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                 0,           sin(alpha),             cos(alpha),            d;
                 0,           0,                      0,                     1];
        end
        
        %% 獲取末端位姿
        function T = getEndEffectorPose(obj, q)
            % getEndEffectorPose 獲取末端執行器位姿
            %   T = arm.getEndEffectorPose(q)
            %   同forwardKinematics，提供更直觀的函數名
            
            T = obj.forwardKinematics(q);
        end
        
        %% 設置關節限制
        function setJointLimits(obj, lower, upper)
            % setJointLimits 設置關節角度限制
            %   arm.setJointLimits(lower, upper)
            %   
            %   輸入:
            %       lower - 1x7 下限向量 (rad)
            %       upper - 1x7 上限向量 (rad)
            
            obj.q_limit_lower = lower;
            obj.q_limit_upper = upper;
        end
        
        %% 顯示DH參數表
        function displayDHTable(obj)
            % displayDHTable 顯示DH參數表
            
            fprintf('\n========== 7R機械臂DH參數表 ==========\n');
            fprintf('關節 | theta      | d          | a          | alpha\n');
            fprintf('-----|------------|------------|------------|------------\n');
            fprintf('  1  | pi/2       | %10.4f | %10.4f | pi/2\n', obj.e, 0);
            fprintf('  2  | pi/2       | %10.4f | %10.4f | pi/2\n', obj.k, 0);
            fprintf('  3  | 0          | %10.4f | %10.4f | 0\n', obj.i, obj.l);
            fprintf('  4  | 0          | %10.4f | %10.4f | 0\n', -obj.m, obj.n);
            fprintf('  5  | 0          | %10.4f | %10.4f | -pi/2\n', 0, 0);
            fprintf('  6  | -pi/2      | %10.4f | %10.4f | -pi/2\n', obj.j, 0);
            fprintf('  7  | 0          | %10.4f | %10.4f | pi\n', -obj.b, 0);
            fprintf('=====================================\n');
            fprintf('P_DH = [%s]\n', num2str(obj.P_DH, '%.4f, '));
            fprintf('=====================================\n\n');
        end
    end
end
