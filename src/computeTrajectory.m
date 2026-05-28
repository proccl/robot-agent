function q_traj = computeTrajectory(cmd, current_q, P_DH)
    % computeTrajectory 獨立函數：根據指令生成關節角軌跡
    %   供 parfeval 後台調用。自動添加 src 目錄到 path 以確保 Arm7R 可找到。
    
    % 確保後台 worker 能找到 Arm7R
    script_dir = fileparts(mfilename('fullpath'));
    addpath(script_dir);
    
    arm = Arm7R(P_DH);
    R_current = eye(3);
    
    steps = 60;
    if isfield(cmd, 'duration')
        steps = max(20, round(cmd.duration * 30));
    end
    
    switch cmd.cmd
        case 'home'
            q_target = zeros(1, 7);
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
            [~, q_traj, ~] = arm.planTrajectoryCartesian(T_current, T_target, steps, 0, cmd.duration);
            if any(isnan(q_traj(:)))
                error('move_to: unreachable target');
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
                    if any(isnan(q_traj(:)))
                        error('line trajectory: unreachable target');
                    end
                    
                otherwise
                    error('Unknown trajectory type: %s', cmd.type);
            end
            
        otherwise
            error('Unknown command: %s', cmd.cmd);
    end
end
