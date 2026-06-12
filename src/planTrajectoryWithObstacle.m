function [q_traj, avoided, info] = planTrajectoryWithObstacle(arm, current_q, T_target, obstacle, duration, robot_tree)
% planTrajectoryWithObstacle 帶避障的軌跡規劃
%   arm:        Arm7R 對象
%   current_q:  1x7 起始關節角（弧度）
%   T_target:   4x4 目標齊次變換矩陣
%   obstacle:   結構體，含 center (3x1)、radius (mm)
%   duration:   運動時間（s），默認 5
%   robot_tree: 可選，預先構建的 rigidBodyTree；若未提供則現場構建
%
%   輸出：
%     q_traj - Nx7 關節角軌跡；規劃失敗時為空矩陣
%     avoided - bool，是否啟用了避障規劃（原始軌跡碰撞時才為 true）
%     info   - 文本信息，成功為空，失敗為 '[PAUSE] ...'
%
%   依賴 Robotics System Toolbox 的 checkCollision 與 manipulatorRRT。

    if nargin < 5 || isempty(duration)
        duration = 5;
    end
    if nargin < 6 || isempty(robot_tree)
        robot_tree = buildRobotTree(arm);
    end
    
    avoided = false;
    info = '';
    
    % 求解目標關節角
    [q_target, err] = arm.inverseKinematics(T_target);
    if err ~= 0
        q_traj = [];
        info = '[PAUSE] IK failed for target pose. Keeping current pose.';
        fprintf('%s\n', info);
        return;
    end
    
    % 生成原始笛卡爾軌跡
    fps = 30;
    steps = max(10, round(duration * fps));
    T_start = arm.forwardKinematics(current_q);
    [~, q_nominal] = arm.planTrajectoryCartesian(T_start, T_target, steps, 0, duration);
    
    % 檢查原始軌跡是否碰撞
    has_collision = false;
    for k = 1:size(q_nominal, 1)
        if any(isnan(q_nominal(k, :)))
            continue;
        end
        [isc, ~] = checkRobotObstacleCollision(robot_tree, q_nominal(k, :), obstacle);
        if isc
            has_collision = true;
            break;
        end
    end
    
    if ~has_collision
        q_traj = q_nominal;
        avoided = false;
        return;
    end
    
    % 需要避障：使用 manipulatorRRT 在關節空間規劃
    avoided = true;
    fprintf('[Info] Nominal trajectory collides with obstacle. Running RRT...\n');
    
    sphere_obj = collisionSphere(obstacle.radius);
    sphere_obj.Pose = trvec2tform(obstacle.center(:)');
    
    rrt = manipulatorRRT(robot_tree, {sphere_obj});
    rrt.SkippedSelfCollisions = 'parent';
    rng(0);  % 可重複
    [path, solnInfo] = plan(rrt, current_q, q_target);
    
    if ~solnInfo.IsPathFound
        q_traj = [];
        info = '[PAUSE] RRT planning failed. Obstacle may block all paths.';
        fprintf('%s\n', info);
        return;
    end
    
    % 將 RRT waypoints 轉換為平滑軌跡
    q_traj = waypointsToQuinticTrajectory(path, duration, fps);
    
    % 驗證平滑後的軌跡
    for k = 1:size(q_traj, 1)
        [isc, ~] = checkRobotObstacleCollision(robot_tree, q_traj(k, :), obstacle);
        if isc
            q_traj = [];
            info = '[PAUSE] Smoothed trajectory still collides. Aborting.';
            fprintf('%s\n', info);
            return;
        end
    end
end

function q_traj = waypointsToQuinticTrajectory(path, duration, fps)
% waypointsToQuinticTrajectory 將關節空間路點轉換為 Quintic 軌跡
%   path:     Mx7 路點
%   duration: 總運動時間（s）
%   fps:      輸出幀率

    n_wp = size(path, 1);
    if n_wp < 2
        q_traj = repmat(path(1, :), max(1, round(duration * fps)), 1);
        return;
    end
    
    % 按關節空間距離分配各段時間
    seg_dists = zeros(n_wp - 1, 1);
    for k = 1:n_wp - 1
        seg_dists(k) = norm(path(k + 1, :) - path(k, :));
    end
    total_dist = sum(seg_dists);
    
    if total_dist < eps
        q_traj = repmat(path(1, :), max(1, round(duration * fps)), 1);
        return;
    end
    
    q_traj = [];
    for k = 1:n_wp - 1
        seg_duration = duration * seg_dists(k) / total_dist;
        n_steps = max(2, round(seg_duration * fps));
        q_seg = quinticTrajectory(path(k, :), path(k + 1, :), seg_duration, fps);
        if k == 1
            q_traj = q_seg;
        else
            q_traj = [q_traj; q_seg(2:end, :)];
        end
    end
end
