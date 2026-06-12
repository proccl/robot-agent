function [is_collision, min_dist] = checkRobotObstacleCollision(robot_tree, q, obstacle)
% checkRobotObstacleCollision 檢測機械臂與球形障礙物的碰撞
%   robot_tree: rigidBodyTree 對象
%   q:          1x7 關節角（弧度）
%   obstacle:   結構體，含 center (3x1)、radius (mm)
%   is_collision: bool，是否發生碰撞（含接觸）
%   min_dist:     機械臂與球體表面的最短距離（mm），負值表示穿透
%
%   依賴 Robotics System Toolbox 的 checkCollision 與 collisionSphere。

    if isempty(robot_tree)
        error('robot_tree is empty. Cannot perform collision check.');
    end
    
    q = q(:)';
    n_joints = countNonFixedJoints(robot_tree);
    if length(q) ~= n_joints
        error('Joint angle vector length (%d) does not match robot non-fixed joints (%d).', ...
              length(q), n_joints);
    end
    
    % 構建球形障礙物
    sphere_obj = collisionSphere(obstacle.radius);
    sphere_obj.Pose = trvec2tform(obstacle.center(:)');
    
    % 調用工具箱進行碰撞檢測
    [is_colliding, sep_dist] = checkCollision(robot_tree, q, {sphere_obj}, ...
                                              'SkippedSelfCollisions', 'parent');
    
    % is_colliding 為 [self_collision, world_collision] 兩元素向量
    is_collision = is_colliding(2);
    
    if is_collision
        % MATLAB 在碰撞時將距離設為 NaN，此處用 -Inf 表示已碰撞
        min_dist = -Inf;
    else
        % sep_dist 大小為 (NumBodies+1) x (NumBodies+1+NumWorldObjects)
        % 後 NumWorldObjects 列對應世界物體（此處為 1 個球）
        n_bodies = robot_tree.NumBodies;
        world_cols = (n_bodies + 2) : (n_bodies + 1 + 1);
        world_dists = sep_dist(:, world_cols);
        world_dists = world_dists(~isinf(world_dists) & ~isnan(world_dists));
        if isempty(world_dists)
            min_dist = Inf;
        else
            min_dist = min(world_dists(:));
        end
    end
end

function n = countNonFixedJoints(tree)
    n = 0;
    for k = 1:tree.NumBodies
        if ~strcmp(tree.Bodies{k}.Joint.Type, 'fixed')
            n = n + 1;
        end
    end
end
