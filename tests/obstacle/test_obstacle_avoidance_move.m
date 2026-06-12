function test_obstacle_avoidance_move()
% test_obstacle_avoidance_move 驗證避障規劃能繞過障礙物到達目標

    fprintf('[Test] planTrajectoryWithObstacle ...\n');
    
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'src'));
    
    arm = Arm7R();
    tree = buildRobotTree(arm);
    
    % 障礙物擋在直線路徑上：home 末端 [266,-93,696] 到目標 [300,-300,600]
    % 直線中點約 [283,-196,648]，將障礙物放在偏離中點但不接觸起終點的位置
    obstacle = struct('center', [290; -250; 640], 'radius', 40);
    
    current_q = zeros(1, 7);
    T_start = arm.forwardKinematics(current_q);
    T_target = T_start;
    T_target(1:3, 4) = [300; -300; 600];
    
    duration = 5;
    [q_traj, avoided, info] = planTrajectoryWithObstacle(arm, current_q, T_target, obstacle, duration, tree);
    
    assert(~isempty(q_traj), 'Trajectory should not be empty. Info: %s', info);
    assert(avoided, 'Should have triggered obstacle avoidance for this setup');
    
    % 驗證軌跡不碰撞
    for k = 1:size(q_traj, 1)
        [isc, ~] = checkRobotObstacleCollision(tree, q_traj(k, :), obstacle);
        assert(~isc, 'Trajectory point %d collides with obstacle', k);
    end
    
    % 驗證終點接近目標
    T_final = arm.forwardKinematics(q_traj(end, :));
    pos_err = norm(T_final(1:3, 4) - T_target(1:3, 4));
    assert(pos_err < 1, 'Final position too far from target: %g mm', pos_err);
    
    fprintf('[PASS] planTrajectoryWithObstacle: avoided=%d, length=%d, final err=%.3f mm\n', ...
            avoided, size(q_traj, 1), pos_err);
end
