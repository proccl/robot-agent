function test_obstacle_disabled()
% test_obstacle_disabled 驗證當軌跡不與障礙物碰撞時不觸發 RRT 避障

    fprintf('[Test] obstacle avoidance disabled (no collision) ...\n');
    
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'src'));
    
    arm = Arm7R();
    tree = buildRobotTree(arm);
    
    % 障礙物遠離直線路徑
    obstacle = struct('center', [1000; 1000; 1000], 'radius', 100);
    
    current_q = zeros(1, 7);
    T_start = arm.forwardKinematics(current_q);
    T_target = T_start;
    T_target(1:3, 4) = [300; -300; 600];
    
    duration = 5;
    [q_traj, avoided, info] = planTrajectoryWithObstacle(arm, current_q, T_target, obstacle, duration, tree);
    
    assert(~isempty(q_traj), 'Trajectory should not be empty. Info: %s', info);
    assert(~avoided, 'Should not trigger RRT when nominal path is collision-free');
    
    T_final = arm.forwardKinematics(q_traj(end, :));
    pos_err = norm(T_final(1:3, 4) - T_target(1:3, 4));
    assert(pos_err < 1, 'Final position too far from target: %g mm', pos_err);
    
    fprintf('[PASS] obstacle avoidance disabled: avoided=%d, length=%d, final err=%.3f mm\n', ...
            avoided, size(q_traj, 1), pos_err);
end
