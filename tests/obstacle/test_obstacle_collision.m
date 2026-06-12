function test_obstacle_collision()
% test_obstacle_collision 驗證機械臂與球形障礙物的碰撞檢測

    fprintf('[Test] checkRobotObstacleCollision ...\n');
    
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'src'));
    
    arm = Arm7R();
    tree = buildRobotTree(arm);
    
    obstacle = struct('center', [800; 0; 0], 'radius', 100);
    
    % Home 位置應該不碰撞
    q_home = zeros(1, 7);
    [isc_home, dist_home] = checkRobotObstacleCollision(tree, q_home, obstacle);
    assert(~isc_home, 'Home position should not collide with obstacle');
    assert(dist_home > 0, 'Home position should be separated from obstacle');
    
    % 構造一個末端進入球內的配置
    % home 末端位於 [266.17, -93.5, 695.818]，沿 X 向球心移動
    T_target = arm.forwardKinematics(q_home);
    T_target(1:3, 4) = obstacle.center + [obstacle.radius * 0.5; 0; 0];  % 球內
    [q_inside, err] = arm.inverseKinematics(T_target);
    assert(err == 0, 'IK failed for inside-obstacle pose');
    
    [isc_inside, dist_inside] = checkRobotObstacleCollision(tree, q_inside, obstacle);
    assert(isc_inside, 'End-effector inside obstacle should report collision');
    assert(dist_inside <= 0, 'Inside obstacle should have non-positive separation distance');
    
    fprintf('[PASS] checkRobotObstacleCollision: home dist=%.3f, inside dist=%.3f\n', ...
            dist_home, dist_inside);
end
