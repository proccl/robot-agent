function test_build_robot_tree()
% test_build_robot_tree 驗證 rigidBodyTree 與 Arm7R 正向運動學一致性

    fprintf('[Test] buildRobotTree: FK consistency with Arm7R ...\n');
    
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'src'));
    
    arm = Arm7R();
    tree = buildRobotTree(arm);
    
    assert(tree.NumBodies == 14, 'Expected 14 bodies (7 dummy + 7 actual)');
    
    rng(42);
    n_samples = 20;
    max_pos_err = 0;
    max_rot_err = 0;
    
    for k = 1:n_samples
        q = (rand(1, 7) * 2 - 1) * pi;
        T_arm = arm.forwardKinematics(q);
        T_tree = getTransform(tree, q, 'body7', 'base');
        
        pos_err = norm(T_arm(1:3, 4) - T_tree(1:3, 4));
        rot_err = norm(T_arm(1:3, 1:3) - T_tree(1:3, 1:3), 'fro');
        max_pos_err = max(max_pos_err, pos_err);
        max_rot_err = max(max_rot_err, rot_err);
        
        assert(pos_err < 1e-9, 'Position mismatch too large at sample %d: %g', k, pos_err);
        assert(rot_err < 1e-9, 'Rotation mismatch too large at sample %d: %g', k, rot_err);
    end
    
    fprintf('[PASS] buildRobotTree: max position error = %g, max rotation error = %g\n', max_pos_err, max_rot_err);
end
