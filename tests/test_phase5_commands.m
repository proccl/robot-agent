function test_phase5_commands()
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    addpath(fullfile(fileparts(script_path), '..', 'src'));
    
    fprintf('\n========================================\n');
    fprintf('  Phase 5 Tests: Command Correctness\n');
    fprintf('========================================\n\n');
    
    output_dir = fullfile(fileparts(script_path), 'output');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    agent = RobotAgent();
    agent.current_q = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
    agent.updatePlot(agent.current_q);
    agent.startRenderLoop();
    pause(0.5);
    
    % --- P5-T1: home command ---
    fprintf('[P5-T1] home command... ');
    resp = agent.executeCommand(struct('cmd', 'home', 'duration', 1));
    assert(strcmp(resp.status, 'ok'), 'P5-T1 failed: %s', resp.message);
    waitBusy(agent, 5);
    assert(norm(agent.current_q) < 0.01, 'P5-T1: not at home');
    printSave(agent, fullfile(output_dir, 'test_p5_t1_home.png'));
    fprintf('OK (at home)\n');
    
    % --- P5-T2: move_to [300,0,700] ---
    fprintf('[P5-T2] move_to [300,0,700]... ');
    resp = agent.executeCommand(struct('cmd', 'move_to', 'position', [300, 0, 700], 'duration', 2));
    assert(strcmp(resp.status, 'ok'), 'P5-T2 failed: %s', resp.message);
    waitBusy(agent, 5);
    T = agent.arm.forwardKinematics(agent.current_q);
    errPos = norm(T(1:3, 4)' - [300, 0, 700]);
    assert(errPos < 1, 'P5-T2: position error %.2f mm', errPos);
    printSave(agent, fullfile(output_dir, 'test_p5_t2_move_to.png'));
    fprintf('OK (error=%.2f mm)\n', errPos);
    
    % --- P5-T3: joint_move joint1 to 90 deg ---
    fprintf('[P5-T3] joint_move joint1 90 deg... ');
    resp = agent.executeCommand(struct('cmd', 'joint_move', 'joint', 1, 'angle', 90, 'angle_deg', true, 'duration', 1));
    assert(strcmp(resp.status, 'ok'), 'P5-T3 failed: %s', resp.message);
    waitBusy(agent, 5);
    assert(abs(agent.current_q(1) - pi/2) < 0.01, 'P5-T3: joint1=%.3f', agent.current_q(1));
    printSave(agent, fullfile(output_dir, 'test_p5_t3_joint_move.png'));
    fprintf('OK (q1=%.3f rad)\n', agent.current_q(1));
    
    % Reset to workspace-friendly pose for trajectory tests
    agent.current_q = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
    agent.updatePlot(agent.current_q);
    pause(0.3);
    
    % --- P5-T4: trajectory circle ---
    fprintf('[P5-T4] trajectory circle r=200... ');
    resp = agent.executeCommand(struct('cmd', 'trajectory', 'type', 'circle', 'radius', 200, 'duration', 3));
    assert(strcmp(resp.status, 'ok'), 'P5-T4 failed: %s', resp.message);
    waitBusy(agent, 8);
    assert(~isempty(agent.trajectory_queue), 'P5-T4: queue empty');
    fprintf('OK (%d points)\n', size(agent.trajectory_queue, 1));
    
    % --- P5-T5: trajectory line ---
    fprintf('[P5-T5] trajectory line to [600,100,700]... ');
    resp = agent.executeCommand(struct('cmd', 'trajectory', 'type', 'line', 'target', [600, 100, 700], 'duration', 2));
    assert(strcmp(resp.status, 'ok'), 'P5-T5 failed: %s', resp.message);
    waitBusy(agent, 5);
    T = agent.arm.forwardKinematics(agent.current_q);
    errPos = norm(T(1:3, 4)' - [600, 100, 700]);
    assert(errPos < 100, 'P5-T5: position error %.2f mm', errPos);
    printSave(agent, fullfile(output_dir, 'test_p5_t5_line.png'));
    fprintf('OK (error=%.2f mm)\n', errPos);
    
    % --- P5-T6: set_speed 2x ---
    fprintf('[P5-T6] set_speed 2x... ');
    resp = agent.executeCommand(struct('cmd', 'set_speed', 'factor', 2.0));
    assert(strcmp(resp.status, 'ok'), 'P5-T6 failed: %s', resp.message);
    assert(abs(agent.anim_speed - 2.0) < 0.01, 'P5-T6: speed=%.1f', agent.anim_speed);
    fprintf('OK (speed=%.1f)\n', agent.anim_speed);
    
    % --- P5-T7: get_status ---
    fprintf('[P5-T7] get_status... ');
    resp = agent.executeCommand(struct('cmd', 'get_status'));
    assert(strcmp(resp.status, 'ok'), 'P5-T7 failed: %s', resp.message);
    assert(isfield(resp, 'joint_angles_rad'), 'P5-T7: missing joint_angles_rad');
    assert(isfield(resp, 'end_effector_position'), 'P5-T7: missing end_effector_position');
    assert(isfield(resp, 'end_effector_rotation'), 'P5-T7: missing end_effector_rotation');
    assert(isfield(resp, 'anim_speed'), 'P5-T7: missing anim_speed');
    assert(isfield(resp, 'is_busy'), 'P5-T7: missing is_busy');
    fprintf('OK (all fields present)\n');
    
    % --- P5-T8: FK/IK consistency ---
    fprintf('[P5-T8] FK/IK consistency... ');
    pass_count = 0;
    for test_i = 1:20
        q_test = [rand()*pi, rand()*pi, rand()*pi, rand()*pi, rand()*pi, rand()*pi, rand()*pi];
        T1 = agent.arm.forwardKinematics(q_test);
        [q_ik, err] = agent.arm.inverseKinematics(T1);
        if err == 0
            T2 = agent.arm.forwardKinematics(q_ik);
            if norm(T1 - T2, 'fro') < 0.1
                pass_count = pass_count + 1;
            end
        end
    end
    assert(pass_count >= 10, 'P5-T8: only %d/20 passed', pass_count);
    fprintf('OK (%d/20 passed)\n', pass_count);
    
    % --- P5-T9: error handling ---
    fprintf('[P5-T9] error handling... ');
    resp = agent.executeCommand(struct('cmd', 'nonexistent'));
    assert(strcmp(resp.status, 'error'), 'P5-T9a: should be error');
    resp = agent.executeCommand(struct('cmd', 'move_to'));
    assert(strcmp(resp.status, 'error'), 'P5-T9b: should be error');
    resp = agent.executeCommand(struct('cmd', 'trajectory'));
    assert(strcmp(resp.status, 'error'), 'P5-T9c: should be error');
    fprintf('OK (all errors caught)\n');
    
    if isvalid(agent.render_timer)
        stop(agent.render_timer);
        delete(agent.render_timer);
    end
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 5 Tests PASSED (9/9)\n');
    fprintf('========================================\n');
end

function waitBusy(agent, maxSec)
    tic;
    while toc < maxSec
        if isempty(agent.trajectory_queue) || agent.queue_idx > size(agent.trajectory_queue, 1)
            pause(0.3);
            return;
        end
        pause(0.2);
    end
end

function printSave(agent, path)
    if isvalid(agent.fig)
        print(agent.fig, '-dpng', path);
    end
end
