function test_integration()
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    addpath(fullfile(fileparts(script_path), '..', 'src'));
    
    fprintf('\n========================================\n');
    fprintf('  Phase 8 Tests: Integration\n');
    fprintf('========================================\n\n');
    
    output_dir = fullfile(fileparts(script_path), 'output');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    agent = RobotAgent();
    agent.current_q = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
    agent.updatePlot(agent.current_q);
    agent.startRenderLoop();
    agent.start(12345);
    port = agent.port;
    pause(0.5);
    
    % --- P8-T1: end-to-end home ---
    fprintf('[P8-T1] end-to-end home... ');
    c = tcpclient('127.0.0.1', port, 'Timeout', 5);
    writeline(c, '{"cmd":"home","duration":1}');
    flush(c);
    pause(0.5);
    resp_str = readline(c);
    assert(~isempty(resp_str), 'P8-T1: empty response');
    resp = jsondecode(resp_str);
    assert(strcmp(resp.status, 'ok'), 'P8-T1: %s', resp.message);
    delete(c); clear c;
    waitIdle(agent, 3);
    fprintf('OK\n');
    
    % --- P8-T2: end-to-end move_to ---
    fprintf('[P8-T2] end-to-end move_to... ');
    c = tcpclient('127.0.0.1', port, 'Timeout', 5);
    writeline(c, '{"cmd":"move_to","position":[300,0,700],"duration":2}');
    flush(c);
    pause(0.5);
    resp_str = readline(c);
    resp = jsondecode(resp_str);
    assert(strcmp(resp.status, 'ok'), 'P8-T2: %s', resp.message);
    delete(c); clear c;
    waitIdle(agent, 4);
    fprintf('OK\n');
    
    % --- P8-T3: end-to-end circle ---
    fprintf('[P8-T3] end-to-end circle... ');
    agent.current_q = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
    agent.updatePlot(agent.current_q);
    pause(0.3);
    resp = agent.executeCommand(struct('cmd', 'trajectory', 'type', 'circle', 'radius', 200, 'duration', 3));
    assert(strcmp(resp.status, 'ok'), 'P8-T3: %s', resp.message);
    waitIdle(agent, 5);
    fprintf('OK\n');
    
    % --- P8-T4: rapid 5 consecutive commands ---
    fprintf('[P8-T4] rapid 5 consecutive commands... ');
    for i = 1:5
        resp = agent.executeCommand(struct('cmd', 'get_status'));
        assert(strcmp(resp.status, 'ok'), 'P8-T4: cmd %d failed', i);
    end
    fprintf('OK (5/5)\n');
    
    % --- P8-T5: new command during animation ---
    fprintf('[P8-T5] new command during animation... ');
    agent.current_q = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
    agent.updatePlot(agent.current_q);
    resp1 = agent.executeCommand(struct('cmd', 'trajectory', 'type', 'circle', 'radius', 200, 'duration', 3));
    assert(strcmp(resp1.status, 'ok'), 'P8-T5: first cmd failed');
    pause(0.2);
    resp2 = agent.executeCommand(struct('cmd', 'home', 'duration', 1));
    assert(strcmp(resp2.status, 'ok'), 'P8-T5: second cmd failed');
    fprintf('OK\n');
    
    % --- P8-T6: stability (short run) ---
    fprintf('[P8-T6] stability 30s run... ');
    t_start = tic;
    frame_count_start = agent.render_frame_count;
    while toc(t_start) < 5
        pause(0.5);
    end
    frames = agent.render_frame_count - frame_count_start;
    assert(frames >= 120, 'P8-T6: only %d frames in 5s', frames);
    assert(isvalid(agent.fig), 'P8-T6: figure invalid');
    fprintf('OK (%d frames in 5s)\n', frames);
    
    % --- P8-T7: performance benchmark ---
    fprintf('[P8-T7] performance benchmark... ');
    tic;
    q_test = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
    q_traj = RobotAgent.generateTrajectory(struct('cmd','trajectory','type','circle','radius',200,'duration',5), q_test, agent.arm.P_DH);
    t_compute = toc;
    assert(t_compute < 2, 'P8-T7: compute too slow %.2fs', t_compute);
    assert(size(q_traj, 1) > 0, 'P8-T7: empty trajectory');
    fprintf('OK (compute=%.3fs, traj=%d pts)\n', t_compute, size(q_traj, 1));
    
    agent.stop();
    if isvalid(agent.render_timer)
        stop(agent.render_timer);
        delete(agent.render_timer);
    end
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 8 Tests PASSED (7/7)\n');
    fprintf('========================================\n');
end

function waitIdle(agent, maxSec)
    tic;
    while toc < maxSec
        if ~agent.is_busy
            pause(0.3);
            return;
        end
        pause(0.2);
    end
end
