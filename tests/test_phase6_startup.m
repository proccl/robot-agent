function test_phase6_startup()
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    addpath(fullfile(fileparts(script_path), '..', 'src'));
    addpath(fullfile(fileparts(script_path), '..', 'scripts'));
    
    fprintf('\n========================================\n');
    fprintf('  Phase 6 Tests: Startup & Helpers\n');
    fprintf('========================================\n\n');
    
    psPath = fullfile(fileparts(script_path), '..', 'scripts', 'send_robot_cmd.ps1');
    
    % --- P6-T1: run_robot_agent exists ---
    fprintf('[P6-T1] run_robot_agent function... ');
    assert(exist('run_robot_agent', 'file') > 0, 'P6-T1: run_robot_agent.m not found');
    fprintf('OK\n');
    
    % --- P6-T2: PowerShell script exists ---
    fprintf('[P6-T2] send_robot_cmd.ps1 exists... ');
    assert(exist(psPath, 'file') > 0, 'P6-T2: send_robot_cmd.ps1 not found');
    fprintf('OK\n');
    
    % --- P6-T3: TCP client send/receive ---
    fprintf('[P6-T3] TCP client send/receive... ');
    agent = RobotAgent();
    agent.startRenderLoop();
    agent.start(12345);
    port = agent.port;
    pause(0.5);
    
    c = tcpclient('127.0.0.1', port, 'Timeout', 5);
    writeline(c, '{"cmd":"get_status"}');
    flush(c);
    pause(0.5);
    resp_str = readline(c);
    assert(~isempty(resp_str), 'P6-T3: empty response');
    assert(contains(resp_str, 'status'), 'P6-T3: response missing status');
    clear c;
    fprintf('OK\n');
    
    % --- P6-T4: non-default port ---
    fprintf('[P6-T4] non-default port... ');
    agent.stop();
    agent.start(12346);
    port2 = agent.port;
    pause(0.5);
    c2 = tcpclient('127.0.0.1', port2, 'Timeout', 5);
    writeline(c2, '{"cmd":"get_status"}');
    flush(c2);
    resp_str2 = '';
    tic;
    while toc < 5
        if c2.NumBytesAvailable > 0
            resp_str2 = readline(c2);
            break;
        end
        pause(0.1);
    end
    assert(~isempty(resp_str2), 'P6-T4: empty response on port %d', port2);
    clear c2;
    fprintf('OK (port %d)\n', port2);
    
    % --- P6-T5: server restart ---
    fprintf('[P6-T5] server restart... ');
    agent.stop();
    pause(0.5);
    agent.start(port);
    port3 = agent.port;
    pause(0.5);
    c3 = tcpclient('127.0.0.1', port3, 'Timeout', 5);
    writeline(c3, '{"cmd":"get_status"}');
    flush(c3);
    resp_str3 = '';
    tic;
    while toc < 5
        if c3.NumBytesAvailable > 0
            resp_str3 = readline(c3);
            break;
        end
        pause(0.1);
    end
    assert(~isempty(resp_str3), 'P6-T5: empty response after restart');
    clear c3;
    fprintf('OK\n');
    
    agent.stop();
    if isvalid(agent.render_timer)
        stop(agent.render_timer);
        delete(agent.render_timer);
    end
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 6 Tests PASSED (5/5)\n');
    fprintf('========================================\n');
end
