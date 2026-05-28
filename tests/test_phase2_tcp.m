function test_phase2_tcp()
    % test_phase2_tcp Phase 2 測試：TCP 通信與 JSON 解析
    
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    addpath(fullfile(fileparts(script_path), '..', 'src'));
    
    fprintf('========================================\n');
    fprintf('  Phase 2 Tests: TCP Communication\n');
    fprintf('========================================\n\n');
    
    %% P2-T1: 服務器啟動（默認端口 12345）
    fprintf('[P2-T1] Server start on default port... ');
    agent = RobotAgent();
    agent.start(12345);
    pause(0.5);
    assert(agent.is_running, 'Server is_running flag false');
    assert(agent.port >= 12345, 'Port should be >= 12345');
    fprintf('OK (port %d)\n', agent.port);
    
    %% P2-T2: 端口被佔用時自動遞增
    fprintf('[P2-T2] Port auto-increment when occupied... ');
    agent2 = RobotAgent();
    agent2.start(12345); % 12345 已被 agent 佔用
    pause(0.5);
    assert(agent2.port > 12345, 'Port should auto-increment above 12345');
    fprintf('OK (port %d)\n', agent2.port);
    
    %% P2-T3: 發送 get_status
    fprintf('[P2-T3] Send {"cmd":"get_status"}... ');
    client = tcpclient('127.0.0.1', agent.port);
    writeline(client, '{"cmd":"get_status"}');
    pause(0.3);
    response = readline(client);
    resp_struct = jsondecode(char(response));
    assert(strcmp(resp_struct.status, 'ok'), 'Status should be ok');
    assert(isfield(resp_struct, 'joint_angles_rad'), 'Missing joint_angles_rad');
    assert(isfield(resp_struct, 'end_effector_position'), 'Missing end_effector_position');
    fprintf('OK (received valid status)\n');
    
    %% P2-T4: 發送無效 JSON
    fprintf('[P2-T4] Send invalid JSON... ');
    writeline(client, 'this-is-not-json');
    pause(0.3);
    response = readline(client);
    resp_struct = jsondecode(char(response));
    assert(strcmp(resp_struct.status, 'error'), 'Should return error for invalid JSON');
    fprintf('OK (returned error)\n');
    
    %% P2-T5: 發送缺少 cmd 字段
    fprintf('[P2-T5] Send JSON missing cmd field... ');
    writeline(client, '{"foo":"bar"}');
    pause(0.3);
    response = readline(client);
    resp_struct = jsondecode(char(response));
    assert(strcmp(resp_struct.status, 'error'), 'Should return error for missing cmd');
    fprintf('OK (returned error)\n');
    
    %% P2-T6: 連續發送 10 條指令
    fprintf('[P2-T6] 10 consecutive commands... ');
    for i = 1:10
        writeline(client, '{"cmd":"get_status"}');
        pause(0.1);
        response = readline(client);
        resp_struct = jsondecode(char(response));
        assert(strcmp(resp_struct.status, 'ok'), 'Command %d failed', i);
    end
    fprintf('OK (10/10 passed)\n');
    
    %% 清理
    clear client;
    agent.stop();
    agent2.stop();
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 2 Tests PASSED (6/6)\n');
    fprintf('========================================\n');
end
