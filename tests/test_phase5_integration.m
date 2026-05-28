function test_phase5_integration()
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    addpath(fullfile(fileparts(script_path), '..', 'src'));
    
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
    
    resp = sendTCPCommand(port, struct('cmd', 'home', 'duration', 1));
    assert(strcmp(resp.status, 'ok'), 'Home command failed: %s', resp.message);
    max_wait = 5; tic;
    while agent.is_busy && toc < max_wait, pause(0.2); end
    assert(~agent.is_busy, 'Home animation did not finish');
    fprintf('OK\n');
    
    agent.stop();
    if isvalid(agent.render_timer)
        stop(agent.render_timer);
        delete(agent.render_timer);
    end
end

function resp = sendTCPCommand(port, cmd)
    json_str = jsonencode(cmd);
    c = tcpclient('127.0.0.1', port, 'Timeout', 5);
    writeline(c, json_str);
    flush(c);
    pause(0.5);
    resp_str = readline(c);
    fprintf('Local function RESP: [%s]\n', resp_str);
    clear c;
    resp = jsondecode(resp_str);
end
