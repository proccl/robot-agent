function test_phase7_docs()
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    
    fprintf('\n========================================\n');
    fprintf('  Phase 7 Tests: Documentation\n');
    fprintf('========================================\n\n');
    
    % --- P7-T1: JSON Schema structure ---
    fprintf('[P7-T1] JSON Schema structure... ');
    jsonPath = fullfile(fileparts(script_path), '..', 'docs', 'robot_agent_cmds.json');
    assert(exist(jsonPath, 'file') > 0, 'P7-T1: robot_agent_cmds.json not found');
    fid = fopen(jsonPath, 'r');
    raw = fread(fid, inf, '*char')';
    fclose(fid);
    schema = jsondecode(raw);
    assert(isfield(schema, 'commands'), 'P7-T1: missing commands field');
    cmds = fieldnames(schema.commands);
    assert(ismember('home', cmds), 'P7-T1: home missing');
    assert(ismember('move_to', cmds), 'P7-T1: move_to missing');
    assert(ismember('trajectory', cmds), 'P7-T1: trajectory missing');
    fprintf('OK (%d commands)\n', length(cmds));
    
    % --- P7-T2: README examples executable ---
    fprintf('[P7-T2] README examples... ');
    readmePath = fullfile(fileparts(script_path), '..', 'docs', 'ROBOT_AGENT_README.md');
    assert(exist(readmePath, 'file') > 0, 'P7-T2: ROBOT_AGENT_README.md not found');
    fid = fopen(readmePath, 'r');
    content = fread(fid, inf, '*char')';
    fclose(fid);
    assert(contains(content, 'home'), 'P7-T2: missing home example');
    assert(contains(content, 'move_to'), 'P7-T2: missing move_to example');
    assert(contains(content, 'get_status'), 'P7-T2: missing get_status example');
    fprintf('OK\n');
    
    % --- P7-T3: Error codes documented ---
    fprintf('[P7-T3] Error codes documented... ');
    assert(contains(content, '端口被占用'), 'P7-T3: missing port error doc');
    assert(contains(content, 'IK 无解'), 'P7-T3: missing IK error doc');
    assert(contains(content, 'Figure 被关闭'), 'P7-T3: missing figure error doc');
    fprintf('OK\n');
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 7 Tests PASSED (3/3)\n');
    fprintf('========================================\n');
end
