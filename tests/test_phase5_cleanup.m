%% test_phase5_cleanup.m — Phase 5: 清理舊架構與文檔一致性測試

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

fprintf('=== Phase 5: Cleanup & Documentation Tests ===\n');

%% P5-T1: 舊 RobotAgent.m 已刪除
fprintf('[P5-T1] Old RobotAgent.m removed... ');
root_dir = fullfile(fileparts(mfilename('fullpath')), '..');
assert(~exist(fullfile(root_dir, 'src', 'RobotAgent.m'), 'file'), 'P5-T1 FAILED: RobotAgent.m still exists');
fprintf('PASS\n');

%% P5-T2: 舊 TCP 腳本已刪除
fprintf('[P5-T2] Old TCP scripts removed... ');
assert(~exist(fullfile(root_dir, 'send_cmd.m'), 'file'), 'P5-T2 FAILED: send_cmd.m still exists');
temp_files = dir(fullfile(root_dir, 'temp_*.m'));
assert(isempty(temp_files), 'P5-T2 FAILED: temp_*.m files still exist');
assert(~exist(fullfile(root_dir, 'scripts', 'run_robot_agent.m'), 'file'), 'P5-T2 FAILED: run_robot_agent.m still exists');
assert(~exist(fullfile(root_dir, 'scripts', 'send_robot_cmd.ps1'), 'file'), 'P5-T2 FAILED: send_robot_cmd.ps1 still exists');
fprintf('PASS\n');

%% P5-T3: 新 robotagent.m 存在且可啟動（只驗證文件存在，不運行）
fprintf('[P5-T3] New robotagent.m exists... ');
assert(exist(fullfile(root_dir, 'robotagent.m'), 'file'), 'P5-T3 FAILED: robotagent.m not found');
fprintf('PASS\n');

%% P5-T4: 所有新函數可訪問
fprintf('[P5-T4] New functions accessible... ');
assert(~isempty(which('initRobotFigure')), 'P5-T4 FAILED: initRobotFigure not on path');
assert(~isempty(which('updateRobotFigure')), 'P5-T4 FAILED: updateRobotFigure not on path');
assert(~isempty(which('animateRobot')), 'P5-T4 FAILED: animateRobot not on path');
assert(~isempty(which('quinticTrajectory')), 'P5-T4 FAILED: quinticTrajectory not on path');
assert(~isempty(which('generate_robot_cmd')), 'P5-T4 FAILED: generate_robot_cmd not on path');
assert(~isempty(which('parseNaturalLanguage')), 'P5-T4 FAILED: parseNaturalLanguage not on path');
assert(~isempty(which('processIncomingCommands')), 'P5-T4 FAILED: processIncomingCommands not on path');
fprintf('PASS\n');

%% P5-T5: README 提及新架構
fprintf('[P5-T5] README mentions new architecture... ');
fid = fopen(fullfile(root_dir, 'README.md'), 'r');
readme = fread(fid, '*char')';
fclose(fid);
assert(contains(readme, 'file-system queue'), 'P5-T5 FAILED: README missing file-system queue');
assert(contains(readme, 'quinticTrajectory'), 'P5-T5 FAILED: README missing quinticTrajectory');
assert(contains(readme, 'natural language'), 'P5-T5 FAILED: README missing natural language');
assert(~contains(readme, 'tcpserver'), 'P5-T5 FAILED: README still mentions tcpserver');
fprintf('PASS\n');

%% P5-T6: AGENTS.md 提及新架構
fprintf('[P5-T6] AGENTS.md mentions new architecture... ');
fid = fopen(fullfile(root_dir, 'AGENTS.md'), 'r');
agents = fread(fid, '*char')';
fclose(fid);
assert(contains(agents, '文件隊列'), 'P5-T6 FAILED: AGENTS.md missing 文件隊列');
assert(contains(agents, 'Quintic Polynomial'), 'P5-T6 FAILED: AGENTS.md missing Quintic');
assert(contains(agents, '5 秒') || contains(agents, '5s'), 'P5-T6 FAILED: AGENTS.md missing 5s default');
assert(~contains(agents, 'tcpserver'), 'P5-T6 FAILED: AGENTS.md still mentions tcpserver');
fprintf('PASS\n');

fprintf('\n=== Phase 5: ALL TESTS PASSED ===\n');
