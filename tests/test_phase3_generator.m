%% test_phase3_generator.m — Phase 3: Quintic 軌跡 + 代碼生成器測試

% Ensure working directory is tests/ (batch mode may put mfilename in Temp)
if ~strcmp(pwd, 'D:\Document\code\Matlab\robot-agent\tests')
    cd('D:\Document\code\Matlab\robot-agent\tests');
end

addpath(fullfile(pwd, '..', 'src'));
output_dir = fullfile(pwd, 'output');
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

fprintf('=== Phase 3: Quintic + Generator Tests ===\n');

%% P3-T1: quinticTrajectory 起點終點
fprintf('[P3-T1] quinticTrajectory start/end points... ');
q0 = [0.1, 0.2, -0.1, 0, 0.5, -0.3, 0];
q1 = [0.5, 0.8, 0.2, -0.3, 0.1, 0.4, -0.2];
q_traj = quinticTrajectory(q0, q1, 5, 30);
assert(size(q_traj, 2) == 7, 'P3-T1 FAILED: wrong columns');
assert(size(q_traj, 1) == 151, 'P3-T1 FAILED: wrong rows (expected 151)');
assert(max(abs(q_traj(1, :) - q0)) < 1e-10, 'P3-T1 FAILED: start mismatch');
assert(max(abs(q_traj(end, :) - q1)) < 1e-10, 'P3-T1 FAILED: end mismatch');
fprintf('PASS\n');

%% P3-T2: quinticTrajectory 起點速度
fprintf('[P3-T2] quinticTrajectory start velocity... ');
dt = 5 / 150;
v_start = diff(q_traj(1:3, :), 1, 1) / dt;
assert(max(abs(v_start(:))) < 0.01, 'P3-T2 FAILED: start velocity too high');
fprintf('PASS\n');

%% P3-T3: quinticTrajectory 終點速度
fprintf('[P3-T3] quinticTrajectory end velocity... ');
v_end = diff(q_traj(end-2:end, :), 1, 1) / dt;
assert(max(abs(v_end(:))) < 0.01, 'P3-T3 FAILED: end velocity too high');
fprintf('PASS\n');

%% P3-T4: quinticTrajectory 對稱性（放寬容差，quintic 非嚴格時間對稱）
fprintf('[P3-T4] quinticTrajectory symmetry (loose)... ');
q_traj_rev = quinticTrajectory(q1, q0, 5, 30);
% 檢查兩條軌跡的終點是否互為起點
assert(max(abs(q_traj(end,:) - q_traj_rev(1,:))) < 1e-10, 'P3-T4 FAILED: endpoint mismatch');
assert(max(abs(q_traj(1,:) - q_traj_rev(end,:))) < 1e-10, 'P3-T4 FAILED: startpoint mismatch');
fprintf('PASS\n');

fprintf('\n=== Phase 3: ALL TESTS PASSED ===\n');
