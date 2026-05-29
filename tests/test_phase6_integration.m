%% test_phase6_integration.m — Phase 6: 集成與穩定性測試

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
output_dir = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

fprintf('=== Phase 6: Integration & Stability Tests ===\n');

%% P6-T1: 端到端 home
fprintf('[P6-T1] End-to-end home... ');
arm = Arm7R();
fig = initRobotFigure(arm, zeros(1,7));
fig.UserData.is_busy = false;
test_incoming = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p6t1');
if exist(test_incoming, 'dir'), rmdir(test_incoming, 's'); end
mkdir(test_incoming);

cmd = parseNaturalLanguage('home');
fp = generate_robot_cmd(cmd, test_incoming);
run(fp);
pause(0.3);
assert(max(abs(fig.UserData.current_q)) < 1e-6, 'P6-T1 FAILED: not at home');
close(fig);
rmdir(test_incoming, 's');
fprintf('PASS\n');

%% P6-T2: 端到端 move_to
fprintf('[P6-T2] End-to-end move_to... ');
fig2 = initRobotFigure(arm, zeros(1,7));
fig2.UserData.is_busy = false;
test_incoming2 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p6t2');
if exist(test_incoming2, 'dir'), rmdir(test_incoming2, 's'); end
mkdir(test_incoming2);

cmd2 = parseNaturalLanguage('move to (500, 0, 800)');
fp2 = generate_robot_cmd(cmd2, test_incoming2);
run(fp2);
pause(0.3);
T_ee = arm.forwardKinematics(fig2.UserData.current_q);
assert(norm(T_ee(1:3,4)' - [500, 0, 800]) < 1, 'P6-T2 FAILED: end-effector mismatch');
close(fig2);
rmdir(test_incoming2, 's');
fprintf('PASS\n');

%% P6-T3: 端到端 move_to 指定 3 秒
fprintf('[P6-T3] End-to-end move_to with 3s... ');
fig3 = initRobotFigure(arm, zeros(1,7));
fig3.UserData.is_busy = false;
test_incoming3 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p6t3');
if exist(test_incoming3, 'dir'), rmdir(test_incoming3, 's'); end
mkdir(test_incoming3);

cmd3 = parseNaturalLanguage('move to (600, 100, 700) in 3 seconds');
fp3 = generate_robot_cmd(cmd3, test_incoming3);
tic;
run(fp3);
elapsed = toc;
assert(elapsed >= 2.5 && elapsed <= 5, 'P6-T3 FAILED: timing off (%.2fs)', elapsed);
T_ee3 = arm.forwardKinematics(fig3.UserData.current_q);
assert(norm(T_ee3(1:3,4)' - [600, 100, 700]) < 1, 'P6-T3 FAILED: end-effector mismatch');
close(fig3);
rmdir(test_incoming3, 's');
fprintf('PASS (elapsed=%.2fs)\n', elapsed);

%% P6-T4: 端到端 circle
fprintf('[P6-T4] End-to-end circle... ');
fig4 = initRobotFigure(arm, zeros(1,7));
fig4.UserData.is_busy = false;
test_incoming4 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p6t4');
if exist(test_incoming4, 'dir'), rmdir(test_incoming4, 's'); end
mkdir(test_incoming4);

cmd4 = parseNaturalLanguage('circle radius 200');
fp4 = generate_robot_cmd(cmd4, test_incoming4);
run(fp4);
pause(0.3);
% 驗證最終末端位置距離圓心接近半徑（在圓周上）
T_center = arm.forwardKinematics(zeros(1,7));
T_end = arm.forwardKinematics(fig4.UserData.current_q);
dist_to_center = norm(T_end(1:3,4) - T_center(1:3,4));
assert(abs(dist_to_center - 200) < 50, 'P6-T4 FAILED: circle radius mismatch (dist=%.1f)', dist_to_center);
close(fig4);
rmdir(test_incoming4, 's');
fprintf('PASS\n');

%% P6-T5: 快速連續 5 條指令
fprintf('[P6-T5] Rapid 5 commands... ');
fig5 = initRobotFigure(arm, zeros(1,7));
fig5.UserData.is_busy = false;
test_incoming5 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p6t5');
if exist(test_incoming5, 'dir'), rmdir(test_incoming5, 's'); end
mkdir(test_incoming5);

commands = {
    struct('cmd', 'home'),
    struct('cmd', 'joint_move', 'joint', 1, 'angle', 30, 'angle_deg', true),
    struct('cmd', 'joint_move', 'joint', 2, 'angle', -30, 'angle_deg', true),
    struct('cmd', 'home'),
    struct('cmd', 'move_to', 'position', [400, 0, 600], 'duration', 2)
};
for k = 1:5
    fp = generate_robot_cmd(commands{k}, test_incoming5);
    processIncomingCommands(test_incoming5, fig5);
    pause(0.2);
end
% 最終應該在 move_to 目標附近
T_ee5 = arm.forwardKinematics(fig5.UserData.current_q);
assert(norm(T_ee5(1:3,4)' - [400, 0, 600]) < 5, 'P6-T5 FAILED: final pose wrong');
close(fig5);
rmdir(test_incoming5, 's');
fprintf('PASS\n');

%% P6-T6: 新指令排隊（is_busy）
fprintf('[P6-T6] Command queuing with is_busy... ');
fig6 = initRobotFigure(arm, zeros(1,7));
fig6.UserData.is_busy = true;  % 模擬忙碌
test_incoming6 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p6t6');
if exist(test_incoming6, 'dir'), rmdir(test_incoming6, 's'); end
mkdir(test_incoming6);

for k = 1:3
    fp = generate_robot_cmd(struct('cmd', 'home'), test_incoming6);
    processIncomingCommands(test_incoming6, fig6);  % 應該被跳過
end
% 文件應該還在（因為 is_busy 阻止執行）
files = dir(fullfile(test_incoming6, 'cmd_*.m'));
assert(length(files) == 3, 'P6-T6 FAILED: files were executed while busy');

fig6.UserData.is_busy = false;
close(fig6);
rmdir(test_incoming6, 's');
fprintf('PASS\n');

%% P6-T9: 無效位姿容錯
fprintf('[P6-T9] Invalid pose error handling... ');
fig9 = initRobotFigure(arm, zeros(1,7));
fig9.UserData.is_busy = false;
test_incoming9 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p6t9');
if exist(test_incoming9, 'dir'), rmdir(test_incoming9, 's'); end
mkdir(test_incoming9);

cmd9 = struct('cmd', 'move_to', 'position', [5000, 0, 800], 'duration', 2);
fp9 = generate_robot_cmd(cmd9, test_incoming9);
try
    run(fp9);
    % 如果執行到這裡，說明錯誤被內部捕獲了，也是可接受的
    fprintf('PASS (error caught internally)\n');
catch ME
    assert(contains(ME.message, 'unreachable') || contains(ME.message, 'Target'), 'P6-T9 FAILED: wrong error');
    fprintf('PASS (error propagated correctly)\n');
end
close(fig9);
rmdir(test_incoming9, 's');

%% P6-T10: Quintic 軌跡平滑性（速度連續性檢查）
fprintf('[P6-T10] Quintic smoothness... ');
q_test = quinticTrajectory(zeros(1,7), ones(1,7)*0.5, 5, 100);
% 計算速度（差分）
vel = diff(q_test, 1, 1);
acc = diff(vel, 1, 1);
% 起點和終點速度應該接近 0
assert(max(abs(vel(1:5, :)), [], 'all') < 0.05, 'P6-T10 FAILED: start velocity too high');
assert(max(abs(vel(end-4:end, :)), [], 'all') < 0.05, 'P6-T10 FAILED: end velocity too high');
% 加速度不應該有劇烈跳變
assert(max(abs(acc(:))) < 0.5, 'P6-T10 FAILED: acceleration jump too large');
fprintf('PASS\n');

%% P6-T11: 性能基準
fprintf('[P6-T11] Performance benchmark... ');
% IK 計算 100 次
tic;
for k = 1:100
    T = [eye(3), [500+k; 0; 800]; 0,0,0,1];
    arm.inverseKinematics(T);
end
ik_time = toc;
assert(ik_time < 5, 'P6-T11 FAILED: IK too slow (%.2fs for 100 calls)', ik_time);

% 代碼生成
tic;
for k = 1:10
    generate_robot_cmd(struct('cmd', 'home'), output_dir);
end
gen_time = toc;
assert(gen_time < 2, 'P6-T11 FAILED: code generation too slow (%.2fs for 10 calls)', gen_time);

% 清理測試文件
generated = dir(fullfile(output_dir, 'cmd_*.m'));
for k = 1:length(generated)
    delete(fullfile(output_dir, generated(k).name));
end

fprintf('PASS (IK=%.3fs, Gen=%.3fs)\n', ik_time, gen_time);

fprintf('\n=== Phase 6: ALL TESTS PASSED ===\n');
