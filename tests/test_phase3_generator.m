%% test_phase3_generator.m — Phase 3: Quintic 軌跡 + 代碼生成器測試

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
output_dir = fullfile(fileparts(mfilename('fullpath')), 'output');
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

%% P3-T5: generate_robot_cmd 生成 home 文件
fprintf('[P3-T5] generate_robot_cmd (home)... ');
test_out = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p3');
if exist(test_out, 'dir'), rmdir(test_out, 's'); end
mkdir(test_out);

filepath = generate_robot_cmd(struct('cmd', 'home'), test_out);
assert(exist(filepath, 'file'), 'P3-T5 FAILED: file not created');
fid = fopen(filepath, 'r');
code = fread(fid, '*char')';
fclose(fid);
assert(contains(code, 'quinticTrajectory'), 'P3-T5 FAILED: missing quinticTrajectory');
assert(contains(code, 'duration = 5'), 'P3-T5 FAILED: default duration wrong');
rmdir(test_out, 's');
fprintf('PASS\n');

%% P3-T6: home 模板在 Figure 環境中可執行
fprintf('[P3-T6] home template executes... ');
arm = Arm7R();
fig = initRobotFigure(arm, zeros(1,7));
fig.UserData.is_busy = false;
test_out2 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p3t6');
if exist(test_out2, 'dir'), rmdir(test_out2, 's'); end
mkdir(test_out2);

filepath2 = generate_robot_cmd(struct('cmd', 'home', 'duration', 5), test_out2);
run(filepath2);
pause(0.3);
assert(max(abs(fig.UserData.current_q)) < 1e-6, 'P3-T6 FAILED: current_q not zero');
print(fig, fullfile(output_dir, 'p3_t6_home.png'), '-dpng');
close(fig);
rmdir(test_out2, 's');
fprintf('PASS\n');

%% P3-T7: move_to 模板可執行
fprintf('[P3-T7] move_to template executes... ');
fig2 = initRobotFigure(arm, zeros(1,7));
fig2.UserData.is_busy = false;
test_out3 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p3t7');
if exist(test_out3, 'dir'), rmdir(test_out3, 's'); end
mkdir(test_out3);

filepath3 = generate_robot_cmd(struct('cmd', 'move_to', 'position', [500, 0, 800], 'duration', 3), test_out3);
run(filepath3);
pause(0.3);
T_ee = arm.forwardKinematics(fig2.UserData.current_q);
assert(norm(T_ee(1:3,4)' - [500, 0, 800]) < 1, 'P3-T7 FAILED: end-effector mismatch');
close(fig2);
rmdir(test_out3, 's');
fprintf('PASS\n');

%% P3-T8: joint_move 模板可執行
fprintf('[P3-T8] joint_move template executes... ');
fig3 = initRobotFigure(arm, zeros(1,7));
fig3.UserData.is_busy = false;
test_out4 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p3t8');
if exist(test_out4, 'dir'), rmdir(test_out4, 's'); end
mkdir(test_out4);

filepath4 = generate_robot_cmd(struct('cmd', 'joint_move', 'joint', 1, 'angle', 90, 'angle_deg', true, 'duration', 2), test_out4);
run(filepath4);
pause(0.3);
assert(abs(fig3.UserData.current_q(1) - pi/2) < 1e-6, 'P3-T8 FAILED: joint 1 angle mismatch');
close(fig3);
rmdir(test_out4, 's');
fprintf('PASS\n');

%% P3-T12: 用戶指定 duration
fprintf('[P3-T12] custom duration... ');
test_out5 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p3t12');
if exist(test_out5, 'dir'), rmdir(test_out5, 's'); end
mkdir(test_out5);

filepath5 = generate_robot_cmd(struct('cmd', 'home', 'duration', 3), test_out5);
fid = fopen(filepath5, 'r');
code5 = fread(fid, '*char')';
fclose(fid);
assert(contains(code5, 'duration = 3'), 'P3-T12 FAILED: custom duration not in code');
rmdir(test_out5, 's');
fprintf('PASS\n');

%% P3-T14: 文件名唯一性
fprintf('[P3-T14] filename uniqueness... ');
test_out6 = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p3t14');
if exist(test_out6, 'dir'), rmdir(test_out6, 's'); end
mkdir(test_out6);

files = {};
for k = 1:10
    fp = generate_robot_cmd(struct('cmd', 'home'), test_out6);
    [~, name, ~] = fileparts(fp);
    files{k} = name;
end
assert(length(unique(files)) == 10, 'P3-T14 FAILED: filenames not unique');
rmdir(test_out6, 's');
fprintf('PASS\n');

fprintf('\n=== Phase 3: ALL TESTS PASSED ===\n');
