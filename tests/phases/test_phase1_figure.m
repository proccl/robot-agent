%% test_phase1_figure.m — Phase 1: 可視化獨立函數測試
%   驗證 initRobotFigure, updateRobotFigure, animateRobot

% Ensure working directory is tests/ (batch mode may put mfilename in Temp)
if ~strcmp(pwd, 'D:\Document\code\Matlab\robot-agent\tests')
    cd('D:\Document\code\Matlab\robot-agent\tests');
end

addpath(fullfile(pwd, '..', 'src'));
output_dir = fullfile(pwd, 'output');
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

fprintf('=== Phase 1: Figure Visualization Tests ===\n');

%% P1-T1: initRobotFigure 初始化
fprintf('[P1-T1] initRobotFigure initialization... ');
arm = Arm7R();
fig = initRobotFigure(arm, zeros(1,7));
assert(isvalid(fig), 'P1-T1 FAILED: fig invalid');
pause(0.5);
print(fig, fullfile(output_dir, 'p1_t1_init.png'), '-dpng');
fprintf('PASS\n');

%% P1-T2: Figure UserData 完整性
fprintf('[P1-T2] UserData completeness... ');
ud = fig.UserData;
assert(isfield(ud, 'arm'), 'P1-T2 FAILED: missing arm');
assert(isfield(ud, 'current_q'), 'P1-T2 FAILED: missing current_q');
assert(isfield(ud, 'ax'), 'P1-T2 FAILED: missing ax');
assert(isfield(ud, 'h_link'), 'P1-T2 FAILED: missing h_link');
assert(isfield(ud, 'h_joints'), 'P1-T2 FAILED: missing h_joints');
assert(isfield(ud, 'h_labels'), 'P1-T2 FAILED: missing h_labels');
assert(isfield(ud, 'h_axes_base'), 'P1-T2 FAILED: missing h_axes_base');
assert(isfield(ud, 'h_axes_ee'), 'P1-T2 FAILED: missing h_axes_ee');
assert(isequal(size(ud.current_q), [1 7]), 'P1-T2 FAILED: current_q size');
fprintf('PASS\n');

%% P1-T3: updateRobotFigure 更新
fprintf('[P1-T3] updateRobotFigure with new pose... ');
q_new = [pi/4, pi/4, 0, 0, 0, -pi/4, 0];
updateRobotFigure(fig, q_new);
pause(0.5);
print(fig, fullfile(output_dir, 'p1_t3_update.png'), '-dpng');
% 視覺檢查：與 p1_t1_init.png 對比，姿態應不同
fprintf('PASS (visual check required)\n');

%% P1-T4: 句柄有效性
fprintf('[P1-T4] Handle validity... ');
ud = fig.UserData;
assert(isvalid(fig), 'P1-T4 FAILED: fig invalid');
assert(isvalid(ud.ax), 'P1-T4 FAILED: ax invalid');
assert(isvalid(ud.h_link), 'P1-T4 FAILED: h_link invalid');
assert(isvalid(ud.h_joints), 'P1-T4 FAILED: h_joints invalid');
for i = 1:length(ud.h_labels)
    assert(isvalid(ud.h_labels(i)), 'P1-T4 FAILED: h_labels(%d) invalid', i);
end
for j = 1:3
    assert(isvalid(ud.h_axes_base(j)), 'P1-T4 FAILED: h_axes_base(%d) invalid', j);
    assert(isvalid(ud.h_axes_ee(j)), 'P1-T4 FAILED: h_axes_ee(%d) invalid', j);
end
fprintf('PASS\n');

%% P1-T5: Figure 關閉後重建
fprintf('[P1-T5] Rebuild after close... ');
close(fig);
pause(0.3);
fig2 = initRobotFigure(arm, zeros(1,7));
assert(isvalid(fig2), 'P1-T5 FAILED: rebuilt fig invalid');
pause(0.5);
print(fig2, fullfile(output_dir, 'p1_t5_rebuild.png'), '-dpng');
close(fig2);
fprintf('PASS\n');

%% P1-T6: animateRobot 播放動畫
fprintf('[P1-T6] animateRobot 30 frames... ');
fig3 = initRobotFigure(arm, zeros(1,7));
steps = 30;
q_traj = zeros(steps, 7);
q_target = [pi/4, pi/4, 0, 0, 0, -pi/4, 0];
for j = 1:7
    q_traj(:, j) = linspace(0, q_target(j), steps);
end
tic;
animateRobot(fig3, q_traj, 30);
elapsed = toc;
assert(elapsed >= 0.8 && elapsed <= 6.0, 'P1-T6 FAILED: timing off (%.2fs)', elapsed);
% 驗證末幀位姿
T_ee = arm.forwardKinematics(fig3.UserData.current_q);
T_target = arm.forwardKinematics(q_target);
assert(norm(T_ee(1:3,4) - T_target(1:3,4)) < 1, 'P1-T6 FAILED: end pose mismatch');
print(fig3, fullfile(output_dir, 'p1_t6_animate_end.png'), '-dpng');
close(fig3);
fprintf('PASS (elapsed=%.2fs)\n', elapsed);

fprintf('\n=== Phase 1: ALL TESTS PASSED ===\n');
