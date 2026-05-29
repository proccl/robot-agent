%% test_phase4_nlp.m — Phase 4: 自然語言解析測試

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

fprintf('=== Phase 4: Natural Language Parsing Tests ===\n');

%% P4-T1: "home"
fprintf('[P4-T1] home... ');
cmd = parseNaturalLanguage('home');
assert(strcmp(cmd.cmd, 'home'), 'P4-T1 FAILED: cmd mismatch');
assert(cmd.duration == 5, 'P4-T1 FAILED: duration mismatch');
fprintf('PASS\n');

%% P4-T2: "回到原位"
fprintf('[P4-T2] 回到原位... ');
cmd = parseNaturalLanguage('回到原位');
assert(strcmp(cmd.cmd, 'home'), 'P4-T2 FAILED: cmd mismatch');
assert(cmd.duration == 5, 'P4-T2 FAILED: duration mismatch');
fprintf('PASS\n');

%% P4-T3: "走到 500 0 800"
fprintf('[P4-T3] 走到 500 0 800... ');
cmd = parseNaturalLanguage('走到 500 0 800');
assert(strcmp(cmd.cmd, 'move_to'), 'P4-T3 FAILED: cmd mismatch');
assert(isequal(cmd.position, [500, 0, 800]), 'P4-T3 FAILED: position mismatch');
assert(cmd.duration == 5, 'P4-T3 FAILED: duration mismatch');
fprintf('PASS\n');

%% P4-T4: "move to (500, 0, 800)"
fprintf('[P4-T4] move to (500, 0, 800)... ');
cmd = parseNaturalLanguage('move to (500, 0, 800)');
assert(strcmp(cmd.cmd, 'move_to'), 'P4-T4 FAILED: cmd mismatch');
assert(isequal(cmd.position, [500, 0, 800]), 'P4-T4 FAILED: position mismatch');
fprintf('PASS\n');

%% P4-T5: "走到 500 0 800 用 3 秒"
fprintf('[P4-T5] 走到 500 0 800 用 3 秒... ');
cmd = parseNaturalLanguage('走到 500 0 800 用 3 秒');
assert(strcmp(cmd.cmd, 'move_to'), 'P4-T5 FAILED: cmd mismatch');
assert(isequal(cmd.position, [500, 0, 800]), 'P4-T5 FAILED: position mismatch');
assert(cmd.duration == 3, 'P4-T5 FAILED: duration mismatch');
fprintf('PASS\n');

%% P4-T6: "關節1轉90度"
fprintf('[P4-T6] 關節1轉90度... ');
cmd = parseNaturalLanguage('關節1轉90度');
assert(strcmp(cmd.cmd, 'joint_move'), 'P4-T6 FAILED: cmd mismatch');
assert(cmd.joint == 1, 'P4-T6 FAILED: joint mismatch');
assert(cmd.angle == 90, 'P4-T6 FAILED: angle mismatch');
assert(cmd.angle_deg == true, 'P4-T6 FAILED: angle_deg mismatch');
assert(cmd.duration == 5, 'P4-T6 FAILED: duration mismatch');
fprintf('PASS\n');

%% P4-T7: "joint 2 -45"
fprintf('[P4-T7] joint 2 -45... ');
cmd = parseNaturalLanguage('joint 2 -45');
assert(strcmp(cmd.cmd, 'joint_move'), 'P4-T7 FAILED: cmd mismatch');
assert(cmd.joint == 2, 'P4-T7 FAILED: joint mismatch');
assert(cmd.angle == -45, 'P4-T7 FAILED: angle mismatch');
assert(cmd.angle_deg == true, 'P4-T7 FAILED: angle_deg mismatch');
fprintf('PASS\n');

%% P4-T8: "畫圓 半徑 200 用 5 秒"
fprintf('[P4-T8] 畫圓 半徑 200 用 5 秒... ');
cmd = parseNaturalLanguage('畫圓 半徑 200 用 5 秒');
assert(strcmp(cmd.cmd, 'trajectory'), 'P4-T8 FAILED: cmd mismatch');
assert(strcmp(cmd.type, 'circle'), 'P4-T8 FAILED: type mismatch');
assert(cmd.radius == 200, 'P4-T8 FAILED: radius mismatch');
assert(cmd.duration == 5, 'P4-T8 FAILED: duration mismatch');
fprintf('PASS\n');

%% P4-T9: 無法識別的輸入
fprintf('[P4-T9] unrecognized input... ');
try
    cmd = parseNaturalLanguage('xyz abc 123');
    error('P4-T9 FAILED: should have thrown error');
catch ME
    assert(contains(ME.message, '無法識別'), 'P4-T9 FAILED: wrong error message');
end
fprintf('PASS\n');

%% P4-T10: 端到端（自然語言 → 生成文件 → 執行）
fprintf('[P4-T10] end-to-end: NLP → file → execution... ');
arm = Arm7R();
fig = initRobotFigure(arm, zeros(1,7));
fig.UserData.is_busy = false;

test_out = fullfile(fileparts(mfilename('fullpath')), 'test_incoming_p4t10');
if exist(test_out, 'dir'), rmdir(test_out, 's'); end
mkdir(test_out);

cmd_struct = parseNaturalLanguage('joint 3 45 deg');
filepath = generate_robot_cmd(cmd_struct, test_out);
run(filepath);
pause(0.3);

assert(abs(fig.UserData.current_q(3) - deg2rad(45)) < 1e-6, 'P4-T10 FAILED: pose mismatch');

close(fig);
rmdir(test_out, 's');
fprintf('PASS\n');

fprintf('\n=== Phase 4: ALL TESTS PASSED ===\n');
