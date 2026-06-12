%% test_phase2_filewatch.m — Phase 2: 文件監聽測試
%   驗證 robotagent 啟動、timer 監聽、文件執行與錯誤處理

% Ensure working directory is tests/ (batch mode may put mfilename in Temp)
if ~strcmp(pwd, 'D:\Document\code\Matlab\robot-agent\tests')
    cd('D:\Document\code\Matlab\robot-agent\tests');
end

addpath(fullfile(pwd, '..', 'src'));
output_dir = fullfile(pwd, 'output');
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

fprintf('=== Phase 2: File Watch Tests ===\n');

%% P2-T2: incoming/ 目錄自動創建（先驗證目錄邏輯）
fprintf('[P2-T2] incoming directory auto-create... ');
test_dir = fullfile(pwd, 'test_incoming');
if exist(test_dir, 'dir'), rmdir(test_dir, 's'); end
assert(~exist(test_dir, 'dir'), 'P2-T2 FAILED: test_dir exists before test');
mkdir(test_dir);
assert(exist(test_dir, 'dir'), 'P2-T2 FAILED: mkdir failed');
rmdir(test_dir);
fprintf('PASS\n');

%% P2-T3: 手動寫入測試 .m 文件並由 processIncomingCommands 執行
fprintf('[P2-T3] processIncomingCommands executes file... ');
arm = Arm7R();
fig = initRobotFigure(arm, zeros(1,7));
fig.UserData.is_busy = false;
test_incoming = fullfile(pwd, 'test_incoming_p2t3');
if exist(test_incoming, 'dir'), rmdir(test_incoming, 's'); end
mkdir(test_incoming);

test_file = fullfile(test_incoming, 'cmd_test_disp.m');
fid = fopen(test_file, 'w');
fprintf(fid, 'disp(''P2T3_HELLO'');\n');
fclose(fid);

processIncomingCommands(test_incoming, fig);
pause(0.3);
assert(~exist(test_file, 'file'), 'P2-T3 FAILED: test file not deleted');
close(fig);
rmdir(test_incoming, 's');
fprintf('PASS\n');

%% P2-T4: 多個文件按時間順序執行
fprintf('[P2-T4] Multiple files execution order... ');
fig2 = initRobotFigure(arm, zeros(1,7));
fig2.UserData.is_busy = false;
test_incoming2 = fullfile(pwd, 'test_incoming_p2t4');
if exist(test_incoming2, 'dir'), rmdir(test_incoming2, 's'); end
mkdir(test_incoming2);

% 寫入兩個文件，B 先於 A（但 A 的 datenum 應該更小？不，按 datenum 排序）
% 為了確保順序，使用 pause 製造時間差
fA = fullfile(test_incoming2, 'cmd_A_second.m');
fid = fopen(fA, 'w'); fprintf(fid, 'global p2t4_flag; p2t4_flag = 1;\n'); fclose(fid);
pause(0.1);
fB = fullfile(test_incoming2, 'cmd_B_first.m');
fid = fopen(fB, 'w'); fprintf(fid, 'global p2t4_flag; p2t4_flag = 2;\n'); fclose(fid);

global p2t4_flag;
p2t4_flag = 0;
processIncomingCommands(test_incoming2, fig2);
% 第一次執行應該是 A（因為 A 先創建，datenum 更小）
assert(p2t4_flag == 1, 'P2-T4 FAILED: first execution wrong (flag=%d)', p2t4_flag);

% B 應該還在，再次執行
processIncomingCommands(test_incoming2, fig2);
assert(p2t4_flag == 2, 'P2-T4 FAILED: second execution wrong (flag=%d)', p2t4_flag);

close(fig2);
rmdir(test_incoming2, 's');
clear global p2t4_flag;
fprintf('PASS\n');

%% P2-T5: 錯誤腳本處理
fprintf('[P2-T5] Error script handling... ');
fig3 = initRobotFigure(arm, zeros(1,7));
fig3.UserData.is_busy = false;
test_incoming3 = fullfile(pwd, 'test_incoming_p2t5');
if exist(test_incoming3, 'dir'), rmdir(test_incoming3, 's'); end
mkdir(test_incoming3);

f_err = fullfile(test_incoming3, 'cmd_error.m');
fid = fopen(f_err, 'w'); fprintf(fid, 'error(''P2T5_INTENTIONAL_ERROR'');\n'); fclose(fid);

processIncomingCommands(test_incoming3, fig3);
pause(0.3);
failed_dir = fullfile(test_incoming3, 'failed');
assert(exist(failed_dir, 'dir'), 'P2-T5 FAILED: failed dir not created');
assert(exist(fullfile(failed_dir, 'cmd_error.m'), 'file'), 'P2-T5 FAILED: error file not moved');
close(fig3);
rmdir(test_incoming3, 's');
fprintf('PASS\n');

%% P2-T6: 執行期間 is_busy 阻止並發
fprintf('[P2-T6] is_busy concurrency guard... ');
fig4 = initRobotFigure(arm, zeros(1,7));
fig4.UserData.is_busy = true;  % 模擬忙碌狀態
test_incoming4 = fullfile(pwd, 'test_incoming_p2t6');
if exist(test_incoming4, 'dir'), rmdir(test_incoming4, 's'); end
mkdir(test_incoming4);

f_busy = fullfile(test_incoming4, 'cmd_busy.m');
fid = fopen(f_busy, 'w'); fprintf(fid, 'disp(''SHOULD_NOT_RUN'');\n'); fclose(fid);

processIncomingCommands(test_incoming4, fig4);  % 應該因 is_busy 直接返回
assert(exist(f_busy, 'file'), 'P2-T6 FAILED: file executed while busy');

fig4.UserData.is_busy = false;
close(fig4);
rmdir(test_incoming4, 's');
fprintf('PASS\n');

%% P2-T7: incoming/ 為空時無異常
fprintf('[P2-T7] Empty incoming no error... ');
fig5 = initRobotFigure(arm, zeros(1,7));
fig5.UserData.is_busy = false;
test_incoming5 = fullfile(pwd, 'test_incoming_p2t7');
if exist(test_incoming5, 'dir'), rmdir(test_incoming5, 's'); end
mkdir(test_incoming5);

for k = 1:5
    processIncomingCommands(test_incoming5, fig5);
    pause(0.1);
end
close(fig5);
rmdir(test_incoming5, 's');
fprintf('PASS\n');

fprintf('\n=== Phase 2: ALL TESTS PASSED ===\n');
