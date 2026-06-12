% run_all_tests.m — 運行全部 6 個測試腳本並報告結果
% Ensure working directory is tests/ (batch mode may put mfilename in Temp)
if ~strcmp(pwd, 'D:\Document\code\Matlab\robot-agent\tests')
    cd('D:\Document\code\Matlab\robot-agent\tests');
end

addpath(fullfile(pwd, '..', 'src'));
addpath(fullfile(pwd));

results = struct();
test_names = {
    'test_phase1_figure'
    'test_phase2_filewatch'
    'test_phase3_generator'
    'test_phase4_cleanup'
    'test_phase5_e2e'
    'test_phase6_complex'
};

fprintf('\n========================================\n');
fprintf('  Running All Tests (v0.0.5 refactor)\n');
fprintf('========================================\n\n');

for i = 1:length(test_names)
    name = test_names{i};
    fprintf('[TEST %d/%d] %s ...\n', i, length(test_names), name);
    try
        eval(name);
        results.(name) = 'PASS';
        fprintf('  => PASS\n\n');
    catch ME
        results.(name) = sprintf('FAIL: %s', ME.message);
        fprintf('  => FAIL: %s\n\n', ME.message);
    end
end

fprintf('========================================\n');
fprintf('  Test Summary\n');
fprintf('========================================\n');
pass_count = 0;
fail_count = 0;
for i = 1:length(test_names)
    name = test_names{i};
    if strcmp(results.(name), 'PASS')
        fprintf('  [PASS] %s\n', name);
        pass_count = pass_count + 1;
    else
        fprintf('  [FAIL] %s: %s\n', name, results.(name));
        fail_count = fail_count + 1;
    end
end
fprintf('\nTotal: %d PASS, %d FAIL\n', pass_count, fail_count);
fprintf('========================================\n');
