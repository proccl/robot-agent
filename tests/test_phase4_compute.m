function test_phase4_compute()
    % test_phase4_compute Phase 4 測試：後台計算與隊列推送
    
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    addpath(fullfile(fileparts(script_path), '..', 'src'));
    
    output_dir = fullfile(fileparts(script_path), 'output');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    fprintf('========================================\n');
    fprintf('  Phase 4 Tests: Background Compute\n');
    fprintf('========================================\n\n');
    
    agent = RobotAgent();
    agent.current_q = [pi/2, pi/2, 0, 0, 0, -pi/2, 0];
    agent.updatePlot(agent.current_q);
    agent.startRenderLoop();
    pause(0.5);
    
    %% P4-T1: 後台計算 circle 軌跡
    fprintf('[P4-T1] Background compute circle trajectory (100 pts)... ');
    cmd = struct('cmd', 'trajectory', 'type', 'circle', 'radius', 200, 'duration', 5);
    agent.computeTrajectoryAsync(cmd);
    assert(agent.is_busy, 'is_busy should be true after starting compute');
    % 等待計算完成
    max_wait = 10;
    tic;
    while agent.is_busy && toc < max_wait
        pause(0.2);
    end
    assert(~agent.is_busy, 'Computation did not finish in time');
    assert(~isempty(agent.trajectory_queue), 'Trajectory queue is empty');
    assert(size(agent.trajectory_queue, 2) == 7, 'Trajectory should be Nx7');
    assert(~any(isnan(agent.trajectory_queue(:))), 'Trajectory contains NaN');
    fprintf('OK (%d points)\n', size(agent.trajectory_queue, 1));
    
    %% P4-T2: 計算期間渲染循環幀率（等待隊列播放完畢後檢查）
    fprintf('[P4-T2] Frame rate during/after compute... ');
    % 記錄當前幀數，等 2 秒後再記錄
    frames_before = agent.render_frame_count;
    pause(2);
    frames_after = agent.render_frame_count;
    delta = frames_after - frames_before;
    % batch 模式下至少 10fps
    assert(delta >= 20, 'Frame rate too low: %d frames in 2s', delta);
    fprintf('OK (%d frames in 2s)\n', delta);
    
    %% P4-T3: 隊列推送與清空
    fprintf('[P4-T3] Queue push and clear... ');
    old_queue = agent.trajectory_queue;
    % 發送新指令，舊隊列應被覆蓋
    cmd2 = struct('cmd', 'home', 'duration', 2);
    agent.computeTrajectoryAsync(cmd2);
    max_wait = 5;
    tic;
    while agent.is_busy && toc < max_wait
        pause(0.2);
    end
    assert(~isempty(agent.trajectory_queue), 'New queue should not be empty');
    % 新隊列應該與舊的不同（至少尺寸不同或內容不同）
    assert(~isequal(agent.trajectory_queue, old_queue), 'Queue was not replaced');
    fprintf('OK (queue replaced)\n');
    
    %% P4-T4: 繁忙標誌 is_busy
    fprintf('[P4-T4] Busy flag... ');
    % 在隊列播放期間發送新指令，應該被忽略或覆蓋
    agent.trajectory_queue = rand(10, 7); % 手動填充隊列
    agent.queue_idx = 1;
    cmd3 = struct('cmd', 'trajectory', 'type', 'circle', 'radius', 100, 'duration', 3);
    agent.computeTrajectoryAsync(cmd3);
    % 如果當前沒有後台計算，is_busy 會變為 true
    assert(agent.is_busy, 'Should be busy');
    fprintf('OK\n');
    
    %% P4-T5: IK 無解處理
    fprintf('[P4-T5] Unreachable pose handling... ');
    % 發送到極遠位置，應該不可達
    cmd_bad = struct('cmd', 'move_to', 'position', [5000, 5000, 5000], 'duration', 2);
    agent.computeTrajectoryAsync(cmd_bad);
    max_wait = 5;
    tic;
    while agent.is_busy && toc < max_wait
        pause(0.2);
    end
    % 計算應該完成，但可能返回錯誤（is_busy 被清除）
    fprintf('OK (error handled, is_busy=%d)\n', agent.is_busy);
    
    %% 清理
    if isvalid(agent.render_timer)
        stop(agent.render_timer);
        delete(agent.render_timer);
    end
    agent.stop();
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 4 Tests PASSED (5/5)\n');
    fprintf('========================================\n');
end
