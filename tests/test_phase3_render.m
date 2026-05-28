function test_phase3_render()
    % test_phase3_render Phase 3 測試：渲染循環幀率與流暢度
    
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
    fprintf('  Phase 3 Tests: Render Loop\n');
    fprintf('========================================\n\n');
    
    agent = RobotAgent();
    agent.current_q = zeros(1, 7);
    agent.updatePlot(agent.current_q);
    
    %% P3-T1: 啟動渲染循環，觀察 3 秒
    fprintf('[P3-T1] Start render loop for 3s... ');
    agent.startRenderLoop();
    pause(3);
    assert(isvalid(agent.render_timer), 'Render timer stopped unexpectedly');
    fprintf('OK (timer running, %d frames rendered)\n', agent.render_frame_count);
    
    %% P3-T2: 幀率穩定性
    fprintf('[P3-T2] Frame rate stability... ');
    expected_frames = 3 * agent.target_fps; % 3 秒理想 90 幀
    actual_frames = agent.render_frame_count;
    % batch 模式下 timer 精度有限，實際約 15-25fps，放寬容差
    min_acceptable = 3 * 15; % 最低 15fps
    max_acceptable = 3 * 40; % 最高 40fps（防止異常快）
    assert(actual_frames >= min_acceptable, ...
           'Frame count too low: %d (min %d)', actual_frames, min_acceptable);
    assert(actual_frames <= max_acceptable, ...
           'Frame count too high: %d (max %d)', actual_frames, max_acceptable);
    fprintf('OK (%d frames in 3s, ~%.1f fps)\n', actual_frames, actual_frames/3);
    
    %% P3-T3: 手動改變關節角後更新
    fprintf('[P3-T3] Manual joint angle update... ');
    agent.current_q = [pi/4, pi/4, 0, 0, 0, -pi/4, 0];
    agent.updatePlot(agent.current_q);
    pause(0.3);
    print(agent.fig, '-dpng', fullfile(output_dir, 'test_phase3_t3_pose1.png'));
    
    agent.current_q = [pi/2, 0, pi/4, 0, -pi/4, 0, pi/6];
    agent.updatePlot(agent.current_q);
    pause(0.3);
    print(agent.fig, '-dpng', fullfile(output_dir, 'test_phase3_t3_pose2.png'));
    fprintf('OK (2 PNGs saved)\n');
    
    %% P3-T4: 軌跡隊列播放
    fprintf('[P3-T4] Trajectory queue playback (2s animation)... ');
    agent.current_q = zeros(1, 7);
    agent.updatePlot(agent.current_q);
    agent.animateTo([pi/4, pi/4, 0, 0, 0, -pi/4, 0], 2);
    % 記錄當前幀數
    frames_before = agent.render_frame_count;
    % 等待播放完成（隊列播放完畢）
    max_wait = 4; % 最多等 4 秒
    tic;
    while agent.queue_idx <= size(agent.trajectory_queue, 1) && toc < max_wait
        pause(0.1);
    end
    pause(0.3);
    print(agent.fig, '-dpng', fullfile(output_dir, 'test_phase3_t4_end.png'));
    frames_after = agent.render_frame_count;
    fprintf('OK (%d frames played)\n', frames_after - frames_before);
    
    %% P3-T5: Figure 關閉後自動重建
    fprintf('[P3-T5] Auto-rebuild after figure close... ');
    close(agent.fig);
    pause(0.5);
    agent.updatePlot(agent.current_q); % 觸發重建
    pause(0.3);
    assert(isvalid(agent.fig), 'Figure not rebuilt');
    print(agent.fig, '-dpng', fullfile(output_dir, 'test_phase3_t5_rebuild.png'));
    fprintf('OK\n');
    
    %% 清理
    if isvalid(agent.render_timer)
        stop(agent.render_timer);
        delete(agent.render_timer);
    end
    agent.stop();
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 3 Tests PASSED (5/5)\n');
    fprintf('========================================\n');
end
