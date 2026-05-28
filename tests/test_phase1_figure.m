function test_phase1_figure()
    % test_phase1_figure Phase 1 測試：Figure 初始化與句柄
    %   測試內容：
    %     P1-T1: 默認姿態 Figure 初始化（視覺測試）
    %     P1-T2: 不同關節角渲染（視覺測試）
    %     P1-T3: 句柄有效性檢查
    %     P1-T4: Figure 關閉後重建（視覺測試）
    
    % 添加 src 到 path
    script_path = mfilename('fullpath');
    if isempty(script_path)
        script_path = pwd;
    end
    src_dir = fullfile(fileparts(script_path), '..', 'src');
    addpath(src_dir);
    
    output_dir = fullfile(fileparts(script_path), 'output');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    fprintf('========================================\n');
    fprintf('  Phase 1 Tests: Figure Initialization\n');
    fprintf('========================================\n\n');
    
    %% P1-T1: 默認姿態 Figure 初始化
    fprintf('[P1-T1] Default pose initialization... ');
    agent = RobotAgent();
    pause(0.5); % 讓 Figure 完全渲染
    png_path = fullfile(output_dir, 'test_phase1_t1_default.png');
    print(agent.fig, '-dpng', png_path);
    fprintf('OK\n');
    fprintf('        PNG saved: %s\n', png_path);
    
    %% P1-T2: 不同關節角渲染
    fprintf('[P1-T2] Different joint angles (pi/4, pi/4, 0, 0, 0, -pi/4, 0)... ');
    agent.current_q = [pi/4, pi/4, 0, 0, 0, -pi/4, 0];
    points = agent.arm.getJointPositions(agent.current_q);
    set(agent.h_link, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
    set(agent.h_joints, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
    for i = 1:length(agent.h_labels)
        if isvalid(agent.h_labels(i))
            set(agent.h_labels(i), 'Position', points(i,:));
        end
    end
    drawnow;
    pause(0.5);
    png_path = fullfile(output_dir, 'test_phase1_t2_pose.png');
    print(agent.fig, '-dpng', png_path);
    fprintf('OK\n');
    fprintf('        PNG saved: %s\n', png_path);
    
    %% P1-T3: 句柄有效性檢查
    fprintf('[P1-T3] Handle validity check... ');
    assert(isvalid(agent.h_link), 'h_link is invalid');
    assert(isvalid(agent.h_joints), 'h_joints is invalid');
    for i = 1:length(agent.h_labels)
        assert(isvalid(agent.h_labels(i)), 'h_labels(%d) is invalid', i);
    end
    for j = 1:3
        assert(isvalid(agent.h_axes_base(j)), 'h_axes_base(%d) is invalid', j);
        assert(isvalid(agent.h_axes_ee(j)), 'h_axes_ee(%d) is invalid', j);
    end
    fprintf('OK (all handles valid)\n');
    
    %% P1-T4: Figure 關閉後重建
    fprintf('[P1-T4] Rebuild after figure close... ');
    close(agent.fig);
    pause(0.3);
    agent.current_q = zeros(1, 7); % 重置為零位，確保與 T1 一致
    agent.initFigure();
    pause(0.5);
    png_path = fullfile(output_dir, 'test_phase1_t4_rebuild.png');
    print(agent.fig, '-dpng', png_path);
    fprintf('OK\n');
    fprintf('        PNG saved: %s\n', png_path);
    
    fprintf('\n========================================\n');
    fprintf('  All Phase 1 Tests PASSED (4/4)\n');
    fprintf('========================================\n');
    
    % 保存 agent 到 base workspace 供後續檢查
    assignin('base', 'phase1_agent', agent);
end
