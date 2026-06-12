function test_obstacle_visualization()
% test_obstacle_visualization 驗證 Figure 中障礙物小球的存在與可見性切換

    fprintf('[Test] obstacle visualization ...\n');
    
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'src'));
    
    arm = Arm7R();
    obstacle = struct('center', [800; 0; 0], 'radius', 100, 'enabled', true);
    fig = initRobotFigure(arm, zeros(1, 7), obstacle);
    
    assert(isfield(fig.UserData, 'h_obstacle'), 'h_obstacle handle missing');
    assert(isvalid(fig.UserData.h_obstacle), 'h_obstacle is not valid');
    assert(strcmp(fig.UserData.h_obstacle.Visible, 'on'), 'Obstacle should be visible when enabled');
    
    % 切換可見性
    fig.UserData.obstacle_enabled = false;
    updateRobotFigure(fig, zeros(1, 7));
    assert(strcmp(fig.UserData.h_obstacle.Visible, 'off'), 'Obstacle should be hidden when disabled');
    
    close(fig);
    fprintf('[PASS] obstacle visualization\n');
end
