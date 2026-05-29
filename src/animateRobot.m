function animateRobot(fig, q_traj, fps)
% animateRobot 播放機械臂動畫
%   fig:    figure 句柄
%   q_traj: Nx7 關節角軌跡
%   fps:    幀率（默認 30）

    if nargin < 3 || isempty(fps)
        fps = 30;
    end
    
    if ~isvalid(fig)
        error('Figure is no longer valid.');
    end
    
    for i = 1:size(q_traj, 1)
        updateRobotFigure(fig, q_traj(i, :));
        drawnow limitrate;
        pause(1/fps);
    end
    
    % 更新最終狀態
    fig.UserData.current_q = q_traj(end, :);
end
