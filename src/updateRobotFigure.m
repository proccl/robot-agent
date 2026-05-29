function updateRobotFigure(fig, q)
% updateRobotFigure 高效更新機械臂 Figure
%   fig: figure 句柄（由 initRobotFigure 創建）
%   q:   關節角 (1x7)

    if ~isvalid(fig)
        error('Figure is no longer valid.');
    end
    
    ud = fig.UserData;
    if ~isfield(ud, 'ax') || ~isvalid(ud.ax)
        error('Axes not found in figure. Re-run initRobotFigure.');
    end
    
    ax = ud.ax;
    arm = ud.arm;
    
    points = arm.getJointPositions(q);
    
    % 更新連桿
    if isvalid(ud.h_link)
        set(ud.h_link, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
    end
    
    % 更新關節點
    if isvalid(ud.h_joints)
        set(ud.h_joints, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
    end
    
    % 更新標註位置
    labels = {'Base', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'EE'};
    for i = 1:length(labels)
        if isvalid(ud.h_labels(i))
            set(ud.h_labels(i), 'Position', points(i,:));
        end
    end
    
    % 更新末端坐標軸
    T_ee = arm.forwardKinematics(q);
    p_ee = T_ee(1:3, 4);
    axis_len_ee = 80;
    colors = {'r', 'g', 'b'};
    for j = 1:3
        if isvalid(ud.h_axes_ee(j))
            dir = T_ee(1:3, j);
            set(ud.h_axes_ee(j), ...
                'XData', [p_ee(1), p_ee(1)+axis_len_ee*dir(1)], ...
                'YData', [p_ee(2), p_ee(2)+axis_len_ee*dir(2)], ...
                'ZData', [p_ee(3), p_ee(3)+axis_len_ee*dir(3)], ...
                'Color', colors{j});
        end
    end
    
    drawnow limitrate;
end
