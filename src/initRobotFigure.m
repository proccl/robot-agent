function fig = initRobotFigure(arm, q)
% initRobotFigure 初始化機械臂 Figure
%   arm: Arm7R 對象
%   q:   初始關節角 (1x7)，默認 zeros(1,7)
%   fig: 返回 figure 句柄，圖形對象句柄存儲在 fig.UserData

    if nargin < 2 || isempty(q)
        q = zeros(1, 7);
    end
    
    fig = figure('Color', 'white', ...
                 'Name', 'RobotAgent - 7R Arm', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 900, 700]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    
    % 計算初始位置
    points = arm.getJointPositions(q);
    
    % 繪製連桿
    h_link = plot3(ax, points(:,1), points(:,2), points(:,3), ...
                   'b-o', 'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    % 繪製關節點
    h_joints = scatter3(ax, points(:,1), points(:,2), points(:,3), ...
                        100, 'b', 'filled');
    
    % 關節標註
    labels = {'Base', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'EE'};
    h_labels = gobjects(length(labels), 1);
    for i = 1:length(labels)
        h_labels(i) = text(ax, points(i,1), points(i,2), points(i,3), ...
                           sprintf('  %s', labels{i}), ...
                           'FontSize', 9, 'FontWeight', 'bold');
    end
    
    % Base 坐標軸
    h_axes_base = gobjects(3, 1);
    colors = {'r', 'g', 'b'};
    axis_len_base = 50;
    for j = 1:3
        dir = eye(3);
        h_axes_base(j) = plot3(ax, ...
            [0, axis_len_base*dir(1,j)], ...
            [0, axis_len_base*dir(2,j)], ...
            [0, axis_len_base*dir(3,j)], ...
            colors{j}, 'LineWidth', 1.5);
    end
    
    % 末端坐標軸
    h_axes_ee = gobjects(3, 1);
    axis_len_ee = 80;
    for j = 1:3
        h_axes_ee(j) = plot3(ax, [0, 0], [0, 0], [0, 0], ...
                             colors{j}, 'LineWidth', 2);
    end
    
    % 坐標軸設置
    xlabel(ax, 'X (mm)', 'FontSize', 12);
    ylabel(ax, 'Y (mm)', 'FontSize', 12);
    zlabel(ax, 'Z (mm)', 'FontSize', 12);
    
    axis(ax, [-1500, 1500, -1500, 1500, -500, 1500]);
    view(ax, 45, 25);
    camproj('perspective');
    rotate3d(ax, 'on');
    
    title(ax, 'RobotAgent - 7R Manipulator', 'FontSize', 14);
    legend(ax, 'Links', 'Location', 'best');
    
    hold(ax, 'off');
    
    % 存儲所有句柄到 UserData
    fig.UserData = struct();
    fig.UserData.ax = ax;
    fig.UserData.h_link = h_link;
    fig.UserData.h_joints = h_joints;
    fig.UserData.h_labels = h_labels;
    fig.UserData.h_axes_base = h_axes_base;
    fig.UserData.h_axes_ee = h_axes_ee;
    fig.UserData.arm = arm;
    fig.UserData.current_q = q;
    
    % 初始化時更新一次，確保 EE 坐標軸正確定位
    updateRobotFigure(fig, q);
end
