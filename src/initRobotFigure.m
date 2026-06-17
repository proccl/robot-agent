function fig = initRobotFigure(arm, q, obstacle)
% initRobotFigure 初始化機械臂 Figure
%   arm:      Arm7R 對象
%   q:        初始關節角 (1x7)，默認 zeros(1,7)
%   obstacle: 障礙物結構體，含 center (1x3 或 3x1)、radius、
%             safety_margin、enabled；可選，默認位於 [800,0,0] 的紅球
%   fig:      返回 figure 句柄，圖形對象句柄存儲在 fig.UserData

    if nargin < 2 || isempty(q)
        q = zeros(1, 7);
    end
    if nargin < 3 || isempty(obstacle)
        obstacle = struct('center', [800; 0; 0], ...
                          'radius', 100, ...
                          'safety_margin', 50, ...
                          'enabled', true);
    end
    if isrow(obstacle.center)
        obstacle.center = obstacle.center(:);
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
    
    % 繪製障礙物（紅色半透明球體）
    [X, Y, Z] = sphere(20);
    X = obstacle.center(1) + obstacle.radius * X;
    Y = obstacle.center(2) + obstacle.radius * Y;
    Z = obstacle.center(3) + obstacle.radius * Z;
    h_obstacle = surf(ax, X, Y, Z, ...
                      'FaceColor', 'r', ...
                      'FaceAlpha', 0.45, ...
                      'EdgeColor', 'none', ...
                      'FaceLighting', 'gouraud', ...
                      'AmbientStrength', 0.4, ...
                      'DiffuseStrength', 0.6, ...
                      'SpecularStrength', 0.5, ...
                      'SpecularExponent', 15, ...
                      'Visible', bool2visible(obstacle.enabled));
    
    % 添加光源與光照模型
    h_light = light(ax, 'Position', [1 0 1], 'Style', 'infinite');
    lighting(ax, 'gouraud');
    
    % 添加地面（淺灰色半透明平面，作為陰影接收面）
    groundSize = 3000;
    h_ground = patch(ax, ...
        [-groundSize, groundSize, groundSize, -groundSize], ...
        [-groundSize, -groundSize, groundSize, groundSize], ...
        [0, 0, 0, 0], ...
        'FaceColor', [0.9, 0.9, 0.9], ...
        'FaceAlpha', 0.15, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'none');
    
    % 添加球的投影陰影（地面上的深色橢圓）
    theta_shadow = linspace(0, 2*pi, 50);
    shadow_x = obstacle.center(1) + obstacle.radius * cos(theta_shadow);
    shadow_y = obstacle.center(2) + obstacle.radius * sin(theta_shadow);
    shadow_z = zeros(size(theta_shadow));
    h_shadow = patch(ax, shadow_x, shadow_y, shadow_z, ...
                     'FaceColor', 'k', ...
                     'FaceAlpha', 0.25, ...
                     'EdgeColor', 'none', ...
                     'FaceLighting', 'none', ...
                     'Visible', bool2visible(obstacle.enabled));
    
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
    
    % 末端位姿矩陣顯示（左上角，透明背景）
    T_init = arm.forwardKinematics(q);
    pose_str = sprintf(['T = [%8.3f %8.3f %8.3f %10.3f\n' ...
                        '     %8.3f %8.3f %8.3f %10.3f\n' ...
                        '     %8.3f %8.3f %8.3f %10.3f\n' ...
                        '     %8.3f %8.3f %8.3f %10.3f]'], ...
        T_init(1,1), T_init(1,2), T_init(1,3), T_init(1,4), ...
        T_init(2,1), T_init(2,2), T_init(2,3), T_init(2,4), ...
        T_init(3,1), T_init(3,2), T_init(3,3), T_init(3,4), ...
        T_init(4,1), T_init(4,2), T_init(4,3), T_init(4,4));
    h_pose_text = annotation('textbox', ...
        'Units', 'normalized', ...
        'Position', [0.02, 0.82, 0.38, 0.15], ...
        'FontName', 'Consolas', ...
        'FontSize', 9, ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'top', ...
        'Color', 'black', ...
        'BackgroundColor', 'none', ...
        'EdgeColor', 'none', ...
        'String', pose_str, ...
        'FitBoxToText', 'off');
    
    % 存儲所有句柄到 UserData
    fig.UserData = struct();
    fig.UserData.ax = ax;
    fig.UserData.h_link = h_link;
    fig.UserData.h_joints = h_joints;
    fig.UserData.h_labels = h_labels;
    fig.UserData.h_axes_base = h_axes_base;
    fig.UserData.h_axes_ee = h_axes_ee;
    fig.UserData.h_pose_text = h_pose_text;
    fig.UserData.arm = arm;
    fig.UserData.current_q = q;
    fig.UserData.obstacle = obstacle;
    fig.UserData.h_obstacle = h_obstacle;
    fig.UserData.h_light = h_light;
    fig.UserData.h_ground = h_ground;
    fig.UserData.h_shadow = h_shadow;
    fig.UserData.obstacle_enabled = obstacle.enabled;
    fig.UserData.obstacle_avoidance_enabled = false;
    
    % 初始化時更新一次，確保 EE 坐標軸正確定位
    updateRobotFigure(fig, q);
end

function vis = bool2visible(flag)
    if flag
        vis = 'on';
    else
        vis = 'off';
    end
end
