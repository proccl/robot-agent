function q_traj = quinticTrajectory(q0, q1, T, fps)
% quinticTrajectory 五次多項式關節空間軌跡規劃
%   q0: 起始關節角 (1x7)
%   q1: 終止關節角 (1x7)
%   T:  總時間 (秒)，默認 5
%   fps: 幀率，默認 30
%   q_traj: Nx7 軌跡矩陣

    if nargin < 3 || isempty(T), T = 5; end
    if nargin < 4 || isempty(fps), fps = 30; end
    
    steps = max(2, round(T * fps) + 1);
    t = linspace(0, T, steps);
    
    q_traj = zeros(steps, 7);
    for j = 1:7
        dq = q1(j) - q0(j);
        a3 = 10 * dq / T^3;
        a4 = -15 * dq / T^4;
        a5 = 6 * dq / T^5;
        q_traj(:, j) = q0(j) + a3 * t.^3 + a4 * t.^4 + a5 * t.^5;
    end
end
