# 腳本模板參考

所有投遞到 `incoming/` 的 `.m` 腳本運行在 `processIncomingCommands` 的函數工作空間，`arm`、`fig`、`current_q` 已作為局部變量存在，直接使用即可。

> 通用前置邏輯：每個模板都應包含 nan 處理和狀態更新。

---

## 模板 A：關節空間（`quinticTrajectory` + `animateRobot`）

**適用場景**：`home` 指令。關節空間保證可達，不觸發 IK。

```matlab
q_target = zeros(1, 7);   % home 位置
q_traj = quinticTrajectory(current_q, q_target, duration, fps);
% 簽名: quinticTrajectory(q0, q1, T, fps)
%   T   = 總時間（秒），默認 5
%   fps = 幀率，默認 30
% 函數內部自動計算 steps = round(T * fps) + 1

animateRobot(fig, q_traj, fps);
% 簽名: animateRobot(fig, q_traj, fps)
%   內部自動更新 fig.UserData.current_q = q_traj(end,:)

fprintf('[Done] home completed.\n');
```

---

## 模板 B：笛卡爾空間軌跡（`planTrajectoryCartesian`）

**適用場景**：`move_to`、`relative_move`、`trajectory` 等點到點運動。獲得笛卡爾直線/圓弧軌跡與四元數 SLERP 姿態插值。

```matlab
T_start = arm.forwardKinematics(current_q);
T_end   = ...;  % 構造目標 4×4 位姿矩陣

steps = max(30, round(duration * 30));
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_start, T_end, steps, 0, duration);
% ⚠️ 返回值順序 [T_traj, q_traj, simin]
% ⚠️ 輸入順序 (T_start, T_end, steps, t_start, t_end)
%    T_start/T_end 必須是 4×4 矩陣，不是 current_q

% ===== nan 處理（必須）=====
if any(isnan(q_traj(1, :))), q_traj(1, :) = current_q; end

nan_rows = find(any(isnan(q_traj), 2));
if ~isempty(nan_rows)
    first_nan = nan_rows(1);
    for i = 1:first_nan-1
        updateRobotFigure(fig, q_traj(i, :));
        drawnow; t0 = tic; while toc(t0) < 1/30, end
    end
    fig.UserData.current_q = q_traj(first_nan-1, :);
    fprintf('[PAUSE] IK unreachable at step %d/%d. current_q=%s target_pos=%s\n', ...
        first_nan, steps, mat2str(fig.UserData.current_q), mat2str(T_end(1:3,4)'));
    return;
end

animateRobot(fig, q_traj, 30);
fprintf('[Done] ...\n');
```

---

## 模板 C：相對移動（保持姿態）

**核心原則**：`T_target = T_cur;` 只改位置分量，保持旋轉矩陣不變。

```matlab
T_cur = arm.forwardKinematics(current_q);
T_target = T_cur;
T_target(axis_idx, 4) = T_target(axis_idx, 4) + distance;
% axis_idx: 1=X, 2=Y, 3=Z

steps = max(30, round(duration * 30));
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);

% nan 處理同上...
```

---

## 模板 D：姿態調整（繞固定軸 Rodrigues）

```matlab
T_cur = arm.forwardKinematics(current_q);

% 繞 n 軸旋轉 theta
n = T_cur(1:3, axis);  % 如 T_cur(1:3,1) = 當前 X 軸
theta = ...;           % 弧度
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);

T_target = T_cur;
T_target(1:3, 1:3) = R * T_cur(1:3, 1:3);

% 後續調用 planTrajectoryCartesian...
```

---

## 模板 E：複合指令（姿態 + 位置同時變化）

`planTrajectoryCartesian` 的 SLERP 可同時處理姿態旋轉與位置移動。只需構造一個同時包含目標位置與目標旋轉矩陣的 `T_target`：

```matlab
T_cur = arm.forwardKinematics(current_q);
T_target = T_cur;

% 同時修改位置和姿態
T_target(1, 4) = T_target(1, 4) + 100;              % 前移 100 mm
T_target(1:3, 1:3) = [0, 0, 1; 1, 0, 0; 0, 1, 0]; % Z 軸指向 +X

steps = max(30, round(duration * 30));
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
% nan 處理與播放...
```

> **關鍵**：`planTrajectoryCartesian` 會在姿態空間走最短路徑（SLERP），中間幀的 X/Y 軸方向會平滑過渡，最終到達指定的目標姿態。

---

## 模板 F：圓軌跡（自定義逐點 IK）

**適用場景**：末端在空間中畫圓。`planTrajectoryCartesian` 只支持直線插值，**無法畫圓弧**，必須自行生成圓周位姿並逐點求 IK。

### 繞末端坐標軸畫圓（通用公式）

讓**當前位置落在圓周上**（`theta = pi`），圓心沿圓平面內的第一個基向量方向偏移 R：

| 旋轉軸 | 圓心偏移方向 | 圓平面基向量 | 備註 |
|--------|-------------|-------------|------|
| **Z 軸** | `+ R * X_dir` | `(X_dir, Y_dir)` | 圓平面 ⊥ Z |
| **X 軸** | `+ R * Y_dir` | `(Y_dir, Z_dir)` | 圓平面 ⊥ X |
| **Y 軸** | `+ R * Z_dir` | `(Z_dir, X_dir)` | 圓平面 ⊥ Y |

```matlab
T_cur = arm.forwardKinematics(current_q);
cur_pos = T_cur(1:3, 4);
X_dir = T_cur(1:3, 1);
Y_dir = T_cur(1:3, 2);
Z_dir = T_cur(1:3, 3);
R = 100;
steps = 150;
theta = linspace(pi, pi + 2*pi, steps);

% === 繞 Z 軸畫圓（圓平面由 X_dir/Y_dir 張成）===
center = cur_pos + R * X_dir;
q_traj = zeros(steps, 7);
for i = 1:steps
    T_i = T_cur;
    pos_i = center + R * (cos(theta(i)) * X_dir + sin(theta(i)) * Y_dir);
    T_i(1:3, 4) = pos_i;
    [qi, err] = arm.inverseKinematics(T_i);
    if err == 0, q_traj(i,:) = qi; else, q_traj(i,:) = nan(1,7); end
end

% nan 處理...
animateRobot(fig, q_traj, 30);
```

> **規律**：圓心偏移方向 = 旋轉軸在末端坐標系中的「下一個」軸（X→Y→Z→X 循環）。這保證 `theta=pi` 時圓上點恰好等於當前位置。

### 多段圓軌跡連續執行

一個腳本中可連續執行多段圓，每段結束後從 `fig.UserData.current_q` 讀取更新後的位姿：

```matlab
% Segment 1: around Z-axis
% ... generate q1, animateRobot(fig, q1, 30) ...

% Segment 2: around X-axis
current_q = fig.UserData.current_q;
T_cur2 = arm.forwardKinematics(current_q);
% ... generate q2 using T_cur2's X_dir2/Y_dir2/Z_dir2 ...
animateRobot(fig, q2, 30);
```

### 平面選擇建議

- **XY 平面等效**（圓平面法向量接近世界 Z）：成功率較高，尤其在中低高度
- **XZ/YZ 平面等效**（圓平面法向量接近世界 X/Y）：容易碰到工作空間邊界，但在合適姿態下仍可成功（經驗 12）

---

## 模板 G：帶障礙物迴避的移動（`planTrajectoryWithObstacle`）

**適用場景**：`move_to` / `relative_move` 且當前 Figure 中啟用了 `obstacle_avoidance_enabled`。規劃器會先檢查直線軌跡是否與紅色障礙球碰撞；若碰撞，則調用 `manipulatorRRT` 自動規劃迴避路徑。

```matlab
ud = fig.UserData;
arm = ud.arm;
current_q = ud.current_q;

T_cur = arm.forwardKinematics(current_q);
T_target = T_cur;
T_target(1:3, 4) = [x; y; z];   % 目標位置

duration = 5;
fps = 30;

if isfield(ud, 'obstacle_avoidance_enabled') && ud.obstacle_avoidance_enabled && ...
   isfield(ud, 'obstacle') && isfield(ud, 'robot_tree') && ~isempty(ud.robot_tree)
    [q_traj, avoided, info] = planTrajectoryWithObstacle(arm, current_q, T_target, ud.obstacle, duration, ud.robot_tree);
    if isempty(q_traj)
        fprintf('%s\n', info);
        fprintf('[PAUSE] Unable to plan obstacle-free path to target. Keeping current pose.\n');
        return;
    end
    if avoided
        fprintf('[Info] Obstacle detected; RRT avoidance path used.\n');
    end
else
    steps = max(30, round(duration * fps));
    [~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
end

% nan 處理（planTrajectoryWithObstacle 不返回 nan，但為統一保留）
if any(isnan(q_traj(:)))
    fprintf('[PAUSE] Trajectory contains NaN. Keeping current pose.\n');
    return;
end

animateRobot(fig, q_traj, fps);
fprintf('[Done] move_to with obstacle avoidance completed.\n');
```

> **前提條件**：`robotagent.m` 中需將 `enable_obstacle_avoidance` 設為 `true` 後再啟動，才會構建 `fig.UserData.robot_tree`。若啟動後 `obstacle_avoidance_enabled` 為 `false`，腳本會回退到普通笛卡爾軌跡。
> 
> **開關控制**：用戶可在 MATLAB 命令行切換：
> ```matlab
> fig.UserData.obstacle_enabled = false;            % 隱藏紅球
> fig.UserData.obstacle_avoidance_enabled = false;  % 關閉避障規劃
> ```

### 模板 G1：移向障礙物中心（動態讀取球位置）

```matlab
ud = fig.UserData;
arm = ud.arm;
current_q = ud.current_q;

T_cur = arm.forwardKinematics(current_q);
T_target = T_cur;
T_target(1:3, 4) = ud.obstacle.center;   % 始終使用當前障礙物中心

% 後續與模板 G 相同：調用 planTrajectoryWithObstacle 或 planTrajectoryCartesian
```

> 注意：若目標點在障礙物內部，`manipulatorRRT` 會報錯 "goal configuration is in world collision"。此時應將目標設在球外。

### 模板 G2：沿障礙物方向移動固定距離

```matlab
ud = fig.UserData;
arm = ud.arm;
current_q = ud.current_q;

T_cur = arm.forwardKinematics(current_q);
cur_pos = T_cur(1:3, 4);
center = ud.obstacle.center;

direction = center - cur_pos;
unit_dir = direction / norm(direction);

T_target = T_cur;
T_target(1:3, 4) = cur_pos + 500 * unit_dir;   % 向球心方向移動 500 mm

% 後續與模板 G 相同
```

---

## 關於動畫幀率

`animateRobot` 內部使用 `pause(1/fps)` 控制幀率。但在 `timer` 回調執行鏈中，`pause` 被 MATLAB 忽略，導致動畫「快進」——循環瞬間完成，所有幀在短時間內全部刷新。

- **當前狀態**：功能不受影響（機械臂會正確到達目標），只是沒有平滑動畫效果
- **如需真實時間播放**：將 `animateRobot` 內部的 `pause` 改為 `tic/toc` busy-wait，或在腳本中自行遍歷 `q_traj`：
  ```matlab
  for i = 1:size(q_traj, 1)
      updateRobotFigure(fig, q_traj(i, :));
      drawnow;
      t0 = tic; while toc(t0) < 1/30, end
  end
  ```
