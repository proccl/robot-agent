# 函數簽名確認表

> **原則**：所有簽名以 `src/*.m` 源碼中的 `function` 行為準。憑記憶寫腳本極易出錯。

## 核心函數

### `quinticTrajectory` — 五次多項式關節空間軌跡

```matlab
function q_traj = quinticTrajectory(q0, q1, T, fps)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `q0` | `1×7 double` | 起始關節角（弧度） |
| `q1` | `1×7 double` | 終止關節角（弧度） |
| `T` | `double` | 總時間（秒），**默認 5** |
| `fps` | `double` | 幀率，**默認 30** |
| `q_traj` | `N×7 double` | 關節角軌跡，N = `round(T * fps) + 1` |

**常見錯誤**：將 `steps` 作為第三參數傳入。函數內部會自動計算 steps，調用者只需傳時間和幀率。

**正確用法**：
```matlab
q_traj = quinticTrajectory(current_q, q_target, 5, 30);  % 5秒，30fps
% 或
q_traj = quinticTrajectory(current_q, q_target);         % 默認 5秒，30fps
```

---

### `animateRobot` — 播放機械臂動畫

```matlab
function animateRobot(fig, q_traj, fps)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `fig` | `figure handle` | Figure 句柄 |
| `q_traj` | `N×7 double` | 關節角軌跡 |
| `fps` | `double` | 幀率，**默認 30** |

**內部行為**：
1. 遍歷 `q_traj` 每一行
2. 調用 `updateRobotFigure(fig, q_traj(i, :))`
3. `drawnow limitrate`
4. `pause(1/fps)`
5. 結束後自動更新 `fig.UserData.current_q = q_traj(end, :)`

**⚠️ 注意**：`pause(1/fps)` 在 `timer` 回調執行鏈中被 MATLAB 忽略，導致動畫「快進」。功能不受影響，但無平滑動畫效果。

**常見錯誤**：誤傳 `arm` 作為第二參數。正確是 `(fig, q_traj, fps)`。

---

### `planTrajectoryCartesian` — 笛卡爾空間軌跡規劃

```matlab
function [T_traj, q_traj, simin] = planTrajectoryCartesian(obj, T_start, T_end, steps, t_start, t_end)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `obj` | `Arm7R` | 機械臂對象 |
| `T_start` | `4×4 double` | 起始位姿（齊次變換矩陣） |
| `T_end` | `4×4 double` | 終止位姿（齊次變換矩陣） |
| `steps` | `int` | 插值點數，**默認 100** |
| `t_start` | `double` | 起始時間（秒），**默認 0** |
| `t_end` | `double` | 終止時間（秒），**默認 1** |

| 返回值 | 類型 | 說明 |
|--------|------|------|
| `T_traj` | `4×4×steps double` | 位姿軌跡陣列 |
| `q_traj` | `steps×7 double` | 關節角軌跡 |
| `simin` | `steps×8 double` | `[time, q1..q7]`，Simulink 可用 |

**記憶口訣**：「**T 在前，q 在中，simin 在後**」

**常見錯誤**：
1. 返回值順序寫錯：`[q_traj, ~]` 拿到的是 `T_traj`（4×4×steps 位姿陣列）
2. 輸入第一參數傳 `current_q`（1×7）—— 必須傳 `T_start`（4×4 矩陣）

**正確用法**：
```matlab
T_start = arm.forwardKinematics(current_q);
T_end   = T_start;
T_end(1:3, 4) = T_end(1:3, 4) + [100; 0; 0];  % 修改位置

steps = max(30, round(duration * 30));
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_start, T_end, steps, 0, duration);
```

---

### `forwardKinematics` — 正向運動學

```matlab
function T = forwardKinematics(obj, q)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `q` | `1×7 double` | 關節角（弧度） |
| `T` | `4×4 double` | 齊次變換矩陣 |

---

### `inverseKinematics` — 逆向運動學（解析閉式解）

```matlab
function [q, err] = inverseKinematics(obj, T)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `T` | `4×4 double` | 目標位姿 |
| `q` | `1×7 double` | 關節角解（僅當 `err==0` 時有效） |
| `err` | `int` | `0`=成功，`1`=無解 |

> 固定 `θ1=0` 消除冗餘。浮點誤差可能導致嚴格判斷失敗，返回 `err=1`。

---

### `updateRobotFigure` — 更新 Figure 顯示

```matlab
function updateRobotFigure(fig, q)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `fig` | `figure handle` | Figure 句柄（需包含 `UserData` 圖形句柄） |
| `q` | `1×7 double` | 關節角 |

**內部行為**：`set()` 更新連桿/關節/標籤/EE 坐標軸 + 障礙球可見性 + 左上角位姿文本 + `drawnow limitrate`。

---

## 避障相關函數

### `buildRobotTree` — 構建 rigidBodyTree

```matlab
function tree = buildRobotTree(arm)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `arm` | `Arm7R` | 機械臂對象 |
| `tree` | `rigidBodyTree` | Robotics System Toolbox 機器人樹 |

**內部行為**：根據 `Arm7R` 的標準 DH 參數構建 `rigidBodyTree`，為每個非固定 body 原點添加 `collisionSphere(20)`（單位 mm）。`DataFormat = 'row'`。

---

### `checkRobotObstacleCollision` — 檢測機械臂與障礙球碰撞

```matlab
function [is_collision, min_dist] = checkRobotObstacleCollision(robot_tree, q, obstacle)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `robot_tree` | `rigidBodyTree` | 由 `buildRobotTree` 構建 |
| `q` | `1×7 double` | 關節角 |
| `obstacle` | `struct` | 必須包含 `center`（3×1）和 `radius` |
| `is_collision` | `logical` | `true` 表示發生碰撞 |
| `min_dist` | `double` | 最小分離距離（碰撞時為 `-inf`） |

---

### `planTrajectoryWithObstacle` — 帶避障的軌跡規劃

```matlab
function [q_traj, avoided, info] = planTrajectoryWithObstacle(arm, current_q, T_target, obstacle, duration, robot_tree)
```

| 參數 | 類型 | 說明 |
|------|------|------|
| `arm` | `Arm7R` | 機械臂對象 |
| `current_q` | `1×7 double` | 起始關節角 |
| `T_target` | `4×4 double` | 目標位姿（齊次變換矩陣） |
| `obstacle` | `struct` | 障礙物，含 `center`、`radius`、`safety_margin` |
| `duration` | `double` | 總時間（秒），默認 5 |
| `robot_tree` | `rigidBodyTree` | 由 `buildRobotTree` 構建 |

| 返回值 | 類型 | 說明 |
|--------|------|------|
| `q_traj` | `N×7 double` | 關節角軌跡；規劃失敗時為空矩陣 `[]` |
| `avoided` | `logical` | `true` 表示使用了 RRT 繞行 |
| `info` | `char` | 失敗時包含 `[PAUSE] ...` 信息 |

**行為**：
1. 先生成笛卡爾直線軌跡 `q_nominal`
2. 若無碰撞，直接返回 `q_nominal`，`avoided = false`
3. 若有碰撞，調用 `manipulatorRRT` 規劃繞行路徑
4. RRT 成功後用 `quinticTrajectory` 分段平滑，並驗證中間點
5. RRT 失敗或驗證失敗：返回 `[]`，`info` 含 `[PAUSE]`

**注意**：起點和終點都必須是無碰撞狀態。目標在障礙物內部時會失敗。
