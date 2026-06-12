# 常見問題排查與開發經驗

## 常見問題速查表

| 現象 | 原因 | 解決 |
|------|------|------|
| `[ERR] Axes not found in figure` | `robotagent.m` 覆蓋了 `fig.UserData` | 關閉 Figure，重新運行 `robotagent.m` |
| `[ERR] 字符向量未正常終止` | `.m` 文件中的 `\n` 被錯誤解析為換行 | 確保 `fprintf` 字符串中的 `\n` 是字面量（無 BOM 編碼） |
| `[ERR] Target unreachable` | 目標位姿在機械臂工作空間外 | 先移動到更容易達到的位置，再放寬姿態約束 |
| 投遞後完全沒反應 | `is_busy` 卡住或 timer 停止 | 關閉 Figure，重新運行 `robotagent.m` |
| 動畫卡頓 | `drawnow limitrate` 丟幀 | 正常現象，降低 fps 或縮短軌跡點數 |
| `[PAUSE] IK unreachable at step X` | 笛卡爾軌跡中間點 IK 無解 | 見「PAUSE 狀態處理」 |
| `[ERR] xxx (line N): ...` | 腳本執行出錯 | 查看 **LOG 區塊** 獲取詳細堆棧 |
| 沒看到 `========== LOG: xxx ==========` 區塊 | `logs/` 目錄無寫入權限，或 `diary` 被腳本內部關閉 | 檢查 `logs/` 目錄權限；確認生成腳本不含 `diary off` |
| `[Done]` 但軌跡完全錯誤（亂跳/不沿直線） | `planTrajectoryCartesian` 返回值順序寫錯，拿到 `T_traj` 而非 `q_traj` | 必須寫 `[~, q_traj, ~] = arm.planTrajectoryCartesian(...)` |
| `[Done]` 但只動了一步就停 | `planTrajectoryCartesian` 大量 `nan`，或腳本用了 `quinticTrajectory` 而非笛卡爾插值 | 確認使用 `[~, q_traj, ~] = planTrajectoryCartesian(...)` 並處理 nan |
| `[ERR] 无法解析名称 'arm.forwardKinematics'` | `assignin('base', ...)` 把變量注入 base workspace，但 `run()` 在函數工作空間執行腳本 | 改用局部變量：`arm = fig.UserData.arm; current_q = fig.UserData.current_q;` |
| 修改 `.m` 後需重啟才能生效 | 以為 MATLAB 會緩存函數 | MATLAB 會自動檢測文件修改並重載，通常無需重啟 |
| 動畫「快進」/瞬間完成 | `animateRobot` 使用 `pause(1/fps)`，在 timer 回調中被 MATLAB 忽略 | 正常現象，功能不受影響；如需真實時間播放需改用 `tic/toc` busy-wait |
| `[ERR] The goal configuration is in world collision` | 目標點在障礙物內部，RRT 無法規劃無碰撞路徑 | 將目標設在球外（如球表面外 10~50 mm），或關閉避障 |
| `[PAUSE] Unable to plan obstacle-free path` | RRT 找不到繞行路徑 | 嘗試換一個目標點，或確認 `ud.obstacle.center` 與實際球位置一致 |
| 機械臂移向錯誤的球位置 | 腳本中硬編碼了障礙物坐標 | 始終使用 `ud.obstacle.center` 動態讀取當前障礙物位置 |
| 杆子穿過紅球但沒報碰撞 | 當前碰撞幾何是關節點小球近似，非精確圓柱體 | 目前為已知限制；如需精確檢測需後續添加圓柱體碰撞體 |

---

## 開發與調試經驗

### 經驗 1：絕對不要在 `processIncomingCommands` 中覆蓋 `fig.UserData.current_q`

**問題**：腳本執行後更新了 `fig.UserData.current_q = q_target`，但 `processIncomingCommands` 隨後執行 `fig.UserData.current_q = evalin('base', 'current_q')`。base workspace 中的 `current_q` 仍是舊值，導致正確更新被覆蓋。

**症狀**：`home` 看起來「一步到位」；`relative_move` 後接其他指令時位置重置。

**解決**：刪除該同步邏輯，讓腳本直接操作 `fig.UserData.current_q`。

---

### 經驗 2：timer 回調中 `pause` 被忽略

**問題**：`processIncomingCommands` 在 timer `TimerFcn` 中執行腳本，腳本內的 `pause(1/30)` 被 MATLAB 忽略。

**症狀**：手動運行腳本動畫正常，投入 `incoming/` 自動執行時「一步到位」。

**解決（腳本層面）**：如需真實時間播放，自行遍歷並使用 busy-wait：
```matlab
for i = 1:size(q_traj, 1)
    updateRobotFigure(fig, q_traj(i, :));
    drawnow;
    t0 = tic; while toc(t0) < 1/30, end
end
```

> **注意**：當前 `animateRobot.m` 源碼使用 `pause(1/fps)`，因此通過 `animateRobot` 調用必然快進。若未來修復 `animateRobot`，上述 busy-wait 方案才是正確做法。

---

### 經驗 3：`planTrajectoryCartesian` 與 `inverseKinematics` 的數值兼容性

**問題**：`forwardKinematics` 矩陣連乘產生浮點誤差，`inverseKinematics` 的解析解判斷 `< 0` 過於嚴格，導致 `planTrajectoryCartesian` 中間點大量返回 `nan`。

**症狀**：生成的軌跡第一步或中間點全部 `nan`，動畫無法播放。

**應對**（不改 `Arm7R.m` 的前提下）：
- `home` 指令不使用 `planTrajectoryCartesian`，直接用 `quinticTrajectory`
- 其他指令優先嘗試 `planTrajectoryCartesian`，若失敗則暫停並詢問用戶回退到 `quinticTrajectory`
- 未來如需嚴格笛卡爾直線，可在腳本中對旋轉矩陣做 SVD 正交化後再傳給 IK

---

### 經驗 4：`run()` 在函數工作空間執行，`assignin('base')` 無效

**問題**：`processIncomingCommands` 使用 `assignin('base', 'arm', fig.UserData.arm)` 將變量注入 base workspace，但 `run()` 在函數內部調用時，腳本實際運行在**函數工作空間**。

**症狀**：腳本中 `arm.forwardKinematics(...)` 報錯「无法解析名称 'arm.forwardKinematics'」。

**解決**：直接聲明局部變量：
```matlab
arm = fig.UserData.arm;
current_q = fig.UserData.current_q;
```

> 推論：`run()` 始終在調用者的工作空間中執行腳本。在函數 A 中調用 `run()`，腳本就在函數 A 的工作空間中運行。

---

### 經驗 5：`planTrajectoryCartesian` 返回值順序是最隱蔽的坑

**問題**：回傳順序是 `[T_traj, q_traj, simin]`，但直覺上很容易寫成 `[q_traj, ~]`。結果拿到的是 4×4×steps 的位姿陣列。

**症狀**：`[Done]` 輸出正常，但機械臂亂跳、不沿直線。

**正確寫法**：
```matlab
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
```

**記憶口訣**：「T 在前，q 在中，simin 在後」—— 位姿陣列、關節角、Simulink 輸入。

---

### 經驗 6：複合指令（姿態 + 位置）可用單一 `planTrajectoryCartesian`

**事實**：`planTrajectoryCartesian` 的 SLERP 插值本質就是**同時對位置線性插值 + 姿態球面插值**。只需構造一個同時包含目標位置與目標旋轉矩陣的 `T_target`，單次調用即可完成。

**成功範例**：home 位置 → 前移 100 mm + Z 軸轉水平（指向 +X）：
```matlab
T_target(1, 4) = T_target(1, 4) + 100;
T_target(1:3, 1:3) = [0, 0, 1; 1, 0, 0; 0, 1, 0];
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
```
結果：5 秒內平滑完成位置與姿態的同步過渡，無 PAUSE。

---

### 經驗 7：錯誤日誌與狀態文件是遠程調試的生命線

**問題**：用戶僅反饋「沒有動」或「一步到位」，無法定位是腳本錯誤、IK 失敗還是動畫刷新問題。

**解決**：
- `processIncomingCommands` 捕獲錯誤時輸出 `[ERR] 文件名:行號 — 錯誤信息`
- `[ERR]` 與 `[PAUSE]` 信息均由 `diary` 自動寫入該次 `logs/log_xxx.txt`

這讓 Kimi CLI 端可以遠程讀取並精確診斷，無需用戶手動複製 MATLAB 命令行輸出。

---

### 經驗 8：函數簽名必須以源碼為準，不能憑記憶

**問題**：憑記憶寫腳本時，容易混淆多個函數的參數順序和簽名。

**已確認的簽名**（以 `src/*.m` 源碼為準）：

| 函數 | 正確簽名 | 常見錯誤 |
|------|---------|---------|
| `quinticTrajectory` | `quinticTrajectory(q0, q1, T, fps)` | 誤傳 `steps` 作為第三參數；T 是總時間（秒），fps 默認 30 |
| `animateRobot` | `animateRobot(fig, q_traj, fps)` | 誤傳 `arm` 作為第二參數 |
| `planTrajectoryCartesian` | `planTrajectoryCartesian(T_start, T_end, steps, t_start, t_end)` | 誤傳 `current_q` 作為第一參數；T_start/T_end 必須是 4×4 位姿矩陣 |

**教訓**：每次生成腳本前，如果對簽名有疑問，用 `grep "^function" src/xxx.m` 確認。

---

### 經驗 9：MATLAB 會自動重新加載修改後的函數文件

**問題**：修改 `processIncomingCommands.m` 後，以為必須重啟 `robotagent` 才能生效。

**事實**：MATLAB 在調用函數時會自動檢測文件修改時間戳，若發現文件已更新，則自動重新解析編譯。用戶未重啟 `robotagent`，修改後的函數已正確生效。

**例外**：若函數已被 `pcode` 編譯或有 persistent 變量鎖定，可能需要 `clear functions` 強制重載。但普通 `.m` 文件無需此操作。

---

### 經驗 10：圓軌跡必須自定義逐點 IK，`planTrajectoryCartesian` 不支持圓弧

**問題**：直覺上可能認為 `planTrajectoryCartesian` 可以畫圓弧，因為它做笛卡爾插值。但實際上它只支持**位置線性插值 + 姿態 SLERP**，路徑是直線而非圓弧。

**症狀**：用 `planTrajectoryCartesian` 的兩個端點設為圓周直徑兩端，期望它走半圓，結果走直線。

**解決**：圓軌跡必須自行生成圓周上的位姿矩陣，逐點調用 `inverseKinematics`：
```matlab
for i = 1:steps
    T_i(1,4) = center(1) + R*cos(theta(i));
    T_i(2,4) = center(2) + R*sin(theta(i));
    [qi, err] = arm.inverseKinematics(T_i);
    % ...
end
```

---

### 經驗 11：工作空間取決於高度 **和** 姿態，不是單一變量

**問題**：直覺上認為「高度越低 = 工作空間越小」，因此在 Z=500 畫圓失敗後，認為 Z=200 更不可能成功。

**事實**：
- **Z=500 保持 home 姿態**：XY 平面 R=100 的圓在 step 2 失敗
- **Z=200 經過姿態調整後**：XY 平面 R=100 的圓完整成功

**原因**：`z下降到500` 只改變了 Z 坐標，保持了 home 姿態（末端 Z 朝下）。在 Z=500 高度，這個姿態接近工作空間邊界。而經過後續運動後，關節角重新分佈，末端姿態更適應低高度，反而獲得了更大的 XY 可達範圍。

**教訓**：工作空間是**末端位姿（位置+姿態）的函數**，不是單純高度的函數。畫圓前若失敗，可嘗試：
1. 改變末端姿態（如讓 Z 軸水平）
2. 換一個高度重試
3. 換一個平面（XY 平面通常比 XZ 平面更容易成功）

---

### 經驗 12：XY 平面圓比 XZ 平面圓更容易成功

**問題**：在 XZ 平面（垂直面）畫 R=200 的圓，step 31 被阻擋。

**事實**：同一機械臂在 **XY 平面**（水平面）畫 R=100 的圓成功。

**原因**：
- XZ 平面的圓同時改變 X 和 Z，Z 方向的運動容易碰到工作空間的上下邊界
- XY 平面的圓只改變 X 和 Y，Z 保持不變，避開了高度方向的限制
- 7R 機械臂的 reachable workspace 在水平方向（XY）通常比垂直方向（XZ/YZ）更寬廣

**教訓**：優先嘗試 **XY 平面** 的圓軌跡。若必須在垂直面畫圓，縮小半徑或降低圓心高度。

---

### 經驗 13：繞末端坐標軸畫圓的通用公式

**問題**：為不同旋轉軸（X/Y/Z）寫圓軌跡時，容易混淆圓心偏移方向和圓平面基向量。

**通用規律**（以末端坐標系為基準）：

| 旋轉軸 | 圓心偏移方向 | 圓平面基向量 |
|--------|-------------|-------------|
| **Z 軸** | `+ R * X_dir` | `(X_dir, Y_dir)` |
| **X 軸** | `+ R * Y_dir` | `(Y_dir, Z_dir)` |
| **Y 軸** | `+ R * Z_dir` | `(Z_dir, X_dir)` |

**記憶法**：圓心偏移方向 = 旋轉軸的「下一個」軸（X→Y→Z→X 循環）。這保證 `theta=pi` 時圓上點恰好等於當前位置。

**已驗證**：
- 末端 Z→+X 姿態下，繞 Z 軸（YZ 平面）和繞 X 軸（XZ 平面）的 R=100 圓均成功
- 多段圓軌跡可在同一腳本中連續執行，段間通過 `fig.UserData.current_q` 傳遞位姿

---

### 經驗 14：障礙物位置必須動態讀取，不能硬編碼

**問題**：生成「移向球心」腳本時，把目標位置硬編碼為 `[800; 0; 0]`，但用戶實際把障礙球移到了 `[400; 0; 500]`，導致機械臂移向錯誤位置。

**正確做法**：始終從 `fig.UserData.obstacle.center` 讀取當前障礙物位置：
```matlab
T_target(1:3, 4) = ud.obstacle.center;
```

**教訓**：任何與障礙物相關的目標點、方向向量都應基於 `ud.obstacle.center` 和 `ud.obstacle.radius` 動態計算，不能假設默認值。

---

### 經驗 15：目標點在障礙物內部時 RRT 會報錯

**問題**：用戶要求「移向球心」，但球心位於障礙物內部。`planTrajectoryWithObstacle` 調用 `manipulatorRRT` 後報錯：
```text
The goal configuration of the robot is in world collision.
```

**原因**：避障規劃要求起點和終點都必須是無碰撞狀態。終點在障礙物內部時，不存在無碰撞路徑。

**解決**：
- 若要測試避障繞行，應將目標設在**球外**，例如球的表面外 10~50 mm
- 若必須進入球內，需要關閉避障：`fig.UserData.obstacle_avoidance_enabled = false;`

---

### 經驗 16：「向某方向移動 N」要計算單位方向向量

**問題**：用戶說「向球心方向移動 500」，不能直接將當前位置加 `[500; 0; 0]`，而應沿當前位置到球心的方向移動 500 mm。

**正確計算**：
```matlab
cur_pos = T_cur(1:3, 4);
center = ud.obstacle.center;
direction = center - cur_pos;
unit_dir = direction / norm(direction);
T_target(1:3, 4) = cur_pos + 500 * unit_dir;
```

**教訓**：相對移動若涉及方向，必須先歸一化方向向量，再乘以距離。

---

### 經驗 17：`ver('Robotics System Toolbox')` 在某些電腦上檢測失敗

**問題**：`robotagent.m` 啟動時用 `ver('Robotics System Toolbox')` 檢測工具箱，在部分安裝上返回空，導致避障被錯誤禁用。

**正確檢測方式**：
```matlab
v = ver;
has_robotics_toolbox = any(strcmpi({v.Name}, 'Robotics System Toolbox'));
```

**教訓**：工具箱檢測應使用 `ver` 返回的結構體數組進行名稱匹配，而非依賴 `ver('Name')`。

---

### 經驗 18：當前碰撞幾何是關節點近似，不保證杆子碰撞

**問題**：用戶問「杆子是否也會和球碰撞」。當前 `buildRobotTree.m` 中為每個非固定 body 添加的是半徑 20 mm 的小球碰撞體，而非圓柱體。

**現狀**：
- ✅ 可檢測末端/關節點進入障礙物
- ⚠️ 杆子中間穿過障礙物、兩端關節點都在球外時，可能檢測不到

**後續改進**：如需精確連桿碰撞，需為每根連桿添加 `collisionCylinder`，並正確設置長度、軸向和 pose。
