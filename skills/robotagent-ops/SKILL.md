---
name: robotagent-ops
description: 操作 RobotAgent 文件監聽架構機械臂系統的指南。涵蓋自然語言指令、自定義姿態調整、常見問題排查與最佳實踐。
---

# RobotAgent 操作指南

## 架構概述

RobotAgent 採用 **文件監聽架構**（非 TCP）：

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Kimi CLI 窗口（Agent + Skill）                       │
│  ┌─────────────────────────────┐    ┌─────────────────────────────────┐    │
│  │  用戶輸入自然語言           │    │  執行結束後：讀取並分析 Log     │    │
│  │  • Skill 解析意圖與模板     │    │  • 讀取 logs/log_*.txt          │    │
│  │  • Agent 生成 .m 代碼       │    │  • 分析 [Done]/[ERR]/[PAUSE]    │    │
│  │  • 寫入 incoming/           │    │  • 向用戶匯報結果               │    │
│  └──────────────┬──────────────┘    └───────────────▲────────────────┘    │
└─────────────────┼────────────────────────────────────┼──────────────────────┘
                  │ 寫入 cmd_xxx.m                     │ 讀取 log_*.txt
                  ▼                                    │
┌─────────────────────────────────────────────────────────────────────────────┐
│                           文 件 系 統 （中間層）                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐  │
│  │  incoming/      │  │  logs/          │  │  incoming_history/          │  │
│  │  (指令隊列)     │  │  (執行日誌)     │  │  (腳本歷史歸檔)             │  │
│  │  └── failed/    │  │  └── log_*.txt  │  │                             │  │
│  │      (失敗歸檔) │  │                 │  │                             │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────────┘  │
└─────────────────┬────────────────────────────────────┼──────────────────────┘
                  │ 讀取 cmd_*.m                       │ 寫入 log_*.txt
                  ▼                                    │
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MATLAB 窗口                                     │
│  ┌───────────────────────┐    ┌──────────────────────────────────────────┐  │
│  │ 用戶點擊運行          │    │ processIncomingCommands (timer)          │  │
│  │ robotagent.m          │    │  ├── 掃描 incoming/，按時間排序         │  │
│  │  → Figure 彈出        │    │  ├── diary 開始記錄 → logs/log_*.txt    │  │
│  │  → timer 啟動監聽     │    │  ├── run(cmd_path) 執行腳本             │  │
│  └───────────────────────┘    │  ├── diary off; type(log) 打印 LOG      │  │
│              │                 │  ├── 複製到 incoming_history/           │  │
│              │ timer 0.5s      │  └── 失敗時移至 incoming/failed/        │  │
│              ▼                 └──────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────────────────┐   │
│  │ Figure 機械臂動畫更新（由腳本內 updateRobotFigure 驅動）              │   │
│  └───────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

- **啟動方式**：在 MATLAB Editor 雙擊 `robotagent.m` 或點 Run
- **核心組件**：
  - `robotagent.m`：一鍵啟動，初始化 Figure + 啟動 timer
  - `processIncomingCommands.m`：timer 回調，按文件名排序執行
  - `generate_robot_cmd.m`：結構化指令 → 可執行 `.m` 文件
  - `parseNaturalLanguage.m`：中英自然語言 → `cmd_struct`
  - `quinticTrajectory.m`：五次多項式關節空間軌跡

## Kimi CLI 操作規範（AI 必讀）

### 投放腳本後必做：主動讀取並分析最新 Log

每次向 `incoming/` 投放腳本後，Kimi CLI **必須**主動完成以下步驟，**不得等待用戶主動詢問**：

1. **讀取 `logs/` 目錄下最新的 `log_*.txt`**：
   ```bash
   ls -t logs/log_*.txt | head -1 | xargs cat
   ```
2. **分析 Log 內容**，確認執行結果：
   - `[Done]` — 執行成功，向用戶確認軌跡結果
   - `[ERR]` — 執行失敗，分析錯誤行號與原因並報告
   - `[PAUSE]` — 軌跡中斷，提取 `current_q` 與 `target_pos` 分析是否超出工作空間
3. **向用戶匯報分析結果**

> ❌ **禁止行為**：投放腳本後只回覆「已寫入，請觀察」而不主動讀取 log。

---

## 快速操作

### 方式一：AI 直接投遞（自然語言）⭐ 推薦

直接告訴 AI 自然語言指令，AI 會直接生成 `.m` 文件到 `incoming/` 並執行。

**範例**：
- 「回零位」
- 「z向下200」
- 「走到 500 0 800」
- 「x向前100，然後 z軸繞x軸轉至水平」

> ⚠️ **每次指令執行後，必須查看 Command Window 的 `========== LOG: xxx ==========` 區塊**，確認 `[Done]`、`[ERR]` 或 `[PAUSE]` 狀態。

### 方式二：MATLAB 自然語言解析

在 MATLAB Command Window 輸入：

```matlab
generate_robot_cmd(parseNaturalLanguage('回零位'), 'incoming');
generate_robot_cmd(parseNaturalLanguage('z向下200'), 'incoming');
```

### 方式三：直接結構體

```matlab
cmd = struct('cmd','move_to','position',[500,0,800],'duration',5);
generate_robot_cmd(cmd, 'incoming');
```

## 支持的自然語言指令

| 指令 | 示例 | 對應 `cmd` |
|------|------|-----------|
| 回零位 | `回零位`、`home` | `home` |
| 絕對移動 | `走到 500 0 800`、`move to (500,0,800)` | `move_to` |
| 相對移動 | `z向下200`、`向下移動100`、`x向移動-50` | `relative_move` |
| 關節運動 | `關節3轉45度`、`joint 2 30 deg` | `joint_move` |
| 畫圓 | `畫圓 半徑200`、`circle radius 200` | `trajectory` (circle) |
| 查詢狀態 | `status`、`姿態` | `get_status` |

### 姿態調整指令（需手寫腳本）

以下指令 **不支持自然語言解析**，需由 AI 直接生成自定義 `.m` 腳本：

| 動作 | 核心算法 |
|------|---------|
| X 軸水平（Y 軸不動） | 繞 Y 軸旋轉 `atan2(X_z, X_x)`，Rodrigues 公式 |
| Z 軸水平（Y 軸不動） | 繞 Y 軸旋轉 `atan2(-Z_z, X_z)`，Rodrigues 公式 |
| Z 軸反向（X 軸不動） | 繞 X 軸旋轉 180°：`R = I + 2*K^2` |
| Z 軸水平（繞 X 軸） | 繞 X 軸旋轉 `atan2(-Z_z, Z_y)`，Rodrigues 公式 |
| Z 軸指向 +X（Y 軸不動） | 繞 Y 軸旋轉 `-90°`：`R_y(-90°) * [0;0;-1] = [1;0;0]` |

### ⚠️ 關鍵洞察：home 位置下的軸向對應

當前 **home 位姿**下：
- 末端 **X 軸** = `[0, 1, 0]` → 與**世界 Y 軸完全重合**
- 末端 **Y 軸** = `[1, 0, 0]` → 與**世界 X 軸完全重合**
- 末端 **Z 軸** = `[0, 0, -1]` → 朝下

**這意味著**：
- 用戶說「繞末端 X 軸旋轉」→ **等價於繞世界 Y 軸旋轉**
- 用戶說「繞末端 Y 軸旋轉」→ **等價於繞世界 X 軸旋轉**

**繞末端 X 軸（= 世界 Y 軸）的效果**：

- Z 軸在 XZ 平面內轉動：從朝下 `[0,0,-1]` 可以轉到朝前 `[1,0,0]`（+X）或朝後 `[-1,0,0]`（-X）
- X 軸方向保持 `[0,1,0]` 不變
- **無法讓 Z 軸朝 ±Y 方向**（因為繞 Y 軸旋轉不改變 Y 分量）

**Rodrigues 公式繞任意軸旋轉**：

```matlab
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
```

## 自定義 `.m` 腳本模板

所有投遞到 `incoming/` 的 `.m` 腳本必須以固定模式開頭和結尾：

```matlab
fig = findobj('Type','figure','Name','RobotAgent - 7R Arm');
if isempty(fig)||~isvalid(fig), error('Figure not found.'); end
ud = fig.UserData; arm = ud.arm; current_q = ud.current_q;

% ===== 用戶代碼區域 =====
% ... 計算 q_target ...

q_traj = quinticTrajectory(current_q, q_target, duration, 30);
for i = 1:size(q_traj,1)
    updateRobotFigure(fig, q_traj(i,:));
    drawnow;                       % 注意：timer 回調中不要用 drawnow limitrate
    t0 = tic;
    while toc(t0) < 1/30, end      % 注意：timer 回調中 pause 會被忽略，改用 busy-wait
end
fig.UserData.current_q = q_target;
fprintf('[Done] ...\n');
```

### 笛卡爾空間軌跡（`planTrajectoryCartesian`）

**`home` 以外的點到點運動**（`relative_move`、`move_to`、`trajectory`）必須使用 `planTrajectoryCartesian`，以獲得笛卡爾直線/圓弧軌跡與四元數 SLERP 姿態插值：

```matlab
steps = max(30, round(duration * 30));
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
% ⚠️ 注意：返回值順序是 [T_traj, q_traj, simin]，第一個是 4x4xsteps 位姿陣列！
```

**nan 處理**（FK→IK 數值兼容性）：
```matlab
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
        first_nan, steps, mat2str(fig.UserData.current_q), mat2str(T_target(1:3,4)'));
    return;
end
```

### 相對移動（保持姿態，笛卡爾空間）

```matlab
T_cur = arm.forwardKinematics(current_q);
T_target = T_cur;
T_target(axis_idx, 4) = T_target(axis_idx, 4) + distance;

steps = max(30, round(duration * 30));
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);

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
        first_nan, steps, mat2str(fig.UserData.current_q), mat2str(T_target(1:3,4)'));
    return;
end

for i = 1:size(q_traj, 1)
    updateRobotFigure(fig, q_traj(i, :));
    drawnow; t0 = tic; while toc(t0) < 1/30, end
end
fig.UserData.current_q = q_traj(end, :);
```

### 姿態調整（繞固定軸）

```matlab
% 繞 n 軸旋轉 theta（Rodrigues）
n = T_cur(1:3, axis);  % 如 X 軸、Y 軸、Z 軸
theta = ...;           % 計算旋轉角
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
T_target = T_cur;
T_target(1:3, 1:3) = R * T_cur(1:3, 1:3);
```

### 複合指令（姿態 + 位置同時變化）

`planTrajectoryCartesian` 的 SLERP 插值可同時處理姿態旋轉與位置移動。例如：末端前移 100 mm 且 Z 軸轉至水平（指向 +X）：

```matlab
T_cur = arm.forwardKinematics(current_q);
T_target = T_cur;
T_target(1, 4) = T_target(1, 4) + 100;           % 前移 100 mm
T_target(1:3, 1:3) = [0, 0, 1; 1, 0, 0; 0, 1, 0]; % Z 軸指向 +X（繞世界 Y 軸 -90°）

steps = max(30, round(duration * 30));
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
% ... nan 處理與播放 ...
```

> **關鍵**：`planTrajectoryCartesian` 會在姿態空間走最短路徑（SLERP），中間幀的 X/Y 軸方向會平滑過渡，最終到達指定的目標姿態。

## 常見問題排查

| 現象 | 原因 | 解決 |
|------|------|------|
| `[ERR] Axes not found in figure` | `robotagent.m` 覆蓋了 `fig.UserData` | 關閉 Figure，重新運行 `robotagent.m` |
| `[ERR] 字符向量未正常終止` | `.m` 文件中的 `\n` 被錯誤解析為換行 | 確保 `fprintf` 字符串中的 `\n` 是字面量（無 BOM 編碼） |
| `[ERR] Target unreachable` | 目標位姿在機械臂工作空間外 | 先移動到更容易達到的位置，再放寬姿態約束 |
| 投遞後完全沒反應 | `is_busy` 卡住或 timer 停止 | 關閉 Figure，重新運行 `robotagent.m` |
| 動畫卡頓 | `drawnow limitrate` 丟幀 | 正常現象，降低 fps 或縮短軌跡點數 |
| `[PAUSE] IK unreachable at step X` | 笛卡爾軌跡中間點 IK 無解 | 見下方「PAUSE 狀態處理」|
| `[ERR] xxx (line N): ...` | 腳本執行出錯 | 查看 **LOG 區塊** 獲取詳細堆棧 |
| 沒看到 `========== LOG: xxx ==========` 區塊 | `logs/` 目錄無寫入權限，或 `diary` 被腳本內部關閉 | 檢查 `logs/` 目錄權限；確認生成腳本不含 `diary off` |
| `[Done]` 但軌跡完全錯誤（亂跳/不沿直線） | `planTrajectoryCartesian` 返回值順序寫錯，拿到 `T_traj` 而非 `q_traj` | 必須寫 `[~, q_traj, ~] = arm.planTrajectoryCartesian(...)` |
| `[Done]` 但只動了一步就停 | `planTrajectoryCartesian` 大量 `nan`，或腳本用了 `quinticTrajectory` 而非笛卡爾插值 | 確認使用 `[~, q_traj, ~] = planTrajectoryCartesian(...)` 並處理 nan |

## 文件目錄說明

| 目錄 | 用途 |
|------|------|
| `incoming/` | 運行時指令隊列。成功執行後刪除，失敗後歸檔到 `failed/` |
| `incoming/failed/` | 執行失敗的腳本歸檔 |
| `incoming_history/` | **所有執行過的腳本副本**（成功與失敗均保留），用於回溯與調試 |
| `logs/` | **每次腳本運行的完整日誌**（`log_時間戳.txt` 格式），含 `[RX]`、`[Done]`、`[ERR]`、`[PAUSE]` 等全部輸出 |

## 運行日誌機制（執行後必做）

> **每次腳本執行結束後，必須查看 Command Window 的 LOG 區塊。這是確認指令是否成功完成的唯一標準。**

MATLAB 會**自動**在 Command Window 打印如下區塊：

```
========== LOG: cmd_20260603_143052_123_move_to ==========
[RX] cmd_20260603_143052_123_move_to
[Done] move_to (500,0,800) (duration=5.0s)
================================
```

- **日誌來源**：`logs/cmd_xxx.txt`，由 `diary` 自動記錄該次運行的全部命令窗口輸出
- **強制查看**：`processIncomingCommands` 運行結束後自動 `type(log_path)`，用戶無需手動打開文件
- **失敗時**：LOG 區塊會包含 `[ERR] xxx (line N): ...` 與完整堆棧信息
- **暫停時**：LOG 區塊會包含 `[PAUSE] IK unreachable at step X/Y. current_q=... target_pos=...`

> **注意**：`logs/` 目錄已在 `.gitignore` 中，不會被提交到版本控制。

## PAUSE 狀態處理

當執行 **笛卡爾空間連續軌跡**（`move_to`、`relative_move`、`trajectory`）時，若中間某點 IK 無解：

1. MATLAB 會**播放至最後有效點後暫停**，不報錯、不崩潰
2. 在 MATLAB Command Window 輸出：
   ```
   [PAUSE] IK unreachable at step X/Y. current_q=[...] target_pos=[...]
   ```
3. **該輸出會被 `diary` 自動記錄到 `logs/log_xxx.txt`**，無需額外生成 pause 文件
4. **Kimi CLI 讀取該次 log 文件後會主動詢問用戶**：
   - **「繼續」** — 跳過不可達點，從最後有效位姿繼續（AI 生成新腳本）
   - **「調整目標」** — 修改目標位姿後重新生成
   - **「回退」** — 改用關節空間 `quinticTrajectory`，保證可達

> **注意**：`home` 指令始終使用 `quinticTrajectory`（關節空間），不會觸發 PAUSE。

## Figure 位姿矩陣顯示

`robotagent.m` 啟動後，Figure **右下角**會實時顯示末端 4×4 齊次變換矩陣：

```
T = [  R11    R12    R13      Px
       R21    R22    R23      Py
       R31    R32    R33      Pz
         0      0      0       1 ]
```

- 動畫播放時**逐幀更新**
- 單位：位置 mm，旋轉矩陣無量綱
- 若 Figure 被縮放過小，位姿文本可能被遮擋，可手動拉伸 Figure 窗口

## 最佳實踐

1. **先移動再轉姿態**：固定位置調姿態容易不可達。先改變位置（如下移 100-300mm），再調姿態成功率更高。

2. **姿態調整優先順序**：
   - 繞當前 X/Y/Z 軸旋轉（Rodrigues）可嚴格保持該軸不變
   - `planTrajectoryCartesian` 的 SLERP 插值**無法保持特定軸方向**（會走姿態空間最短路徑，中間點的 Y 軸會偏離）
   - **home 位置繞末端 X 軸 = 繞世界 Y 軸**：可讓 Z 軸從朝下轉到朝 ±X，但無法朝 ±Y

3. **文件生成編碼**：
   - ❌ `Out-File -Encoding utf8`（產生 BOM）
   - ✅ `System.IO.File.WriteAllText(path, content, [System.Text.UTF8Encoding]::new($false))`

4. **組合指令**：複雜動作拆成多個 `.m` 文件，按文件名排序（`cmd_01_xxx.m`、`cmd_02_xxx.m`）。`is_busy` 機制確保前一條完成後才執行下一條，形成自動隊列。

5. **IK 限制**：7R 機械臂固定 `θ1=0`，姿態自由度受限。若姿態不可達，可嘗試：
   - 改變位置後重試
   - 放寬姿態約束（允許其他軸微小變化）
   - 改用關節空間指令（`joint_move`）

## 開發與調試經驗（2026-06-03 更新）

### 經驗 1：絕對不要在 `processIncomingCommands` 中覆蓋 `fig.UserData.current_q`

**問題**：腳本執行後更新了 `fig.UserData.current_q = q_target`，但 `processIncomingCommands` 隨後執行：
```matlab
fig.UserData.current_q = evalin('base', 'current_q');
```
base workspace 中的 `current_q` 仍是舊值，導致正確更新被覆蓋。下一個指令讀取時以為機械臂還在零位。

**症狀**：`home` 看起來「一步到位」（因為 `quinticTrajectory(zeros, zeros)` 所有幀相同），`relative_move` 後接其他指令時位置重置。

**解決**：刪除該同步邏輯，讓腳本直接操作 `fig.UserData.current_q` 即可。

### 經驗 2：timer 回調中 `pause` 被忽略

**問題**：`processIncomingCommands` 在 timer `TimerFcn` 中執行腳本，腳本內的 `pause(1/30)` 在 timer 回調環境中被 MATLAB 忽略，導致 `for` 循環瞬間跑完，151 幀只刷新一次。

**症狀**：手動運行腳本動畫正常，投入 `incoming/` 自動執行時「一步到位」。

**解決**：所有腳本統一使用 `drawnow` + `tic/toc` busy-wait 替代 `drawnow limitrate` + `pause(1/30)`：
```matlab
for i = 1:size(q_traj, 1)
    updateRobotFigure(fig, q_traj(i, :));
    drawnow;
    t0 = tic;
    while toc(t0) < 1/30, end
end
```

### 經驗 3：`planTrajectoryCartesian` 與 `inverseKinematics` 的數值兼容性

**問題**：`forwardKinematics` 矩陣連乘產生浮點誤差，`inverseKinematics` 的解析解判斷 `< 0` 過於嚴格，導致 `planTrajectoryCartesian` 中間點大量返回 `nan`。

**症狀**：`planTrajectoryCartesian` 生成的軌跡第一步或中間點全部 `nan`，動畫無法播放。

**應對**（不改 `Arm7R.m` 的前提下）：
- `home` 指令不使用 `planTrajectoryCartesian`，直接用 `quinticTrajectory`
- 其他指令優先嘗試 `planTrajectoryCartesian`，若失敗則暫停並詢問用戶回退到 `quinticTrajectory`
- 未來如需嚴格笛卡爾直線，可在腳本中對旋轉矩陣做 SVD 正交化後再傳給 IK

### 經驗 5：`planTrajectoryCartesian` 返回值順序是最隱蔽的坑

**問題**：`planTrajectoryCartesian` 的回傳順序是 `[T_traj, q_traj, simin]`，但直覺上很容易寫成 `[q_traj, ~]`。結果拿到的是 4×4×steps 的位姿陣列，用 `q_traj(i,:)` 索引時得到的是 4×4 矩陣的行，軌跡完全錯誤。

**症狀**：`[Done]` 輸出正常，但機械臂亂跳、不沿直線、或動作完全不符合預期。

**正確寫法**：
```matlab
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
```

**記憶口訣**：「T 在前，q 在中，simin 在後」—— 位姿陣列、關節角、Simulink 輸入。

### 經驗 6：複合指令（姿態 + 位置）可用單一 `planTrajectoryCartesian`

**問題**：用戶需求常為「移動到某位置同時調整姿態」，直覺上可能拆成「先轉姿態再移動」兩步，或擔心 `planTrajectoryCartesian` 無法同時處理兩者。

**事實**：`planTrajectoryCartesian` 的 SLERP 插值本質就是**同時對位置線性插值 + 姿態球面插值**。只需構造一個同時包含目標位置與目標旋轉矩陣的 `T_target`，單次調用即可完成。

**成功範例**：home 位置 → 前移 100 mm + Z 軸轉水平（指向 +X）：
```matlab
T_target(1, 4) = T_target(1, 4) + 100;
T_target(1:3, 1:3) = [0, 0, 1; 1, 0, 0; 0, 1, 0];
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
```
結果：5 秒內平滑完成位置與姿態的同步過渡，無 PAUSE。

### 經驗 7：錯誤日誌與狀態文件是遠程調試的生命線

**問題**：用戶僅反饋「沒有動」或「一步到位」，無法定位是腳本錯誤、IK 失敗還是動畫刷新問題。

**解決**：
- `processIncomingCommands` 捕獲錯誤時輸出 `[ERR] 文件名:行號 — 錯誤信息`
- `[ERR]` 信息會被 `diary` 自動記錄到該次 `logs/log_xxx.txt`
- `[ERR]` 與 `[PAUSE]` 信息均由 `diary` 自動寫入該次 `logs/log_xxx.txt`，無需額外文件

這讓 Kimi CLI 端可以遠程讀取並精確診斷，無需用戶手動複製 MATLAB 命令行輸出。

## 關鍵數學

### 五次多項式軌跡
```matlab
q(t) = q0 + a3·t³ + a4·t⁴ + a5·t⁵
a3 = 10·(q1-q0)/T³,  a4 = -15·(q1-q0)/T⁴,  a5 = 6·(q1-q0)/T⁵
```

### 旋轉軸反轉（180°）
```matlab
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + 2*(K*K);  % 繞 n 軸旋轉 180°
```

---

*最後更新：2026年6月3日*
