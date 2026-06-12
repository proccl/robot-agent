# RobotAgent 系統架構與運行機制

## 架構概述

RobotAgent 採用 **文件監聽架構**（非 TCP）：

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Kimi CLI 窗口                                  │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │   Agent（Kimi）                                                      │   │
│  │  • 接收用戶自然語言 → 讀取 Skill 解析意圖 → 生成 .m 代碼 → 寫入      │   │
│  │  • 執行結束後：讀取 Log → 分析 [Done]/[ERR]/[PAUSE] → 向用戶匯報   │   │
│  └─────────────────────────────────┬───────────────────────────────────┘   │
└────────────────────────────────────┼────────────────────────────────────────┘
                                     │ 讀取 Skill
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           文 件 系 統 （中間層）                             │
│                                                                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐  │
│  │  incoming/      │  │  logs/          │  │  incoming_history/          │  │
│  │  (指令隊列)     │  │  (執行日誌)     │  │  (腳本歷史歸檔)             │  │
│  │  └── failed/    │  │  └── log_*.txt  │  │                             │  │
│  │      (失敗歸檔) │  │                 │  │                             │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────────┘  │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │   Skill（robotagent-ops）                                           │   │
│  │  • SKILL.md（入口速查）                                             │   │
│  │  ├── references/architecture-reference.md（架構與運行機制）         │   │
│  │  ├── references/script-templates-reference.md（腳本模板）           │   │
│  │  ├── references/pose-adjustment-reference.md（姿態調整算法）        │   │
│  │  ├── references/troubleshooting-reference.md（問題排查與經驗）      │   │
│  │  └── references/function-signatures-reference.md（函數簽名確認）    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└────────────────────────────────────┬────────────────────────────────────────┘
                                     │ 寫入 cmd_xxx.m    讀取 log_*.txt
                                     ▼
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
  - `quinticTrajectory.m`：五次多項式關節空間軌跡
  - `buildRobotTree.m`：將 `Arm7R` 轉換為 `rigidBodyTree`（避障用）
  - `checkRobotObstacleCollision.m`：檢測機械臂與球形障礙物碰撞
  - `planTrajectoryWithObstacle.m`：直線軌跡碰撞時調用 RRT 繞行

## 文件目錄說明

| 目錄 | 用途 |
|------|------|
| `incoming/` | 運行時指令隊列。成功執行後刪除，失敗後歸檔到 `failed/` |
| `incoming/failed/` | 執行失敗的腳本歸檔 |
| `incoming_history/` | **所有執行過的腳本副本**（成功與失敗均保留），用於回溯與調試 |
| `logs/` | **每次腳本運行的完整日誌**（`log_時間戳.txt` 格式），含 `[RX]`、`[Done]`、`[ERR]`、`[PAUSE]` 等全部輸出 |

> **注意**：`logs/` 目錄已在 `.gitignore` 中，不會被提交到版本控制。

## 運行日誌機制

> **每次腳本執行結束後，必須查看 Command Window 的 LOG 區塊。這是確認指令是否成功完成的唯一標準。**

MATLAB 會**自動**在 Command Window 打印如下區塊：

```
========== LOG: cmd_20260603_143052_123_move_to ==========
[RX] cmd_20260603_143052_123_move_to
[Done] move_to (500,0,800) (duration=5.0s)
================================
```

- **日誌來源**：`logs/log_*.txt`，由 `diary` 自動記錄該次運行的全部命令窗口輸出
- **強制查看**：`processIncomingCommands` 運行結束後自動 `type(log_path)`，用戶無需手動打開文件
- **失敗時**：LOG 區塊會包含 `[ERR] xxx (line N): ...` 與完整堆棧信息
- **暫停時**：LOG 區塊會包含 `[PAUSE] IK unreachable at step X/Y. current_q=... target_pos=...`

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

## 障礙物與避障開關

### 可視化開關

Figure 中可顯示一個紅色半透明障礙球：

```matlab
fig.UserData.obstacle_enabled = true;   % 顯示紅球
fig.UserData.obstacle_enabled = false;  % 隱藏紅球
```

球的幾何參數存儲在 `fig.UserData.obstacle`：

```matlab
center = fig.UserData.obstacle.center;   % 3×1，單位 mm
radius = fig.UserData.obstacle.radius;   % 單位 mm
```

### 避障規劃開關

`obstacle_enabled` 只控制**顯示**，不控制規劃。避障規劃由另一個字段控制：

```matlab
fig.UserData.obstacle_avoidance_enabled = true;   % 啟用 RRT 避障
fig.UserData.obstacle_avoidance_enabled = false;  % 使用普通笛卡爾軌跡
```

**啟用條件**：
1. `robotagent.m` 啟動前將 `enable_obstacle_avoidance = true`
2. MATLAB 必須安裝 **Robotics System Toolbox**
3. 工具箱檢測通過（見下方「工具箱檢測」）

滿足後，`robotagent.m` 會調用 `buildRobotTree(arm)` 構建 `fig.UserData.robot_tree`，並設置 `obstacle_avoidance_enabled = true`。

### 工具箱檢測

`ver('Robotics System Toolbox')` 在某些 MATLAB 安裝上會返回空，導致避障被錯誤禁用。正確做法：

```matlab
v = ver;
has_robotics_toolbox = any(strcmpi({v.Name}, 'Robotics System Toolbox'));
```

### 目標點在障礙物內部

`manipulatorRRT` 要求起點和終點都處於無碰撞狀態。若目標在紅球內部，會報錯：

```text
The goal configuration of the robot is in world collision.
```

解決：將目標設在球外（如表面外 10~50 mm），或臨時關閉避障。

---

## Figure 位姿矩陣顯示

`robotagent.m` 啟動後，Figure **左上角**會實時顯示末端 4×4 齊次變換矩陣（透明背景，不遮擋圖形）：

```
T = [  R11    R12    R13      Px
       R21    R22    R23      Py
       R31    R32    R33      Pz
         0      0      0       1 ]
```

- 動畫播放時**逐幀更新**
- 單位：位置 mm，旋轉矩陣無量綱
- 若 Figure 被縮放過小，位姿文本可能被遮擋，可手動拉伸 Figure 窗口
- 實現方式：`annotation('textbox', 'BackgroundColor','none', 'EdgeColor','none')` + normalized units

## 多段指令排隊機制

`processIncomingCommands` 內置了天然的**單線程隊列**：

```matlab
if fig.UserData.is_busy, return; end     % 動畫期間拒絕新指令
[~, idx] = sort([files.datenum]);        % 按文件創建時間排序
cmd_path = fullfile(incoming_dir, files(idx(1)).name);  % 只取最早一個
fig.UserData.is_busy = true;             % 加鎖
run(cmd_path);                           % 執行
fig.UserData.is_busy = false;            % 解鎖
```

**執行流程**（以 4 段指令為例）：

```
t=0s   寫入 cmd_01_pose_yz.m  →  創建時間 t1
t=1s   寫入 cmd_02_move_xz.m  →  創建時間 t2
t=2s   寫入 cmd_03_circle_z.m →  創建時間 t3
t=3s   寫入 cmd_04_home.m     →  創建時間 t4

timer 0.5s 觸發：
  第 1 次（t=0.5s）：發現 cmd_01，is_busy=false → 執行 cmd_01，is_busy=true
  第 2 次（t=1.0s）：is_busy=true → 跳過
  ...
  第 N 次（t≈5s）：cmd_01 完成，is_busy=false → 執行 cmd_02（datenum 最早）
  ...
  依此類推，直到 cmd_04 完成
```

**關鍵點**：
- 文件名前綴 `cmd_01_`、`cmd_02_` **僅供人類識別**，真正決定順序的是文件創建時間 `datenum`
- 只要按順序寫入文件（先寫 cmd_01，再寫 cmd_02），就會按順序執行
- `is_busy` 標誌確保動畫期間不會並發執行新指令，避免狀態混亂
- 執行失敗的腳本會被移到 `incoming/failed/`，不會阻塞後續指令

## 變量注入機制

`processIncomingCommands` 將關鍵變量注入**函數工作空間**（非 base workspace）：

```matlab
arm = fig.UserData.arm;           % Arm7R 對象
current_q = fig.UserData.current_q; % 當前關節角 1×7
% fig 已經是函數參數，直接可用
```

> ⚠️ `run()` 在函數工作空間執行腳本。`assignin('base', ...)` 對腳本無效。
