# Robot-Agent 項目指南

## 項目概述

Robot-Agent 是一個基於 MATLAB 的 7 自由度（7R）機械臂可視化與自然語言控制系統。項目採用**文件隊列橋接架構**：Kimi CLI 根據用戶自然語言直接生成可執行的 `.m` 腳本，投遞到 `incoming/` 目錄；MATLAB 通過 `timer` 定期掃描並自動執行，實時驅動 3D Figure 動畫。

- **語言**: MATLAB（純基礎環境，無強制工具箱依賴）
- **通信協議**: 文件系統隊列（`incoming/` 目錄）
- **代碼與文檔語言**: 繁體中文
- **MATLAB 版本要求**: R2020b 或更高版本（需內置 `timer`）
- **當前版本**: v0.0.6（已加入障礙物可視化與基於 RRT 的避障規劃模塊）

> **架構演進說明**: 本項目早期使用 TCP 服務器架構，後經歷以下重大重構：
> - v0.0.1–v0.0.3: 文件監聽架構 + 自然語言解析層（`parseNaturalLanguage.m` / `generate_robot_cmd.m`）
> - v0.0.4: 整理 skill reference、修復 `processIncomingCommands` 作用域問題
> - v0.0.5: 移除中間解析層，改為 **AI 直接生成 `.m` 腳本**；新增統一測試入口；重組測試目錄
> - v0.0.6: 新增障礙物可視化與避障規劃（需 Robotics System Toolbox）
>
> 舊版 TCP 相關文件（`RobotAgent.m`、`run_robot_agent.m`、`send_cmd.m`、`send_robot_cmd.ps1`、`send_via_tcp.py` 等）已不再使用，部分被 `.gitignore` 排除。但 `docs/ROBOT_AGENT_README.md` 與 `docs/robot_agent_cmds.json` 仍作為歷史參考保留在倉庫中。

## 文件結構

```
robot-agent/
├── robotagent.m                  % 一鍵啟動腳本（初始化 Figure + 啟動文件監聽 timer）
├── README.md                     % 項目總覽（Quick Start、架構圖、Changelog）
├── AGENTS.md                     % 本文件
├── .gitignore                    % 排除 incoming/、logs/、測試輸出 PNG、MATLAB 自動備份等
├── src/                          % 源代碼
│   ├── Arm7R.m                   % 7R 機械臂運動學類（FK / IK / 軌跡規劃 / 雅可比）
│   ├── initRobotFigure.m         % Figure 初始化（含障礙物紅球、末端位姿文字）
│   ├── updateRobotFigure.m       % Figure 高效更新（set + drawnow limitrate）
│   ├── animateRobot.m            % 動畫播放循環
│   ├── quinticTrajectory.m       % 五次多項式關節空間插值
│   ├── processIncomingCommands.m % 文件監聽執行器（timer 回調）
│   ├── buildRobotTree.m          % 將 Arm7R DH 轉換為 rigidBodyTree
│   ├── checkRobotObstacleCollision.m  % 機械臂與球形障礙物碰撞檢測
│   ├── planTrajectoryWithObstacle.m   % RRT 避障軌跡規劃
│   └── computeTrajectory.m       % 舊架構遺留的獨立軌跡計算函數（當前未使用）
│
├── tests/                        % 測試腳本
│   ├── phases/                   % 分 Phase 測試
│   │   ├── run_all_tests.m       % 統一測試入口（僅運行 6 個 phase 測試）
│   │   ├── test_phase1_figure.m  % Figure 初始化、更新、動畫
│   │   ├── test_phase2_filewatch.m    % 文件監聽、執行順序、錯誤歸檔、並發保護
│   │   ├── test_phase3_generator.m    % Quintic 軌跡數學性質
│   │   ├── test_phase4_cleanup.m      % 舊架構清理與文檔一致性
│   │   ├── test_phase5_e2e.m          % 端到端集成（home / move_to / relative / joint / queue）
│   │   ├── test_phase6_complex.m      % 多段複合指令鏈路
│   │   └── output/                    % 測試截圖（實際由腳本寫入 tests/output/）
│   └── obstacle/                 % 避障模塊測試
│       ├── test_build_robot_tree.m
│       ├── test_obstacle_visualization.m
│       ├── test_obstacle_collision.m
│       ├── test_obstacle_avoidance_move.m
│       └── test_obstacle_disabled.m
│
├── scripts/                      % 歷史遺留目錄，目前為空
│
├── skills/
│   └── robotagent-ops/           % Kimi CLI 項目 skill
│       ├── SKILL.md              % 操作指南與快速導航
│       └── references/           % 架構、模板、姿態調整、函數簽名、排查表
│           ├── architecture-reference.md
│           ├── function-signatures-reference.md
│           ├── pose-adjustment-reference.md
│           ├── script-templates-reference.md
│           └── troubleshooting-reference.md
│
├── docs/                         % 文檔
│   ├── README_Arm7R.md           % Arm7R API 與數學原理
│   ├── ROBOT_AGENT_README.md     % 舊版 TCP 架構文檔（僅供參考）
│   ├── plan_refactor_filewatch.md % 架構重構計劃
│   ├── robot_agent_cmds.json     % 舊版 TCP 協議 JSON（僅供參考）
│   ├── plan_obstacle_avoidance_v0.0.6.md
│   ├── plan1.md                  % 歷史實施計劃
│   ├── plan2.md
│   ├── plan3.md
│   └── test_refactor_plan.md
│
├── incoming/                     % 運行時指令隊列（.gitignore 排除）
│   └── failed/                   % 執行失敗的腳本歸檔
├── incoming_history/             % 每次執行腳本備份（.gitignore 排除）
└── logs/                         % diary 日誌輸出（.gitignore 排除）
```

> **注意**: 本項目為純 MATLAB 腳本集合，**不存在** `package.json`、`pyproject.toml`、`Cargo.toml`、`Makefile`、`CMakeLists.txt` 等傳統構建配置文件。

## 技術棧與依賴

### 核心運行時（無需額外工具箱）
- **MATLAB R2020b+**（`timer` 為基礎環境函數）
- 所有運動學計算、繪圖、文件 I/O 均使用純 MATLAB 實現。

### 可選工具箱
| 工具箱 | 用途 | 缺失時行為 |
|--------|------|-----------|
| **Robotics Toolbox** (Peter Corke) | `Arm7R.plot()`、`teach()`、解析法 `jacobian()` | `jacobian()` 回退到數值微分；`plot()`/`teach()` 報錯提示安裝 |
| **Robotics System Toolbox** (MathWorks) | `rotm2quat` / `quat2rotm` / `quatinterp`（SLERP 姿態插值）、`rigidBodyTree` / `checkCollision` / `manipulatorRRT`（避障模塊） | 笛卡爾軌跡規劃的姿態插值降級為軸角線性插值；避障規劃功能不可用 |

工具箱在 `Arm7R` 構造函數中通過 `try-catch` 自動檢測，分別存儲於 `arm.hasRoboticsToolbox` 與 `arm.hasRoboticsSystemToolbox`。

## 代碼組織與模塊劃分

### 1. Arm7R（運動學核心）
`Arm7R` 是 `handle` 類，封裝 7R 機械臂全部運動學功能：

| 方法 | 說明 |
|------|------|
| `forwardKinematics(q)` | 正向運動學，返回 4×4 齊次變換矩陣 |
| `inverseKinematics(T)` | 逆向運動學，解析閉式解；固定 θ1=0 消除冗餘；返回 `[q, err]` |
| `planTrajectoryCartesian(T_start, T_end, steps, t_start, t_end)` | 笛卡爾空間軌跡規劃（位置 Lerp + 姿態 SLERP/軸角插值） |
| `planTrajectoryJoint(q_start, q_end, t_start, t_end)` | 關節空間線性插值，輸出 Simulink 可用的 `simin` 格式 |
| `jacobian(q)` | 雅可比矩陣；有 Robotics Toolbox 時用 `jacob0`，否則數值微分 |
| `conditionNumber(q)` | 雅可比條件數，評估奇異點接近程度 |
| `simplePlot(q)` | 無依賴的 3D 骨架繪圖 |
| `getJointPositions(q)` | 計算各關節在世界坐標系中的位置（9×3） |
| `displayDHTable()` | 打印標準 DH 參數表 |
| `getEndEffectorPose(q)` | 同 `forwardKinematics`，提供更直觀的函數名 |
| `setJointLimits(lower, upper)` | 設置關節角度限制 |

### 2. 文件隊列橋接架構

```
┌─────────────────┐         ┌─────────────────────────────┐
│   MATLAB 窗口    │         │      Kimi CLI 窗口          │
│  ┌───────────┐  │         │  ┌─────────────────────┐    │
│  │ 點擊運行   │  │         │  │ 用戶輸入自然語言    │    │
│  │robotagent │  │         │  │  • 讀取 project skill│   │
│  │→ Figure   │  │         │  │  • 生成 .m 腳本      │    │
│  │→ timer    │  │         │  │  • 寫入 incoming/   │    │
│  └───────────┘  │         │  └──────────┬──────────┘    │
│         │        │         │             │ 寫文件         │
│         │ 0.5s   │         │             ▼               │
│         ▼        │         │  ┌─────────────────────┐    │
│  ┌───────────┐  │         │  │  cmd_xxx.m          │    │
│  │掃描incoming│◄─┼─────────┼──┤  (完整軌跡 + 動畫)  │    │
│  │→ run()    │  │ 文件系統 │  └─────────────────────┘    │
│  └───────────┘  │         └─────────────────────────────┘
│         │        │
│         ▼        │
│  ┌───────────┐  │
│  │ Figure 更新│  │
│  └───────────┘  │
└─────────────────┘
```

- **啟動層**: `robotagent.m` 一鍵啟動，自動添加 `src` 路徑、初始化 Figure、啟動 `timer`
- **監聽層**: `timer` 每 0.5 秒掃描 `incoming/` 目錄，發現 `cmd_*.m` 即執行
- **執行層**: `processIncomingCommands.m` 按文件創建時間排序執行，使用 `diary` 記錄輸出到 `logs/log_*.txt`

`processIncomingCommands` 的運行細節：
1. 檢查 `fig.UserData.is_busy`，若為 `true` 則直接返回
2. 使用 `dir(fullfile(incoming_dir, 'cmd_*.m'))` 查找指令文件，按 `datenum` 排序
3. 啟動 `diary` 捕獲命令窗口輸出
4. 將 `fig.UserData.arm`、`fig.UserData.current_q` 注入函數工作空間，連同已有的 `fig` 一起供腳本使用
5. 調用 `run(cmd_path)` 執行腳本
6. 執行成功：備份到 `incoming_history/`，刪除原文件
7. 執行失敗：備份到 `incoming_history/`，移至 `incoming/failed/`，輸出 `[ERR]` 並打印堆棧
8. 結束時使用 `type` 打印日誌內容

### 3. 可視化層
- `initRobotFigure.m`: 創建白色 Figure，繪製連桿、關節、Base/末端坐標軸、障礙物紅球、左上角位姿文字；所有圖形句柄存於 `fig.UserData`
- `updateRobotFigure.m`: 通過 `set` 更新圖形對象屬性，使用 `drawnow limitrate` 保證效率
- `animateRobot.m`: 遍歷軌跡並調用 `updateRobotFigure`，默認 30 fps；結束後更新 `fig.UserData.current_q`

> **動畫幀率注意事項**: `animateRobot` 內部使用 `pause(1/fps)` 控制播放速度。但在 `timer` 回調執行鏈中，`pause` 會被 MATLAB 忽略，導致動畫「快進」——功能不受影響（機械臂會正確到達目標），只是沒有平滑動畫效果。如需真實時間播放，需在腳本中自行遍歷 `q_traj` 並使用 `tic/toc` busy-wait。

### 4. 避障模塊（v0.0.6 新增）
- `buildRobotTree.m`: 將 `Arm7R` DH 參數轉換為 14-body 的 `rigidBodyTree`（7 個 dummy fixed body + 7 個 revolute body），確保 FK 與 `Arm7R.forwardKinematics` 完全一致；為每個非固定 body 原點添加 `collisionSphere(20)` 作為簡化碰撞幾何
- `checkRobotObstacleCollision.m`: 使用 `checkCollision` 檢測機械臂與球形障礙物碰撞，返回 `[is_collision, min_dist]`
- `planTrajectoryWithObstacle.m`: 先檢查原始笛卡爾軌跡是否碰撞；若碰撞則調用 `manipulatorRRT` 規劃避障路徑，再將路點分段轉換為 Quintic 軌跡並驗證；返回 `[q_traj, avoided, info]`

## 啟動與運行

### 一鍵啟動（MATLAB GUI）

在 MATLAB 的 **Current Folder** 中找到 `robot-agent` 文件夾，雙擊 `robotagent.m` 或在 Editor 中點擊 **運行** 按鈕 ▶：

```
========================================
RobotAgent started.
Figure: RobotAgent - 7R Arm
Watching: D:\...\robot-agent\incoming
Obstacle avoidance: enabled / disabled
========================================
```

Figure 彈出，MATLAB 命令行不阻塞。

### 啟動參數

在 `robotagent.m` 中可手動調整：

```matlab
enable_obstacle_avoidance = true;   % 是否啟用避障規劃（需 Robotics System Toolbox）
obstacle = struct('center', [400; 0; 500], ...   % 障礙物中心（mm）
                  'radius', 100, ...              % 障礙物半徑（mm）
                  'safety_margin', 50, ...        % 預留安全邊距字段
                  'enabled', true);               % 是否可視化紅球
```

可視化開關與避障規劃開關相互獨立：
- `fig.UserData.obstacle_enabled` — 控制紅球顯示/隱藏
- `fig.UserData.obstacle_avoidance_enabled` — 控制是否使用避障規劃器

`robotagent.m` 啟動時會檢測是否安裝 Robotics System Toolbox；若未安裝而 `enable_obstacle_avoidance = true`，則發出 warning 並禁用避障規劃。

### 發送指令

在 Kimi CLI 中輸入自然語言，例如：

```
讓機械臂末端移動到 (500, 0, 800)，用 3 秒
```

Kimi CLI 內部處理流程：
1. 讀取 `skills/robotagent-ops/SKILL.md` 與 references
2. 直接生成包含完整軌跡邏輯的 `.m` 腳本
3. 寫入 `robot-agent/incoming/cmd_yyyymmdd_HHMMSS_FFF_cmdname.m`
4. MATLAB `timer` 檢測並自動執行
5. 執行結果寫入 `logs/log_*.txt`，Kimi CLI 主動讀取最新 log 並報告 `[Done]` / `[ERR]` / `[PAUSE]`

## 開發規範

### 單位約定
- **長度**: 毫米 (mm)
- **角度**: 弧度 (rad)
- **時間**: 秒 (s)

### DH 參數約定
使用**標準 DH 參數**（Standard DH），參數向量 `P_DH = [e, k, i, l, m, n, j, b]`。
默認值: `[149.438, 147.9, 0, 458.09, 93.5, 360.71, 118.27, 272.42]`

### 代碼風格
- 類與方法使用中文注釋，遵循 MATLAB Help Text 格式
- 方法間使用 `%%` 分隔，便於 MATLAB 分節導航
- 輸入輸出參數在注釋中明確標註維度與單位
- 工具箱缺失時優先降級（fallback）而非強制報錯
- 錯誤處理使用 `try-catch`，執行失敗的腳本移至 `incoming/failed/`

### 生成腳本約定
投遞到 `incoming/` 的 `.m` 腳本運行在 `processIncomingCommands` 的函數工作空間，可直接使用局部變量 `arm`、`fig`、`current_q`。常見約定：
- 腳本開頭讀取 `fig.UserData` 以獲取最新狀態：
  ```matlab
  ud = fig.UserData;
  arm = ud.arm;
  current_q = ud.current_q;
  ```
- 成功結尾輸出 `fprintf('[Done] ...\n');`
- IK 不可達時輸出 `fprintf('[ERR] ...\n');` 並保持原姿態
- 軌跡中斷時輸出 `fprintf('[PAUSE] ...\n');` 並更新 `fig.UserData.current_q` 到最後有效幀
- 使用 `planTrajectoryCartesian` 時必須處理 `NaN` 行（詳見 `skills/robotagent-ops/references/script-templates-reference.md`）
- 障礙物相關腳本應動態讀取 `ud.obstacle.center`，不要硬編碼障礙物位置

### 重要函數簽名

```matlab
q_traj = quinticTrajectory(q0, q1, T, fps);          % T 為總時間，不是步數
animateRobot(fig, q_traj, fps);                       % 第二參數為軌跡，不是 arm
[T_traj, q_traj, simin] = arm.planTrajectoryCartesian(T_start, T_end, steps, t_start, t_end);
[q_traj, avoided, info] = planTrajectoryWithObstacle(arm, current_q, T_target, obstacle, duration, robot_tree);
T = arm.forwardKinematics(q);                         % q 為 1x7，T 為 4x4
[q, err] = arm.inverseKinematics(T);                  % err=0 成功，err=1 無解
points = arm.getJointPositions(q);                    % points 為 9x3
```

## 構建與測試

### 構建

無需編譯或構建步驟。直接在 MATLAB 中運行 `robotagent.m` 即可。

### 運行全部 Phase 測試

`tests/phases/run_all_tests.m` 會依次運行 6 個 phase 測試（不包含 obstacle 測試）：

```matlab
cd('D:\Document\code\Matlab\robot-agent\tests\phases');
addpath('../../src');
run_all_tests;
```

或從 PowerShell/CMD 批量運行：

```powershell
matlab -batch "cd('D:\Document\code\Matlab\robot-agent\tests\phases'); addpath('../../src'); run_all_tests;"
```

> **注意**: `run_all_tests.m` 與各個 `test_phase*.m` 對工作目錄的處理不一致。`run_all_tests.m` 會切換到 `tests\phases`，而各個 phase 測試腳本會切換到 `tests\` 並將截圖寫入 `tests\output\`。實際運行時通常不影響結果，但截圖輸出路徑為 `tests/output/` 而非 `tests/phases/output/`。

### 運行單個 Phase 測試

```matlab
cd('D:\Document\code\Matlab\robot-agent\tests');
addpath('../src');

test_phase1_figure;
test_phase2_filewatch;
test_phase3_generator;
test_phase4_cleanup;
test_phase5_e2e;
test_phase6_complex;
```

各個 phase 測試腳本內部會自動切換到 `D:\Document\code\Matlab\robot-agent\tests` 並將截圖保存到 `tests/output/`。

### 運行避障測試

```matlab
cd('D:\Document\code\Matlab\robot-agent');
addpath('src');
addpath('tests/obstacle');

test_build_robot_tree;
test_obstacle_visualization;
test_obstacle_collision;
test_obstacle_avoidance_move;
test_obstacle_disabled;
```

> 避障測試需要 **Robotics System Toolbox**，否則會因 `rigidBodyTree`、`checkCollision`、`manipulatorRRT` 不可用而失敗。

### 測試策略

| 測試文件 | 覆蓋範圍 |
|----------|----------|
| `test_phase1_figure.m` | Figure 初始化、句柄有效性、關閉後重建、視覺截圖、動畫播放 |
| `test_phase2_filewatch.m` | 文件監聽、指令執行順序、錯誤處理、`is_busy` 並發保護 |
| `test_phase3_generator.m` | Quintic 軌跡數學性質（起終點、速度、對稱性） |
| `test_phase4_cleanup.m` | 舊架構清理驗證、新架構一致性、文檔更新檢查 |
| `test_phase5_e2e.m` | home、move_to（可達/不可達）、relative_move、joint_move、多段隊列、PAUSE 機制 |
| `test_phase6_complex.m` | 複合多段指令鏈路（姿態調整 + 相對移動 + 畫圓 + 回零） |
| `test_obstacle_*.m` | rigidBodyTree FK 一致性、碰撞檢測、RRT 避障、可視化開關 |

視覺測試會自動將 Figure 截圖保存到 `tests/output/`，供人工審查。

## 安全與穩定性注意事項

1. **冗餘處理**: 7R 機械臂有 1 個冗餘自由度，逆運動學固定 `θ1 = 0` 以獲得唯一閉式解。若需優化 θ1，需在外部層疊加優化器。
2. **Figure 容錯**: `updateRobotFigure` 每次更新前檢查 `isvalid(fig)`，若用戶誤關窗口則報錯提示重新運行 `robotagent`。
3. **IK 無解**: 不可達位姿返回 `err = 1`，生成的腳本應捕獲錯誤並輸出提示，機械臂保持原位不動。
4. **指令隊列**: `incoming/` 目錄中的文件按創建時間排序執行；`is_busy` 標誌防止動畫期間並發執行新指令。
5. **錯誤腳本**: 執行失敗的 `.m` 文件自動移至 `incoming/failed/`，同時備份到 `incoming_history/`，避免阻塞後續指令。
6. **日誌機制**: `processIncomingCommands` 使用 `diary` 捕獲命令窗口輸出，包括 `fprintf`、`warning`、`error` 信息。
7. **精度與降級**: 缺少 Robotics System Toolbox 時，笛卡爾軌跡規劃的姿態插值降級為軸角線性插值，路徑可能非最優（非恆定角速度）。
8. **文件編碼**: 生成 `.m` 腳本時應避免 UTF-8 BOM，推薦使用無 BOM 的 UTF-8 編碼。
9. **避障模塊限制**: `buildRobotTree` 目前使用關節點球體近似碰撞幾何，未覆蓋連桿本體；RRT 規劃要求起終點均無碰撞，否則會 PAUSE。
10. **變量作用域**: `run()` 在函數工作空間執行，腳本中應通過 `fig.UserData.arm` / `fig.UserData.current_q` 獲取狀態，不要依賴 `base` 工作空間。
11. **動畫幀率限制**: 在 `timer` 回調中執行的腳本若調用 `animateRobot`，其內部 `pause(1/fps)` 會被忽略，動畫會快進完成。

## 部署與發布

本項目無傳統部署流程。使用方式為：
1. 將項目文件夾複製到目標機器
2. 在 MATLAB 中設置 `robot-agent` 為 Current Folder
3. 運行 `robotagent.m`
4. 通過 Kimi CLI 或手動向 `incoming/` 投放 `.m` 腳本進行控制

如需在無 GUI 的 MATLAB 會話中運行測試，可使用 `-batch` 模式（見上文）。

## 擴展預留

- 渲染層與運動學層邏輯解耦。未來若遷移為前後端分離，只需將 `updateRobotFigure` 替換為「通過 TCP 發送關節角序列到前端」，核心運動學代碼無需改動。
- 未來如需雙臂協同，可擴展持有兩個 `Arm7R` 對象，指令中添加 `"arm": "left"` / `"right"` 字段。
- 避障模塊可進一步替換為更精確的連桿圓柱體碰撞幾何。
