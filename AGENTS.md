<!-- AGENTS.md for robot-agent -->
# Robot-Agent 項目指南

## 項目概述

這是一個基於 MATLAB 的 7 自由度（7R）機械臂可視化與自然語言控制系統。項目採用**文件隊列橋接模式**實現跨進程通信：Kimi CLI 端解析自然語言生成 `.m` 腳本，投遞到 `incoming/` 目錄；MATLAB 端通過 `timer` 每 0.5 秒掃描並自動執行，驅動 Figure 中的機械臂動畫。

- **語言**: MATLAB（純基礎環境，無強制工具箱依賴）
- **通信協議**: 文件系統隊列（`incoming/` 目錄）
- **代碼與文檔語言**: 繁體中文
- **MATLAB 版本要求**: R2020b 或更高版本（需內置 `timer`）

> **架構演進說明**: 本項目早期版本使用 TCP 服務器架構，現已完全重構為文件隊列架構。舊版 TCP 相關文件（`RobotAgent.m`、`run_robot_agent.m`、`send_robot_cmd.ps1`、`send_cmd.m` 等）均已移除。遺留文件 `send_via_tcp.py` 與 `computeTrajectory.m` 僅供參考；`test_integration.m` 為舊架構遺留測試，運行會失敗。

### 系統架構圖

```matlab
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
│  │ 用戶點擊運行           │    │ processIncomingCommands (timer)          │  │
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

## 文件結構

```
robot-agent/
├── robotagent.m                  % 一鍵啟動腳本（初始化 Figure + 啟動文件監聽 timer）
├── send_via_tcp.py               % 舊版 TCP 發送腳本（已廢棄，保留供參考）
├── skills/                       % 項目輔助 skill（Kimi CLI 使用）
│   └── robotagent-ops/
│       └── SKILL.md              % RobotAgent 操作指南（自然語言指令、姿態調整、問題排查、Log 分析規範）
├── src/                          % 源代碼
│   ├── Arm7R.m                   % 7R 機械臂運動學類（FK / IK / 軌跡規劃 / 雅可比 / 條件數 / simplePlot）
│   ├── initRobotFigure.m         % Figure 初始化函數（白色背景、存儲句柄到 UserData、末端位姿矩陣顯示）
│   ├── updateRobotFigure.m       % Figure 高效更新函數（set + drawnow limitrate + 位姿文本更新）
│   ├── animateRobot.m            % 動畫播放函數（循環調用 updateRobotFigure）
│   ├── quinticTrajectory.m       % 五次多項式關節空間軌跡規劃
│   ├── generate_robot_cmd.m      % 代碼生成器（支持 home/move_to/joint_move/trajectory/relative_move/get_status）
│   ├── parseNaturalLanguage.m    % 自然語言解析器（中英混合輸入）
│   ├── processIncomingCommands.m % 文件監聽執行器（timer 回調 + diary 日誌 + 歷史歸檔）
│   └── computeTrajectory.m       % 獨立軌跡計算函數（舊架構遺留，供參考）
│
├── tests/                        % 測試腳本（按 Phase 組織）
│   ├── test_phase1_figure.m      % Phase 1: Figure 初始化、UserData 完整性、updateRobotFigure 更新、句柄有效性、關閉重建、animateRobot 時序與末幀位姿驗證、視覺截圖
│   ├── test_phase2_filewatch.m   % Phase 2: 目錄自動創建、processIncomingCommands 單文件執行、多文件按 datenum 排序執行、錯誤腳本移至 failed/、is_busy 並發阻止、空目錄無異常
│   ├── test_phase2_tcp.m         % Phase 2 (TCP 遺留): TCP 相關測試（舊架構，可能失效）
│   ├── test_phase3_generator.m   % Phase 3: Quintic 軌跡數學性質（起終點精度、速度趨零、對稱性）、代碼生成器、各指令模板可執行性、自定義 duration、文件名唯一性
│   ├── test_phase3_render.m      % Phase 3 (渲染): 渲染層測試
│   ├── test_phase4_compute.m     % Phase 4 (計算): 運動學計算測試
│   ├── test_phase4_nlp.m         % Phase 4: 自然語言解析正確性（home/move_to/joint_move/circle/line/relative_move/status）、無法識別輸入報錯、端到端 NLP→文件→執行
│   ├── test_phase5_cleanup.m     % Phase 5: 舊文件清理驗證、新架構一致性、README 與 AGENTS.md 內容檢查
│   ├── test_phase5_commands.m    % Phase 5 (指令): 指令集測試
│   ├── test_phase5_integration.m % Phase 5 (集成): 集成測試
│   ├── test_phase6_integration.m % Phase 6: 端到端全鏈路測試、快速連續指令、is_busy 排隊、無效位姿容錯、Quintic 軌跡平滑性、性能基準
│   ├── test_phase6_startup.m     % Phase 6 (啟動): 啟動流程測試
│   ├── test_phase7_docs.m        % Phase 7: 文檔一致性測試
│   ├── test_integration.m        % 綜合集成測試（舊 TCP 架構遺留，運行會失敗）
│   └── output/                   % 測試輸出的 PNG 截圖與狀態文件
│
├── incoming/                     % 運行時指令隊列目錄（.gitignore 排除）
│   └── failed/                   % 執行失敗的腳本歸檔
│
├── incoming_history/             % 所有執行過的腳本歷史歸檔（成功與失敗均複製到此，.gitignore 排除）
│
├── incoming_test_relative/       % 相對移動測試用的指令目錄
│
├── logs/                         % 每次腳本運行的完整日誌（.gitignore 排除）
│                                   % [ERR] 與 [PAUSE] 均由 diary 自動記錄到 log_*.txt
│
├── docs/                         % 文檔
│   ├── README_Arm7R.md           % Arm7R API 文檔與數學原理詳解（DH 參數、FK/IK 推導、SLERP）
│   ├── ROBOT_AGENT_README.md     % 舊版 RobotAgent 使用說明（僅供參考）
│   ├── robot_agent_cmds.json     % 指令協議 JSON Schema（舊版 TCP 協議，僅供參考）
│   ├── plan_refactor_filewatch.md % 架構重構計劃文檔（含 Quintic 多項式推導）
│   ├── plan1.md / plan2.md       % 系統實施計劃與架構設計文檔
│   └── plan3.md                  % 軌跡執行失敗問題總結與改進計劃（pause 機制、歷史歸檔、Figure 位姿顯示）
│
├── README.md                     % 項目總覽（Quick Start、架構圖、支持的指令）
└── AGENTS.md                     % 本文件
```

> **注意**: 本項目**沒有**傳統的配置文件（如 `pyproject.toml`、`package.json`、`Cargo.toml`、`setup.py`、`Makefile` 等），因為這是一個純 MATLAB 腳本項目，無需編譯或包管理。MATLAB 路徑通過 `robotagent.m` 中的 `addpath` 或手動 `addpath('src')` 管理。

## 技術棧與依賴

### 核心運行時（無需額外工具箱）
- **MATLAB R2020b+**（`timer` 為基礎環境函數）
- 所有運動學計算、繪圖、文件 I/O 均使用純 MATLAB 實現。

### 可選工具箱
| 工具箱 | 用途 | 缺失時行為 |
|--------|------|-----------|
| **Robotics Toolbox** (Peter Corke) | `Arm7R.plot()`、`teach()`、`jacobian()`（解析法） | `jacobian()` 回退到數值微分（delta=1e-6）；`plot()`/`teach()` 報錯提示安裝 |
| **Robotics System Toolbox** (MathWorks) | `planTrajectoryCartesian()` 中的 SLERP 姿態插值（`rotm2quat`、`quatinterp`、`quat2rotm`） | 回退到軸角線性插值，並發出 `warning` |

工具箱在 `Arm7R` 構造函數中通過 `try-catch` 自動檢測，檢測標誌存儲在 `hasRoboticsToolbox` 和 `hasRoboticsSystemToolbox` 屬性中。

## 代碼組織與模塊劃分

### 1. Arm7R（運動學核心）
`Arm7R` 是 `handle` 類，封裝 7R 機械臂的全部運動學功能：

| 方法 | 說明 |
|------|------|
| `forwardKinematics(q)` | 正向運動學，返回 4x4 齊次變換矩陣 |
| `inverseKinematics(T)` | 逆向運動學，解析閉式解；固定 theta1=0 消除冗餘；返回 `[q, err]`（err=0 成功，err=1 無解） |
| `planTrajectoryCartesian(...)` | 笛卡爾空間軌跡規劃（位置 Lerp + 姿態 SLERP/軸角插值），輸出 `simin` 格式；IK 失敗點填充 `nan(1,7)` |
| `planTrajectoryJoint(...)` | 關節空間線性插值，輸出 Simulink 可用的 `simin` 格式（1ms 一個點） |
| `jacobian(q)` | 雅可比矩陣；有工具箱時用 `jacob0`，否則數值微分（delta=1e-6） |
| `conditionNumber(q)` | 雅可比條件數，評估奇異點接近程度 |
| `simplePlot(q)` | 無依賴的 3D 骨架繪圖（白色背景、透視、rotate3d） |
| `getJointPositions(q)` | 計算各關節在世界坐標系中的位置（8x3） |
| `displayDHTable()` | 在命令行打印 DH 參數表 |
| `setJointLimits(lower, upper)` | 設置關節角度限制（默認 [-pi, pi]） |
| `rotm2axang(R)` / `axang2rotm(axis, angle)` | 旋轉矩陣與軸角互相轉換（Rodrigues 公式） |

### 2. 文件隊列橋接架構

- **啟動層**: `robotagent.m` 一鍵啟動，自動初始化 Figure 與文件監聽 timer
- **監聽層**: `timer` 每 0.5 秒掃描 `incoming/` 目錄，發現新文件即執行
- **執行層**: 生成的 `.m` 腳本包含完整的 Quintic 軌跡規劃與動畫播放邏輯

支持的指令類型（`generate_robot_cmd.m` 中 `buildCode` 的 `switch` 分支）：
- `home` — 回到零位（`q_target = zeros(1,7)`，使用 `quinticTrajectory` 關節空間插值）
- `move_to` — 笛卡爾空間點到點移動（保持當前姿態，只改位置；使用 `planTrajectoryCartesian`）
- `relative_move` — 相對移動（沿 X/Y/Z 軸偏移，保持姿態；使用 `planTrajectoryCartesian`）
- `joint_move` — 單關節轉動（支持度或弧度，默認度；使用 `quinticTrajectory`）
- `trajectory` — 預定義軌跡（`circle` 圓周、`line` 直線；使用 `planTrajectoryCartesian`）
- `get_status` — 查詢當前關節角、末端位姿

### 3. 自然語言控制

`parseNaturalLanguage.m` 通過正則表達式解析中英混合輸入：

**支持的 NLP 指令**（實際代碼中的正則模式）：
- `home` / `回到原位` / `回零位` / `歸零` -> `home`
- `走到 500 0 800`、`move to (500,0,800)` -> `move_to`
- `z向下200`、`向下移動100`、`x向移動-50`、`move x by -50` -> `relative_move`
- `關節3轉45度`、`joint 2 30 deg` -> `joint_move`（默認 `angle_deg = true`）
- `畫圓 半徑200`、`circle radius 200` -> `trajectory` (circle)
- `直線到 (500,0,800)`、`line to (500,0,800)` -> `trajectory` (line)
- `status` / `現在姿態` / `狀態` / `get status` -> `get_status`

**時間解析**: `parseDuration` 支持 `用 X 秒`、`X秒`、`in X seconds`、`X s` / `X sec`，默認 **5 秒**。

**姿態調整指令**（不支持 NLP 解析，需 AI 直接生成自定義腳本）：
- X 軸水平（Y 軸不動）-> 繞 Y 軸旋轉
- Z 軸水平（Y 軸不動）-> 繞 Y 軸旋轉
- Z 軸反向（X 軸不動）-> 繞 X 軸旋轉 180度
- Z 軸指向 +X -> 繞 Y 軸旋轉 -90度

**⚠️ home 位置軸向對應**（由 `Arm7R.forwardKinematics(zeros(1,7))` 驗證）：
- 末端 X 軸 = `[0, 1, 0]` -> 與**世界 Y 軸重合**
- 末端 Y 軸 = `[1, 0, 0]` -> 與**世界 X 軸重合**
- 末端 Z 軸 = `[0, 0, -1]` -> 朝下

因此，在 home 位置下「繞末端 X 軸旋轉」等價於「繞世界 Y 軸旋轉」，可讓 Z 軸從朝下轉到朝 +/-X。

### 4. 文件調用關係總圖

#### 啟動流程
```
robotagent.m
    ├── fileparts(mfilename('fullpath')) -> scriptDir
    ├── addpath(fullfile(scriptDir, 'src'))
    ├── arm = Arm7R()                        % 構造函數：檢測工具箱、初始化 DH
    ├── current_q = zeros(1, 7)
    ├── fig = initRobotFigure(arm, current_q) % 創建 white figure，繪製初始骨架
    ├── fig.UserData.arm = arm
    ├── fig.UserData.current_q = current_q
    ├── fig.UserData.is_busy = false
    ├── mkdir(incoming_dir)
    ├── t = timer('ExecutionMode','fixedRate','Period',0.5, ...)
    │             'TimerFcn', @(~,~) processIncomingCommands(incoming_dir, fig))
    └── start(t)
```

#### 指令生成流程（Kimi CLI 端）
```
用戶輸入自然語言
    │
    ▼
parseNaturalLanguage.m ---> cmd_struct
    │                           ├── cmd = 'home' / 'move_to' / 'relative_move' / ...
    │                           ├── position / axis / distance / duration / radius
    │                           └── 正則表達式解析中英文混合輸入
    ▼
generate_robot_cmd(cmd_struct, 'incoming/')
    ├── buildCode(cmd)          % 按 cmd 類型組裝 MATLAB 代碼字符串
    │   ├── home              -> fprintf + quinticTrajectory(zeros, zeros, ...)
    │   ├── move_to           -> FK -> 改位置 -> IK -> planTrajectoryCartesian
    │   ├── relative_move     -> FK -> 改單軸偏移 -> IK -> planTrajectoryCartesian
    │   ├── joint_move        -> 直接修改單關節 -> quinticTrajectory
    │   ├── trajectory circle -> 圓周採樣 -> 逐點 IK -> updateRobotFigure
    │   ├── trajectory line   -> planTrajectoryCartesian
    │   └── get_status        -> FK + fprintf 當前位姿
    └── 寫入 incoming/cmd_yyyymmdd_HHMMSS_FFF_cmdname.m
```

#### 指令執行流程（MATLAB 端）
```
timer (每 0.5 秒觸發)
    │
    ▼
processIncomingCommands(incoming_dir, fig)
    ├── if fig.UserData.is_busy -> return（動畫期間不處理新指令）
    ├── files = dir(fullfile(incoming_dir, 'cmd_*.m'))
    ├── [~, idx] = sort([files.datenum])  % 按創建時間排序
    ├── cmd_path = fullfile(incoming_dir, files(idx(1)).name)
    ├── log_path = fullfile(logs, 'log_時間戳.txt')
    ├── diary off; diary(log_path)         % 開始捕獲命令窗口輸出
    ├── fig.UserData.is_busy = true
    ├── assignin('base', 'arm', ...) / assignin('base', 'fig', ...) / assignin('base', 'current_q', ...)
    ├── try
    │       run(cmd_path)       % 執行生成的 .m 腳本（所有輸出被 diary 捕獲）
    │       copyfile(cmd_path, incoming_history/)  % 成功後保存歷史
    │       delete(cmd_path)    % 成功後刪除
    │   catch ME
    │       copyfile(cmd_path, incoming_history/)  % 失敗也保存歷史
    │       movefile(cmd_path, 'incoming/failed/') % 失敗後歸檔
    ├── diary off
    ├── printLog(cmd_name, log_path)    % 自動打印 LOG 區塊到命令窗口
    └── fig.UserData.is_busy = false
```

#### 生成的 .m 腳本內部執行流程（以 move_to 為例）
```
cmd_20260529_143052_123_move_to.m
    │
    ├── fig = findobj('Type', 'figure', 'Name', 'RobotAgent - 7R Arm')
    ├── ud = fig.UserData; arm = ud.arm; current_q = ud.current_q
    ├── T_cur = arm.forwardKinematics(current_q)
    ├── T_target = T_cur; T_target(1:3,4) = [x; y; z]   % 只改位置，保持姿態
    ├── steps = max(30, round(duration * 30))
    ├── [~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration)
    ├── if any(isnan(q_traj(1, :))), q_traj(1, :) = current_q; end   % 修復首幀 nan
    ├── nan_rows = find(any(isnan(q_traj), 2))
    ├── if ~isempty(nan_rows)
    │       % 播放有效段，然後暫停（不報錯）
    │       for i = 1:first_nan-1 -> updateRobotFigure(fig, q_traj(i,:))
    │       fig.UserData.current_q = q_traj(first_nan-1, :)
    │       fprintf('[PAUSE] IK unreachable at step X/Y. current_q=... target_pos=...')
    │       % [PAUSE] 信息由 diary 自動記錄到 logs/log_時間戳.txt
    │       return
    ├── for i = 1:size(q_traj, 1)
    │       updateRobotFigure(fig, q_traj(i, :))
    │       drawnow + tic/toc 忙等幀間隔（約 1/30 秒）
    ├── fig.UserData.current_q = q_target
    └── fprintf('[Done] move_to ...\n')
```

#### 核心模塊依賴圖
```
                              robotagent.m
                         （一鍵啟動 + timer 管理）
     │                    │                           │
     ▼                    ▼                           ▼
┌──────────┐      ┌──────────────┐          ┌─────────────────────┐
│ Arm7R.m  │      │initRobotFigure│         │processIncomingCommands│
│ (運動學)  │      │  (圖形初始化)  │         │    (文件監聽執行器)    │
└───┬──┬───┘      └──────────────┘         └──────────┬──────────┘
    │  │                                               │
    │  │         ┌─────────────────────────────────────┘
    │  │         │
    │  │    ┌────▼────┐       ┌──────────────┐
    │  └───►│   FK    │◄──────│ 生成的 .m 腳本 │
    │       │   IK    │       │（含完整軌跡邏輯）│
    │       │getJointPos│     └──────┬───────┘
    │       └────┬────┘              │
    │            │                   │
    │            │            ┌──────▼────────┐
    │            │            │quinticTrajectory│
    │            │            │ (五次多項式插值) │
    │            │            └──────┬────────┘
    │            │                   │
    │            │            ┌──────▼────────┐
    │            └───────────►│ animateRobot   │
    │                         │ (動畫播放循環)  │
    │                         └──────┬────────┘
    │                                │
    │                         ┌──────▼────────┐
    └────────────────────────►│updateRobotFigure│
                              │ (高效 set + drawnow limitrate)
                              └────────────────┘

外層（Kimi CLI，非 MATLAB 進程）：
    parseNaturalLanguage.m -> generate_robot_cmd.m -> incoming/*.m
```

### 5. Quintic Polynomial 速度規劃

`home` 與 `joint_move` 指令使用 **五次多項式（Quintic Polynomial）** 關節空間插值（`quinticTrajectory.m`）：

```
q(t) = q0 + a3*t^3 + a4*t^4 + a5*t^5
a3 = 10*(q1-q0) / T^3
a4 = -15*(q1-q0) / T^4
a5 = 6*(q1-q0) / T^5
```

邊界條件：起點與終點的速度、加速度均為零，確保運動平滑無頓挫。

參數默認值：`T = 5` 秒（用戶未指定時），`fps = 30`。總幀數 = `max(2, round(T * fps) + 1)`。

### 6. 笛卡爾空間軌跡規劃

`move_to`、`relative_move`、`trajectory` 使用 `Arm7R.planTrajectoryCartesian`：
- 位置線性插值（Lerp）
- 姿態插值：有 Robotics System Toolbox 時用 **SLERP**（四元數球面線性插值）；無時用軸角線性插值
- 逐點調用 `inverseKinematics`；若某點 IK 無解，該行填充 `nan(1,7)`
- 生成的腳本會檢測 `nan` 行，播放有效段後暫停（`fprintf '[PAUSE]'` 輸出到 log），而非直接拋出 `error`

## 構建與運行

無需編譯或構建步驟。MATLAB 為解釋型環境，直接運行腳本即可。

### 一鍵啟動（MATLAB GUI）

在 MATLAB 的 **Current Folder** 中找到 `robot-agent` 文件夾，雙擊 `robotagent.m` 或在 Editor 中點擊 **運行** 按鈕 ▶：

```
========================================
RobotAgent started.
Figure: RobotAgent - 7R Arm
Watching: D:\...\robot-agent\incoming
========================================
```

Figure 彈出，MATLAB 命令行不阻塞。

### 發送指令（Kimi CLI）

在 Kimi CLI 中輸入自然語言：
```
讓機械臂末端移動到 (500, 0, 800)，用 3 秒
```

Kimi CLI 內部處理流程：
1. `parseNaturalLanguage` 解析為 `cmd_struct`
2. `generate_robot_cmd` 生成對應 `.m` 代碼（內含 `quinticTrajectory` 或 `planTrajectoryCartesian`）
3. 寫入 `robot-agent/incoming/cmd_xxx.m`
4. MATLAB 的 `timer` 檢測並自動執行

### MATLAB 中直接解析（可選）

```matlab
cmd = parseNaturalLanguage('關節1轉90度');
generate_robot_cmd(cmd);
```

### PowerShell / 命令行批量運行測試

```powershell
matlab -batch "cd('D:\Document\code\Matlab\robot-agent'); addpath('src'); addpath('tests'); test_phase1_figure; test_phase2_filewatch; test_phase3_generator; test_phase4_nlp; test_phase5_cleanup; test_phase6_integration;"
```

## 測試策略

測試按 Phase 分層組織，覆蓋從底層圖形到端到端集成的完整鏈路：

| 測試文件 | 覆蓋範圍 |
|----------|----------|
| `test_phase1_figure.m` | Figure 初始化、UserData 完整性（含 `h_pose_text`）、updateRobotFigure 更新、句柄有效性、關閉後重建、animateRobot 時序與末幀位姿驗證、視覺截圖 |
| `test_phase2_filewatch.m` | 目錄自動創建、processIncomingCommands 單文件執行、多文件按 datenum 排序執行、錯誤腳本移至 `failed/` 並複製到 `incoming_history/`、`is_busy` 並發阻止、空目錄無異常 |
| `test_phase3_generator.m` | Quintic 起點終點精確性、起點/終點速度趨近零、反向對稱性、home/move_to/joint_move 代碼生成與實際執行、自定義 duration、文件名唯一性（毫秒級時間戳） |
| `test_phase4_nlp.m` | 中英混合 NLP 解析正確性（home/move_to/joint_move/circle/line/relative_move/status）、無法識別輸入報錯、端到端 NLP->生成->執行 |
| `test_phase5_cleanup.m` | 舊架構文件已刪除（RobotAgent.m、send_cmd.m、tcpserver 相關）、新函數可訪問、README 與 AGENTS.md 提及新架構且不含舊術語 |
| `test_phase6_integration.m` | 端到端 home/move_to/circle、帶時長的 move_to、快速連續 5 條指令排隊、`is_busy` 指令排隊、無效位姿容錯（unreachable target）、Quintic 軌跡平滑性（速度/加速度檢查）、IK 與代碼生成性能基準 |
| 其他測試文件 | `test_phase2_tcp.m`、`test_phase3_render.m`、`test_phase4_compute.m`、`test_phase5_commands.m`、`test_phase5_integration.m`、`test_phase6_startup.m`、`test_phase7_docs.m`、`test_integration.m` 覆蓋 TCP 遺留、渲染層、計算層、指令集、啟動流程、文檔一致性等額外維度（部分為舊架構遺留，可能失效） |

### 運行測試
```matlab
cd('D:\Document\code\Matlab\robot-agent');
addpath('src');
addpath('tests');

test_phase1_figure;
test_phase2_filewatch;
test_phase3_generator;
test_phase4_nlp;
test_phase5_cleanup;
test_phase6_integration;
```

視覺測試會自動將 Figure 截圖保存到 `tests/output/`，供人工審查。

## 代碼風格規範

### 單位約定
- **長度**: 毫米 (mm)
- **角度**: 弧度 (rad)（`joint_move` 的 `angle_deg` 標誌用於 NLP 層的度->弧度轉換）
- **時間**: 秒 (s)

### DH 參數約定
使用**標準 DH 參數**（Standard DH），參數向量 `P_DH = [e, k, i, l, m, n, j, b]`。
默認值: `[149.438, 147.9, 0, 458.09, 93.5, 360.71, 118.27, 272.42]`

### 代碼風格
- 類與方法使用**繁體中文注釋**，遵循 MATLAB Help Text 格式（`% 函數名 說明` + `%   輸入:` / `%   輸出:`）
- 方法間使用 `%%` 分隔，便於 MATLAB 分節導航
- 輸入輸出參數在注釋中明確標註維度與單位
- 工具箱缺失時優先降級（fallback）而非強制報錯
- 錯誤處理使用 `try-catch`，執行失敗的腳本移至 `incoming/failed/`
- 生成的 `.m` 腳本避免 UTF-8 BOM，推薦無 BOM 的 UTF-8 編碼

### Figure 句柄管理
- `initRobotFigure` 將所有圖形句柄（`ax`, `h_link`, `h_joints`, `h_labels`, `h_axes_base`, `h_axes_ee`, `h_pose_text`）存儲在 `fig.UserData` 結構體中
- `updateRobotFigure` 每次更新前檢查 `isvalid(fig)` 與 `isvalid(ud.ax)`，若無效則報錯提示重新運行 `robotagent`
- `robotagent.m` 在 `initRobotFigure` 後補充 `fig.UserData.arm`、`fig.UserData.current_q`、`fig.UserData.is_busy`
- 末端位姿矩陣（4x4）以等寬字體實時顯示在 Figure 左上角 annotation 中

## 安全與穩定性注意事項

1. **冗餘處理**: 7R 機械臂有 1 個冗餘自由度，逆運動學固定 `theta1 = 0` 以獲得唯一閉式解。若需優化 theta1，需在外部層疊加優化器。
2. **Figure 容錯**: `updateRobotFigure` 每次更新前檢查 `isvalid(fig)`，若用戶誤關窗口則報錯提示重新運行 `robotagent`。
3. **IK 無解與暫停機制**: 不可達位姿返回 `err = 1`。對於笛卡爾連續軌跡（`move_to`、`relative_move`、`trajectory`），若中間點 IK 無解，腳本不會拋出 `error`，而是：
   - 播放已計算出的有效段動畫
   - `[PAUSE]` 信息由 `diary` 自動記錄到 `logs/log_時間戳.txt`
   - 打印 `[PAUSE] IK unreachable at step X/Y`
   - 返回，保留 `fig.UserData.current_q` 為最後有效位姿
4. **指令隊列**: `incoming/` 目錄中的文件按創建時間排序執行；`is_busy` 標誌防止動畫期間並發執行新指令。
5. **錯誤腳本**: 執行失敗的 `.m` 文件自動移至 `incoming/failed/` 目錄，同時複製到 `incoming_history/`，避免阻塞後續指令。
6. **歷史歸檔**: 所有執行過的腳本（無論成功或失敗）均複製到 `incoming_history/` 目錄，便於回溯與調試。
7. **日誌記錄**: `processIncomingCommands` 使用 `diary` 捕獲每次執行的完整命令窗口輸出到 `logs/log_時間戳.txt`，執行結束後自動 `type(log_path)` 打印到命令窗口。
8. **精度與降級**: 缺少 Robotics System Toolbox 時，笛卡爾軌跡規劃的姿態插值降級為軸角線性插值，路徑可能非最優（非恆定角速度）。
9. **文件編碼**: 生成 `.m` 腳本時應避免 UTF-8 BOM，推薦使用無 BOM 的 UTF-8 編碼。
10. **base workspace 注入**: `processIncomingCommands` 通過 `assignin('base', ...)` 將 `arm`、`fig`、`current_q` 注入 base workspace，確保 `run(cmd_path)` 時腳本能訪問這些變量。

## 擴展預留

- 渲染層與運動學層邏輯解耦。未來若遷移為前後端分離，只需將 `updateRobotFigure` 替換為「通過 TCP 發送關節角序列到前端」，核心運動學代碼無需改動。
- 未來如需雙臂協同，可擴展持有兩個 `Arm7R` 對象，指令中添加 `"arm": "left"` / `"right"` 字段。
