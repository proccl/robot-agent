<!-- AGENTS.md for robot-agent -->
# Robot-Agent 項目指南

## 項目概述

這是一個基於 MATLAB 的 7 自由度（7R）機械臂可視化與即時指令執行系統。項目採用**文件隊列橋接架構**，實現「自然語言 → AI 生成 `.m` 腳本 → 文件隊列 → MATLAB 自動執行 → 實時動畫」的完整閉環。

- **語言**：MATLAB（純基礎環境，無強制工具箱依賴）
- **通信協議**：文件系統隊列（`incoming/` 目錄）
- **代碼與文檔語言**：繁體中文
- **MATLAB 版本要求**：R2020b 或更高版本（需內置 `timer`）

> **架構演進說明**：本項目早期版本使用 TCP 服務器架構，現已完全重構為文件隊列架構。舊版 TCP 相關文件（`RobotAgent.m`、`run_robot_agent.m`、`send_robot_cmd.ps1`、`send_cmd.m`、`send_via_tcp.py` 等）均已移除。舊版的 `parseNaturalLanguage.m` 與 `generate_robot_cmd.m` 也已在 v0.0.4 重構中移除，現只保留 **AI 直接寫 `.m` 腳本**模式。

## 關鍵配置文件

本項目為純 MATLAB 腳本項目，**沒有** `pyproject.toml`、`package.json`、`Cargo.toml`、`Makefile` 或類似的編譯/包管理配置文件。唯一與構建相關的設置是：

- `.gitignore`：排除運行時目錄與臨時文件（`incoming/`、`logs/`、`incoming_history/`、`tests/output/*.png`、`.asv`、`.m~` 等）。
- `robotagent.m`：啟動腳本，內含 `addpath('src')` 與 `timer` 配置。

## 文件結構

```
robot-agent/
├── robotagent.m                  % 一鍵啟動腳本（初始化 Figure + 啟動文件監聽 timer）
├── skills/                       % 項目輔助 skill（供 Kimi CLI 使用）
│   └── robotagent-ops/
│       ├── SKILL.md              % RobotAgent 操作指南（自然語言指令、姿態調整、問題排查）
│       └── references/           % 詳細參考文檔
│           ├── architecture-reference.md
│           ├── function-signatures-reference.md
│           ├── pose-adjustment-reference.md
│           ├── script-templates-reference.md
│           └── troubleshooting-reference.md
├── src/                          % 源代碼
│   ├── Arm7R.m                   % 7R 機械臂運動學類（FK / IK / 軌跡規劃 / 雅可比）
│   ├── initRobotFigure.m         % Figure 初始化函數
│   ├── updateRobotFigure.m       % Figure 高效更新函數
│   ├── animateRobot.m            % 動畫播放函數
│   ├── quinticTrajectory.m       % 五次多項式關節空間軌跡規劃
│   ├── processIncomingCommands.m % 文件監聽執行器（timer 回調）
│   └── computeTrajectory.m       % 獨立軌跡計算函數（舊架構遺留，供參考）
│
├── tests/                        % 測試腳本（分 Phase 組織）
│   ├── run_all_tests.m           % 統一測試入口（執行全部 6 個 Phase）
│   ├── test_phase1_figure.m      % Phase 1: Figure 初始化、句柄有效性、動畫
│   ├── test_phase2_filewatch.m   % Phase 2: 文件監聽、執行順序、錯誤歸檔、並發保護
│   ├── test_phase3_generator.m   % Phase 3: Quintic 軌跡數學性質
│   ├── test_phase4_cleanup.m     % Phase 4: 舊架構清理驗證與文檔一致性
│   ├── test_phase5_e2e.m         % Phase 5: AI 直接寫代碼模式的端到端集成測試
│   ├── test_phase6_complex.m     % Phase 6: 複合多段指令鏈路測試
│   └── output/                   % 測試輸出的 PNG 截圖
│
├── incoming/                     % 運行時指令隊列目錄（.gitignore 排除）
│   └── failed/                   % 執行失敗的腳本歸檔
├── incoming_history/             % 所有執行過的腳本副本（成功與失敗均保留）
├── logs/                         % 每次腳本執行的 diary 日誌（.gitignore 排除）
│
├── docs/                         % 文檔
│   ├── README_Arm7R.md           % Arm7R API 文檔與數學原理詳解
│   ├── ROBOT_AGENT_README.md     % 舊版 RobotAgent 使用說明（僅供參考）
│   ├── robot_agent_cmds.json     % 指令協議 JSON Schema（舊版 TCP 協議，僅供參考）
│   ├── plan_refactor_filewatch.md % 架構重構計劃文檔
│   ├── test_refactor_plan.md     % 測試重構計劃（AI 直接寫代碼模式）
│   └── plan1.md / plan2.md / plan3.md % 系統實施計劃與架構設計文檔
│
├── README.md                     % 項目總覽（Quick Start、架構圖）
└── AGENTS.md                     % 本文件
```

## 技術棧與依賴

### 核心運行時（無需額外工具箱）
- **MATLAB R2020b+**（`timer` 為基礎環境函數）
- 所有運動學計算、繪圖、文件 I/O 均使用純 MATLAB 實現。

### 可選工具箱
| 工具箱 | 用途 | 缺失時行為 |
|--------|------|-----------|
| **Robotics Toolbox** (Peter Corke) | `Arm7R.plot()`、`teach()`、`jacobian()`（解析法） | `jacobian()` 回退到數值微分；`plot()`/`teach()` 報錯提示安裝 |
| **Robotics System Toolbox** (MathWorks) | `planTrajectoryCartesian()` 中的 SLERP 姿態插值 | 回退到軸角線性插值，並發出 `warning` |

工具箱在 `Arm7R` 構造函數中通過 `try-catch` 自動檢測。

## 代碼組織與模塊劃分

### 1. Arm7R（運動學核心）
`Arm7R` 是 `handle` 類，封裝 7R 機械臂的全部運動學功能：

| 方法 | 說明 |
|------|------|
| `forwardKinematics(q)` | 正向運動學，返回 4×4 齊次變換矩陣 |
| `inverseKinematics(T)` | 逆向運動學，解析閉式解；固定 θ1=0 消除冗餘；返回 `[q, err]`，`err=0` 成功，`err=1` 無解 |
| `planTrajectoryCartesian(T_start, T_end, steps, t_start, t_end)` | 笛卡爾空間軌跡規劃（位置 Lerp + 姿態 SLERP/軸角插值） |
| `planTrajectoryJoint(q_start, q_end, t_start, t_end)` | 關節空間線性插值，輸出 Simulink 可用的 `simin` 格式 |
| `jacobian(q)` | 雅可比矩陣；有工具箱時用 `jacob0`，否則數值微分 |
| `conditionNumber(q)` | 雅可比條件數，評估奇異點接近程度 |
| `simplePlot(q)` | 無依賴的 3D 骨架繪圖 |
| `getJointPositions(q)` | 計算各關節在世界坐標系中的位置（9×3） |
| `displayDHTable()` | 打印 DH 參數表 |

### 2. 可視化模塊
- `initRobotFigure.m`：創建白底 Figure，繪製連桿、關節散點、標註、Base/EE 坐標軸、左上角 4×4 位姿矩陣文本。
- `updateRobotFigure.m`：通過 `set()` 更新句柄，高效重繪；包含 `isvalid(fig)` 容錯。
- `animateRobot.m`：遍歷 `q_traj` 調用 `updateRobotFigure`，默認 30 fps。

### 3. 文件隊列橋接架構

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Kimi CLI 窗口                                  │
│                                                                             │
│  ┌─────────────────────────┐        ┌─────────────────────────────────┐    │
│  │   Agent（Kimi 主控）     │◄──────►│   Skill（robotagent-ops）       │    │
│  │  • 接收用戶自然語言      │ 查詢   │  • 自然語言指令對照表           │    │
│  │  • 讀取 Skill 解析意圖   │ 模板   │  • 腳本模板（home/move/圓軌跡） │    │
│  │  • 生成 .m 代碼並寫入    │ 經驗   │  • 函數簽名確認表               │    │
│  │  • 執行後讀取 Log 分析   │        │  • 常見問題與調試經驗           │    │
│  └───────────┬─────────────┘        └─────────────────────────────────┘    │
│              │                                                              │
│              │ 寫入 cmd_xxx.m                                               │
│              ▼                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ 執行結束後：讀取並分析 Log                                           │   │
│  │  • 讀取 logs/log_*.txt → 分析 [Done]/[ERR]/[PAUSE] → 向用戶匯報     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                  │                                    ▲
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
│  │ robotagent.m          │    │  ├── 掃描 incoming/，按 datenum 排序    │  │
│  │  → Figure 彈出        │    │  ├── diary 開始記錄 → logs/log_*.txt    │  │
│  │  → timer 啟動監聽     │    │  ├── run(cmd_path) 執行腳本             │  │
│  └───────────────────────┘    │  ├── diary off; type(log) 打印 LOG      │  │
│              │                 │  ├── 成功：複製到 incoming_history/     │  │
│              │ timer 0.5s      │  │      刪除 incoming 中的原文件         │  │
│              ▼                 │  └── 失敗：複製到 history + failed/     │  │
│  ┌───────────────────────────────────────────────────────────────────────┐   │
│  │ Figure 機械臂動畫更新（由腳本內 updateRobotFigure 驅動）              │   │
│  └───────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

- **啟動層**: `robotagent.m` 一鍵啟動，自動初始化 Figure 與文件監聽 `timer`。
- **監聽層**: `timer` 每 0.5 秒掃描 `incoming/` 目錄，發現 `cmd_*.m` 文件即執行最舊的一個。
- **執行層**: 生成的 `.m` 腳本包含完整的軌跡規劃與動畫播放邏輯，直接操作 `fig.UserData` 中的 `arm` 與 `current_q`。

支持的指令類型（由 AI 直接生成腳本實現）：
- `home` — 回到零位（使用關節空間 `quinticTrajectory`，不會觸發 PAUSE）
- `move_to` — 笛卡爾空間點到點移動（保持當前姿態）
- `relative_move` — 相對移動（沿 X/Y/Z 軸偏移，保持姿態）
- `joint_move` — 單關節轉動
- `trajectory` — 自定義軌跡（如 `circle`）
- `get_status` — 查詢當前關節角、末端位姿

### 自然語言控制

在 Kimi CLI 中直接輸入自然語言，AI 會根據 `skills/robotagent-ops/SKILL.md` 及 references 生成對應 `.m` 腳本投遞到 `incoming/`。

**支持的常見指令**：
- `回零位`、`home` → `home`
- `走到 500 0 800`、`move to (500,0,800)` → `move_to`
- `z向下200`、`向下移動100`、`x向移動-50` → `relative_move`
- `關節3轉45度`、`joint 2 30 deg` → `joint_move`
- `畫圓 半徑200`、`circle radius 200` → `trajectory`
- `status`、`姿態` → `get_status`

**姿態調整指令**（需 AI 直接生成自定義腳本，不經自然語言解析器）：
- X 軸水平（Y 軸不動）→ 繞 Y 軸旋轉
- Z 軸水平（Y 軸不動）→ 繞 Y 軸旋轉
- Z 軸反向（X 軸不動）→ 繞 X 軸旋轉 180°
- Z 軸指向 +X → 繞 Y 軸旋轉 -90°

**⚠️ home 位置軸向對應**：
- 末端 X 軸 = `[0, 1, 0]` → 與**世界 Y 軸重合**
- 末端 Y 軸 = `[1, 0, 0]` → 與**世界 X 軸重合**
- 末端 Z 軸 = `[0, 0, -1]` → 朝下

因此，在 home 位置下「繞末端 X 軸旋轉」等價於「繞世界 Y 軸旋轉」，可讓 Z 軸從朝下轉到朝 ±X。

### 指令執行流程（MATLAB 端）

```
timer (每 0.5 秒觸發)
    │
    ▼
processIncomingCommands(incoming_dir, fig)
    ├── if fig.UserData.is_busy → return（動畫期間不處理新指令）
    ├── files = dir(fullfile(incoming_dir, 'cmd_*.m'))
    ├── [~, idx] = sort([files.datenum])  % 按創建時間排序
    ├── cmd_path = fullfile(incoming_dir, files(idx(1)).name)
    ├── diary(log_path) 開始記錄
    ├── fig.UserData.is_busy = true
    ├── try
    │       run(cmd_path)       % 在函數工作空間執行生成的腳本
    │       copyfile → incoming_history/  % 成功後保存歷史
    │       delete(cmd_path)    % 成功後從隊列刪除
    │   catch ME
    │       copyfile → incoming_history/  % 失敗也保存歷史
    │       movefile(cmd_path, 'incoming/failed/')
    ├── diary off; type(log_path)  % 打印日誌到命令窗口
    └── fig.UserData.is_busy = false
```

執行腳本時，以下變量已在 `processIncomingCommands` 的函數工作空間中可用：
- `fig`：Figure 句柄（timer 回調參數）
- `arm = fig.UserData.arm`：Arm7R 對象
- `current_q = fig.UserData.current_q`：當前關節角

因此腳本通常以 `ud = fig.UserData; arm = ud.arm; current_q = ud.current_q;` 開頭。

### Quintic Polynomial 速度規劃

關節空間運動使用 **五次多項式（Quintic Polynomial）** 插值：

```
q(t) = q0 + a3·t³ + a4·t⁴ + a5·t⁵
a3 = 10·(q1-q0) / T³
a4 = -15·(q1-q0) / T⁴
a5 = 6·(q1-q0) / T⁵
```

邊界條件：起點與終點的速度、加速度均為零，確保運動平滑無頓挫。用戶未指定 `duration` 時，默認為 **5 秒**。

### 笛卡爾空間軌跡規劃

`move_to`、`relative_move`、`trajectory` 等笛卡爾空間指令可使用 `planTrajectoryCartesian`：

- **位置插值**：線性插值（Lerp）
- **姿態插值**：
  - 有 Robotics System Toolbox → SLERP（四元數球面插值）
  - 無工具箱 → 軸角線性插值（Rodrigues 公式），並發 `warning`
- **IK 逐點求解**：對每個插值位姿調用 `inverseKinematics`

**NaN / 無解處理策略（PAUSE 機制）**：
- 若軌跡中間某點 IK 無解，不調用 `error()`，不讓腳本崩潰。
- 將當前已播放的有效軌跡正常播放完畢。
- 更新 `fig.UserData.current_q` 為最後有效關節角。
- 在 MATLAB Command Window 輸出 `[PAUSE] IK unreachable at step X/Y. ...`
- Kimi CLI 端讀取 `logs/log_*.txt` 後主動詢問用戶決策（繼續 / 調整目標 / 回退）。

> `home` 指令始終使用 `quinticTrajectory`（關節空間），不會觸發 PAUSE。

## 開發規範

### 單位約定
- **長度**: 毫米 (mm)
- **角度**: 弧度 (rad)
- **時間**: 秒 (s)

### DH 參數約定
使用**標準 DH 參數**（Standard DH），參數向量 `P_DH = [e, k, i, l, m, n, j, b]`。
默認值: `[149.438, 147.9, 0, 458.09, 93.5, 360.71, 118.27, 272.42]`

### 代碼風格
- 類與方法使用中文注釋，遵循 MATLAB Help Text 格式。
- 方法間使用 `%%` 分隔，便於 MATLAB 分節導航。
- 輸入輸出參數在注釋中明確標註維度與單位。
- 工具箱缺失時優先降級（fallback）而非強制報錯。
- 錯誤處理使用 `try-catch`，執行失敗的腳本移至 `incoming/failed/`。

### 文件編碼
生成 `.m` 腳本時應避免 UTF-8 BOM，推薦使用無 BOM 的 UTF-8 編碼。

## 構建與運行

無需編譯或構建步驟。

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
1. 讀取 `skills/robotagent-ops/SKILL.md` 及 references 解析意圖。
2. AI 直接生成完整 `.m` 腳本（內含 `quinticTrajectory` 或 `planTrajectoryCartesian`）。
3. 寫入 `robot-agent/incoming/cmd_xxx.m`。
4. MATLAB 的 `timer` 檢測並自動執行。

**投放腳本後必做：主動讀取並分析最新 Log**

每次向 `incoming/` 投放腳本後，Kimi CLI **必須**主動完成以下步驟，**不得等待用戶主動詢問**：

1. **讀取 `logs/` 目錄下最新的 `log_*.txt`**：
   ```bash
   ls -t logs/log_*.txt | head -1 | xargs cat
   ```
2. **分析 Log 內容**，確認執行結果：
   - `[Done]` — 執行成功，向用戶確認軌跡結果
   - `[ERR]` — 執行失敗，分析錯誤行號與原因並報告
   - `[PAUSE]` — 軌跡中斷，提取 `current_q` 與 `target_pos` 分析是否超出工作空間
3. **注意**：`log_*.txt` 的文件創建時間可能早於內容對應的腳本投遞時間，**務必以日誌內容的 `[RX] cmd_...` 行確認對應關係**，不要只按文件時間排序。
4. **向用戶匯報分析結果**。

> ❌ **禁止行為**：投放腳本後只回覆「已寫入，請觀察」而不主動讀取 log。

## 測試策略

測試按 Phase 分層組織，覆蓋從底層圖形到端到端集成的完整鏈路：

| 測試文件 | 覆蓋範圍 |
|----------|----------|
| `test_phase1_figure.m` | Figure 初始化、句柄有效性、關閉後重建、視覺截圖、動畫播放 |
| `test_phase2_filewatch.m` | 文件監聽、指令執行順序、錯誤處理、`is_busy` 並發保護 |
| `test_phase3_generator.m` | Quintic 軌跡數學性質（起終點、速度、對稱性） |
| `test_phase4_cleanup.m` | 舊架構文件已刪除、新函數可訪問、README/AGENTS.md 提及新架構 |
| `test_phase5_e2e.m` | AI 直接寫代碼模式的端到端測試（home、move_to 可達/不可達、relative_move、joint_move、多段排隊、PAUSE 機制） |
| `test_phase6_complex.m` | 複合多段指令鏈路（姿態+位置調整、相對移動、圓軌跡、回零位） |

### 運行測試

推薦使用統一入口：
```matlab
cd('D:\Document\code\Matlab\robot-agent\tests');
addpath('../src');
run_all_tests;
```

或單獨運行各 Phase 測試：
```matlab
cd('D:\Document\code\Matlab\robot-agent');
addpath('src');
addpath('tests');

test_phase1_figure;
test_phase2_filewatch;
test_phase3_generator;
test_phase4_cleanup;
test_phase5_e2e;
test_phase6_complex;
```

視覺測試會自動將 Figure 截圖保存到 `tests/output/`，供人工審查。

或通過 PowerShell / CMD 一次性運行：
```powershell
matlab -batch "cd('D:\Document\code\Matlab\robot-agent\tests'); addpath('../src'); run_all_tests;"
```

## 安全與穩定性注意事項

1. **冗餘處理**: 7R 機械臂有 1 個冗餘自由度，逆運動學固定 `θ1 = 0` 以獲得唯一閉式解。若需優化 θ1，需在外部層疊加優化器。
2. **Figure 容錯**: `updateRobotFigure` 每次更新前檢查 `isvalid(fig)`，若用戶誤關窗口則報錯提示重新運行 `robotagent`。
3. **IK 無解**: 不可達位姿返回 `err = 1`。笛卡爾軌跡中遇到中間點無解時觸發 PAUSE 機制，播放至最後有效點後暫停，不崩潰。`home` 指令因使用關節空間插值，不會觸發此問題。
4. **指令隊列**: `incoming/` 目錄中的文件按創建時間排序執行；`is_busy` 標誌防止動畫期間並發執行新指令。
5. **錯誤腳本**: 執行失敗的 `.m` 文件自動移至 `incoming/failed/` 目錄，同時複製到 `incoming_history/` 供回溯，避免阻塞後續指令。
6. **日誌機制**: `processIncomingCommands` 使用 `diary` 捕獲每次執行的全部命令窗口輸出到 `logs/log_*.txt`，執行結束後自動 `type(log_path)` 打印到命令窗口。
7. **精度與降級**: 缺少 Robotics System Toolbox 時，笛卡爾軌跡規劃的姿態插值降級為軸角線性插值，路徑可能非最優（非恆定角速度）。
8. **文件編碼**: 生成 `.m` 腳本時應避免 UTF-8 BOM，推薦使用無 BOM 的 UTF-8 編碼。
9. **動畫時序**: `animateRobot.m` 使用 `pause(1/fps)`，在 `timer` 回調鏈中可能被 MATLAB 忽略，導致動畫快速完成。如需真實時間播放，可在腳本中使用 `tic/toc` 自行 busy-wait。

## 擴展預留

- 渲染層與運動學層邏輯解耦。未來若遷移為前後端分離，只需將 `updateRobotFigure` 替換為「通過 TCP 發送關節角序列到前端」，核心運動學代碼無需改動。
- 未來如需雙臂協同，可擴展持有兩個 `Arm7R` 對象，指令中添加 `"arm": "left"` / `"right"` 字段。

## 常見陷阱速查

| 陷阱 | 正確做法 |
|------|---------|
| `planTrajectoryCartesian` 返回值順序錯誤 | `[~, q_traj, ~] = ...`（記憶口訣：「T 在前，q 在中」） |
| `planTrajectoryCartesian` 傳入 `current_q` | 第一參數必須是 `T_start`（4×4 位姿矩陣），用 `arm.forwardKinematics(current_q)` 獲取 |
| `assignin('base', 'arm', ...)` 後腳本找不到 `arm` | `run()` 在函數工作空間執行，改用 `arm = fig.UserData.arm;` |
| `quinticTrajectory` 傳 `steps` 作為第三參數 | 第三參數是 `T`（總時間秒），`steps` 由函數內部自動計算 |
| `animateRobot` 傳 `arm` 作為第二參數 | 簽名是 `animateRobot(fig, q_traj, fps)` |
| 腳本中覆蓋 `fig.UserData` | `initRobotFigure` 存了圖形句柄，啟動腳本只能追加字段，不能整體覆蓋 |
| 投放後立即讀取 log 卻找不到對應記錄 | `timer` 最長 0.5s 才觸發，加上動畫 duration，需等待足夠時間再讀取 |
| 只看 log 文件時間判斷是否為最新指令 | 文件創建時間可能早於內容對應的腳本時間，應以 `[RX] cmd_...` 內容為準 |

> 完整排查表與開發經驗見 `skills/robotagent-ops/references/troubleshooting-reference.md`。
