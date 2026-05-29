# Robot-Agent 項目指南

## 項目概述

這是一個基於 MATLAB 的 7 自由度（7R）機械臂可視化與自然語言控制系統。項目在原有 `Arm7R` 運動學庫的基礎上，通過文件隊列橋接模式，實現「自然語言 → 即時 .m 代碼 → 文件隊列 → MATLAB 自動執行 → 實時動畫」的完整閉環。

- **語言**: MATLAB（純基礎環境，無強制工具箱依賴）
- **通信協議**: 文件系統隊列（incoming/ 目錄）
- **代碼與文檔語言**: 繁體中文
- **MATLAB 版本要求**: R2020b 或更高版本（需內置 `timer`）

## 文件結構

```
robot-agent/
├── robotagent.m                  % 一鍵啟動腳本（初始化 Figure + 啟動文件監聽 timer）
├── src/                          % 源代碼
│   ├── Arm7R.m                   % 7R 機械臂運動學類（FK / IK / 軌跡規劃 / 雅可比）
│   ├── initRobotFigure.m         % Figure 初始化函數
│   ├── updateRobotFigure.m       % Figure 高效更新函數
│   ├── animateRobot.m            % 動畫播放函數
│   ├── quinticTrajectory.m       % 五次多項式軌跡規劃
│   ├── generate_robot_cmd.m      % 代碼生成器（支持 home/move_to/joint_move/trajectory/relative_move/get_status）
│   ├── parseNaturalLanguage.m    % 自然語言解析器（中英混合）
│   └── processIncomingCommands.m % 文件監聽執行器
│
├── tests/                        % 測試腳本（分 Phase 組織）
│   ├── test_phase1_figure.m      % Phase 1: Figure 初始化與句柄有效性
│   ├── test_phase2_filewatch.m   % Phase 2: 文件監聽與指令執行
│   ├── test_phase3_generator.m   % Phase 3: Quintic 軌跡與代碼生成器
│   ├── test_phase4_nlp.m         % Phase 4: 自然語言解析
│   ├── test_phase5_cleanup.m     % Phase 5: 清理舊架構驗證
│   ├── test_phase6_integration.m % Phase 6: 端到端集成測試
│   └── output/                   % 測試輸出的 PNG 截圖
│
├── docs/                         % 文檔
│   ├── README_Arm7R.md           % Arm7R API 文檔與數學原理詳解
│   ├── ROBOT_AGENT_README.md     % RobotAgent 使用說明與故障排查
│   └── plan1.md                  % 系統實施計劃與架構設計文檔
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
| `inverseKinematics(T)` | 逆向運動學，解析閉式解；固定 θ1=0 消除冗餘；返回 `[q, err]` |
| `planTrajectoryCartesian(...)` | 笛卡爾空間軌跡規劃（位置 Lerp + 姿態 SLERP/軸角插值） |
| `planTrajectoryJoint(...)` | 關節空間線性插值，輸出 Simulink 可用的 `simin` 格式 |
| `jacobian(q)` | 雅可比矩陣；有工具箱時用 `jacob0`，否則數值微分 |
| `conditionNumber(q)` | 雅可比條件數，評估奇異點接近程度 |
| `simplePlot(q)` | 無依賴的 3D 骨架繪圖 |
| `getJointPositions(q)` | 計算各關節在世界坐標系中的位置（9×3） |

### 2. 文件隊列橋接架構

```
┌─────────────────┐         ┌─────────────────────────────┐
│   MATLAB 窗口    │         │      Kimi CLI 窗口          │
│  ┌───────────┐  │         │  ┌─────────────────────┐    │
│  │ 點擊運行   │  │         │  │ 用戶輸入自然語言    │    │
│  │robotagent │  │         │  │  • 解析意圖          │    │
│  │→ Figure   │  │         │  │  • 生成 .m 代碼      │    │
│  │→ timer    │  │         │  │  • 寫入 incoming/    │    │
│  └───────────┘  │         │  └──────────┬──────────┘    │
│         │        │         │             │ 寫文件         │
│         │ 0.5s   │         │             ▼               │
│         ▼        │         │  ┌─────────────────────┐    │
│  ┌───────────┐  │         │  │  cmd_xxx.m          │    │
│  │掃描incoming│◄─┼─────────┼──┤  (Quintic 軌跡代碼) │    │
│  │→ run()    │  │ 文件系統 │  └─────────────────────┘    │
│  └───────────┘  │         └─────────────────────────────┘
│         │        │
│         ▼        │
│  ┌───────────┐  │
│  │ Figure 更新│  │
│  └───────────┘  │
└─────────────────┘
```

- **啟動層**: `robotagent.m` 一鍵啟動，自動初始化 Figure 與文件監聽 timer
- **監聽層**: `timer` 每 0.5 秒掃描 `incoming/` 目錄，發現新文件即執行
- **執行層**: 生成的 `.m` 腳本包含完整的 Quintic 軌跡規劃與動畫播放邏輯

支持的指令類型：
- `home` — 回到零位
- `move_to` — 笛卡爾空間點到點移動（保持當前姿態）
- `relative_move` — 相對移動（沿 X/Y/Z 軸偏移，保持姿態）
- `joint_move` — 單關節轉動（默認角度）
- `trajectory` — 預定義軌跡（`circle`、`line`）
- `get_status` — 查詢當前關節角、末端位姿

### 自然語言控制

在 Kimi CLI 中直接輸入自然語言，AI 會解析意圖並生成對應 `.m` 腳本投遞到 `incoming/`。

**支持的 NLP 指令**：
- `回零位`、`home` → `home`
- `走到 500 0 800`、`move to (500,0,800)` → `move_to`
- `z向下200`、`向下移動100`、`x向移動-50` → `relative_move`
- `關節3轉45度`、`joint 2 30 deg` → `joint_move`
- `畫圓 半徑200`、`circle radius 200` → `trajectory`
- `status`、`姿態` → `get_status`

**姿態調整指令**（需 AI 直接生成自定義腳本）：
- X 軸水平（Y 軸不動）→ 繞 Y 軸旋轉
- Z 軸水平（Y 軸不動）→ 繞 Y 軸旋轉
- Z 軸反向（X 軸不動）→ 繞 X 軸旋轉 180°
- Z 軸指向 +X → 繞 Y 軸旋轉 -90°

**⚠️ home 位置軸向對應**：
- 末端 X 軸 = `[0, 1, 0]` → 與**世界 Y 軸重合**
- 末端 Y 軸 = `[1, 0, 0]` → 與**世界 X 軸重合**
- 末端 Z 軸 = `[0, 0, -1]` → 朝下

因此，在 home 位置下「繞末端 X 軸旋轉」等價於「繞世界 Y 軸旋轉」，可讓 Z 軸從朝下轉到朝 ±X。

### 3. Quintic Polynomial 速度規劃

所有關節空間運動使用 **五次多項式（Quintic Polynomial）** 插值：

```
q(t) = q0 + a3·t³ + a4·t⁴ + a5·t⁵
a3 = 10·(q1-q0) / T³
a4 = -15·(q1-q0) / T⁴
a5 = 6·(q1-q0) / T⁵
```

邊界條件：起點與終點的速度、加速度均為零，確保運動平滑無頓挫。

用戶未指定 duration 時，默認為 **5 秒**。

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
1. `parseNaturalLanguage` 解析為 `cmd_struct`
2. `generate_robot_cmd` 生成對應 `.m` 代碼（內含 `quinticTrajectory`）
3. 寫入 `robot-agent/incoming/cmd_xxx.m`
4. MATLAB 的 `timer` 檢測並自動執行

### MATLAB 中直接解析（可選）

```matlab
cmd = parseNaturalLanguage('關節1轉90度');
generate_robot_cmd(cmd);
```

## 測試策略

測試按 Phase 分層組織，覆蓋從底層圖形到端到端集成的完整鏈路：

| 測試文件 | 覆蓋範圍 |
|----------|----------|
| `test_phase1_figure.m` | Figure 初始化、句柄有效性、關閉後重建、視覺截圖、動畫播放 |
| `test_phase2_filewatch.m` | 文件監聽、指令執行順序、錯誤處理、`is_busy` 並發保護 |
| `test_phase3_generator.m` | Quintic 軌跡數學性質、代碼生成器、各指令模板可執行性 |
| `test_phase4_nlp.m` | 自然語言解析正確性、端到端 NLP→文件→執行 |
| `test_phase5_cleanup.m` | 舊文件清理、新架構一致性、文檔更新 |
| `test_phase6_integration.m` | 端到端全鏈路測試、快速連續指令、穩定性、性能基準 |

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

## 安全與穩定性注意事項

1. **冗餘處理**: 7R 機械臂有 1 個冗餘自由度，逆運動學固定 `θ1 = 0` 以獲得唯一閉式解。若需優化 θ1，需在外部層疊加優化器。
2. **Figure 容錯**: `updateRobotFigure` 每次更新前檢查 `isvalid(fig)`，若用戶誤關窗口則報錯提示重新運行 `robotagent`。
3. **IK 無解**: 不可達位姿返回 `err = 1`，生成的腳本會捕獲錯誤並輸出提示，機械臂保持原位不動。
4. **指令隊列**: `incoming/` 目錄中的文件按創建時間排序執行；`is_busy` 標誌防止動畫期間並發執行新指令。
5. **錯誤腳本**: 執行失敗的 `.m` 文件自動移至 `incoming/failed/` 目錄，避免阻塞後續指令。
6. **精度與降級**: 缺少 Robotics System Toolbox 時，笛卡爾軌跡規劃的姿態插值降級為軸角線性插值，路徑可能非最優（非恆定角速度）。

## 擴展預留

- 渲染層與運動學層邏輯解耦。未來若遷移為前後端分離，只需將 `updatePlot` 替換為「通過 TCP 發送關節角序列到前端」，核心運動學代碼無需改動。
- 未來如需雙臂協同，可擴展 `RobotAgent` 持有兩個 `Arm7R` 對象，指令中添加 `"arm": "left"` / `"right"` 字段。
