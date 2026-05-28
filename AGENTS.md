# Robot-Agent 項目指南

## 項目概述

這是一個基於 MATLAB 的 7 自由度（7R）機械臂可視化與 TCP 遠程控制系統。項目在原有 `Arm7R` 運動學庫的基礎上，疊加了 `RobotAgent` TCP 服務端，實現「JSON 指令 → 軌跡計算 → 實時動畫」的完整閉環。

- **語言**: MATLAB（純基礎環境，無強制工具箱依賴）
- **通信協議**: JSON over TCP（localhost）
- **代碼與文檔語言**: 繁體中文
- **MATLAB 版本要求**: R2020b 或更高版本（需內置 `tcpserver`）

## 文件結構

```
robot-agent/
├── src/                          % 源代碼
│   ├── Arm7R.m                   % 7R 機械臂運動學類（FK / IK / 軌跡規劃 / 雅可比）
│   ├── RobotAgent.m              % TCP 服務端主類（通信 + 指令調度 + 動畫引擎）
│   └── computeTrajectory.m       % 獨立軌跡生成函數（供後台 worker 調用）
│
├── scripts/                      % 啟動與輔助腳本
│   ├── run_robot_agent.m         % 一鍵啟動 RobotAgent 服務器
│   └── send_robot_cmd.ps1        % PowerShell TCP 客戶端
│
├── tests/                        % 測試腳本（分 Phase 組織）
│   ├── test_phase1_figure.m      % Phase 1: Figure 初始化與句柄有效性
│   ├── test_phase2_tcp.m         % Phase 2: TCP 通信與 JSON 解析
│   ├── test_phase3_render.m      % Phase 3: 渲染循環幀率與流暢度
│   ├── test_phase4_compute.m     % Phase 4: 後台計算與隊列推送
│   ├── test_phase5_commands.m    % Phase 5: 各指令正確性驗證
│   ├── test_phase5_integration.m % Phase 5: 簡化集成驗證
│   ├── test_phase6_startup.m     % Phase 6: 啟動腳本與輔助工具
│   ├── test_phase7_docs.m        % Phase 7: 文檔一致性檢查
│   ├── test_integration.m        % Phase 8: 端到端集成測試
│   └── output/                   % 測試輸出的 PNG 截圖
│
├── docs/                         % 文檔
│   ├── README_Arm7R.md           % Arm7R API 文檔與數學原理詳解
│   ├── ROBOT_AGENT_README.md     % RobotAgent 使用說明與故障排查
│   ├── robot_agent_cmds.json     % TCP 指令協議 JSON Schema
│   └── plan1.md                  % 系統實施計劃與架構設計文檔
│
├── README.md                     % 項目總覽（Quick Start、架構圖）
└── AGENTS.md                     % 本文件
```

## 技術棧與依賴

### 核心運行時（無需額外工具箱）
- **MATLAB R2020b+**（`tcpserver`、`jsondecode`、`timer` 為基礎環境函數）
- 所有運動學計算、繪圖、TCP 通信均使用純 MATLAB 實現。

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

### 2. RobotAgent（服務端主類）
`RobotAgent < handle` 整合了三層職責：

- **通信層**: `tcpserver` 生命周期管理（`start` / `stop` / `dataCallback`）
- **指令調度層**: `executeCommand` 根據 `cmd` 字段分發到具體動作
- **動畫引擎層**: `timer` 驅動的 30fps 渲染循環 + 軌跡隊列播放

支持的 TCP 指令（詳見 `docs/robot_agent_cmds.json`）：
- `home` — 回到零位
- `move_to` — 笛卡爾空間點到點移動
- `joint_move` — 單關節轉動（支持角度/弧度）
- `trajectory` — 預定義軌跡（`circle`、`line`）
- `set_speed` — 動畫倍速（0.1 ~ 5.0）
- `get_status` — 查詢當前關節角、末端位姿、忙碌狀態
- `plot` — 強制刷新 Figure

### 3. 異步雙循環架構
```
┌─────────────────┐     ┌─────────────────┐
│   渲染循環      │     │   計算循環      │
│   (timer)       │     │   (同步回調)    │
│   30fps         │     │                 │
│   讀取隊列      │←────┤   收到指令      │
│   updatePlot    │     │   計算軌跡      │
└─────────────────┘     │   推入隊列      │
                        └─────────────────┘
```
- **渲染循環**: 獨立 `timer('fixedRate', Period=1/30)`，持續運行，從 `trajectory_queue` 取幀並調用 `updatePlot`
- **計算循環**: TCP 回調中同步調用 `computeTrajectoryAsync`（因 MATLAB `-batch` 模式下 `parfeval` 無法序列化用戶定義函數，已回退為同步計算；`timer` 仍保持異步渲染）
- **隊列機制**: 新指令到達時覆蓋舊隊列；`is_busy` 標誌防止重複觸發

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
- 錯誤處理使用 `try-catch`，TCP 層錯誤通過 JSON 響應回傳客戶端

## 構建與運行

無需編譯或構建步驟。在 MATLAB 中將項目文件夾加入路徑即可：

```matlab
% GUI 模式啟動
cd('D:\Document\code\Matlab\robot-agent');
run_robot_agent;        % 默認端口 12345
run_robot_agent(12346); % 指定端口

% 或手動創建對象
agent = RobotAgent();
agent.start(12345);
agent.startRenderLoop();
```

```powershell
# Batch 模式啟動（無 GUI，適合後台運行）
matlab -batch "cd('D:\Document\code\Matlab\robot-agent'); addpath('src'); addpath('scripts'); run_robot_agent;"
```

發送指令示例：

```powershell
# 通過 PowerShell（僅發送，讀取響應不穩定）
.\scripts\send_robot_cmd.ps1 -Message '{"cmd":"home","duration":2}'
```

```matlab
% 通過 MATLAB tcpclient（推薦，讀寫均可靠）
c = tcpclient('127.0.0.1', 12345);
configureTerminator(c, 'LF');
writeline(c, '{"cmd":"get_status"}');
flush(c); pause(0.5);
resp = readline(c);

% move_to 指令格式（注意是 position 數組，非單獨 x/y/z）
writeline(c, '{"cmd":"move_to","position":[266.17,-93.5,595.818],"duration":2}');
flush(c); pause(0.5);
resp = readline(c);
```

```matlab
% 通過 batch 模式臨時腳本（CLI Agent 推薦方案）
% 寫入 temp_cmd.m，然後 matlab -batch "temp_cmd"
```

## 測試策略

測試按 Phase 分層組織，覆蓋從底層圖形到端到端集成的完整鏈路：

| 測試文件 | 覆蓋範圍 |
|----------|----------|
| `test_phase1_figure.m` | Figure 初始化、句柄有效性、關閉後重建、視覺截圖 |
| `test_phase2_tcp.m` | 服務器啟動、端口自動遞增、JSON 解析、無效輸入處理、連續指令 |
| `test_phase3_render.m` | 渲染循環幀率（~30fps）、手動更新、隊列播放、Figure 自動重建 |
| `test_phase4_compute.m` | 軌跡計算、隊列覆蓋、`is_busy` 標誌、不可達位姿容錯 |
| `test_phase5_commands.m` | 各指令正確性、FK/IK 一致性、錯誤處理 |
| `test_phase6_startup.m` | 一鍵啟動腳本、PowerShell 客戶端、非默認端口、服務器重啟 |
| `test_phase7_docs.m` | JSON Schema 完整性、文檔示例覆蓋率、錯誤碼文檔 |
| `test_integration.m` | 端到端 TCP 發送 → 動畫執行、快速連續指令、穩定性、性能基準 |

### 運行測試
```matlab
cd('D:\Document\code\Matlab\robot-agent');
addpath('src');
addpath('tests');

test_phase1_figure;
test_phase2_tcp;
test_phase3_render;
test_phase4_compute;
test_phase5_commands;
test_phase6_startup;
test_phase7_docs;
test_integration;
```

視覺測試會自動將 Figure 截圖保存到 `tests/output/`，供人工審查。

## 安全與穩定性注意事項

1. **冗餘處理**: 7R 機械臂有 1 個冗餘自由度，逆運動學固定 `θ1 = 0` 以獲得唯一閉式解。若需優化 θ1，需在外部層疊加優化器。
2. **TCP 僅綁定 localhost**: `tcpserver('127.0.0.1', port)`，默認不接受外部網絡連接。
3. **端口佔用**: `start()` 捕獲端口佔用異常，自動遞增嘗試備用端口（12346、12347…），最多 10 次。
   - **實際案例**：舊 MATLAB 進程 PID 31932 長期占用 12345，新實例自動遞增到 12348，機制有效。
   - **檢查占用**：`netstat -ano | findstr ":12345"`
4. **Figure 容錯**: `updatePlot` 每次更新前檢查 `isvalid(fig)`，若用戶誤關窗口則自動調用 `initFigure` 重建。
5. **IK 無解**: 不可達位姿返回 `err = 1`，TCP 響應 `status: error`，機械臂保持原位不動。
6. **Batch 模式限制**: `parfeval` / `backgroundPool` 在 `-batch` 模式下無法序列化用戶定義函數，軌跡計算已回退為同步執行；渲染仍通過 `timer` 保持異步。
7. **跨進程 TCP 通信**: GUI MATLAB 運行 RobotAgent（`tcpserver`）與外部進程（PowerShell/Python）的 TCP 讀取兼容性差。實戰中採用「GUI MATLAB 運行服務端 + batch MATLAB 臨時腳本發送指令」的模式最為穩定。
8. **指令格式嚴格性**: `move_to` 指令必須使用 `position` 數組字段（如 `[266.17,-93.5,595.818]`），單獨傳遞 `x`、`y`、`z` 字段會返回錯誤。
7. **精度與降級**: 缺少 Robotics System Toolbox 時，笛卡爾軌跡規劃的姿態插值降級為軸角線性插值，路徑可能非最優（非恆定角速度）。

## 擴展預留

- 渲染層與運動學層邏輯解耦。未來若遷移為前後端分離，只需將 `updatePlot` 替換為「通過 TCP 發送關節角序列到前端」，核心運動學代碼無需改動。
- 未來如需雙臂協同，可擴展 `RobotAgent` 持有兩個 `Arm7R` 對象，指令中添加 `"arm": "left"` / `"right"` 字段。
