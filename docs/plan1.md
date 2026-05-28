# Robot-Agent 系統實施計劃

## 目標
在現有 `Arm7R` 基礎上，建立一個 **robot-agent** 系統，實現：
> 用戶在 Kimi CLI 輸入自然語言指令 → MATLAB Figure 中的機械臂即時執行對應軌跡。

## 核心約束
- **純 MATLAB 實時通信**：MATLAB 端僅用內置函數（`tcpserver`，R2020b+ 可用，R2023b 確認支持），不額外依賴 Python 橋接層。
- **可視化**：使用 `Arm7R.simplePlot()`（無需 Robotics Toolbox）。
- **指令格式**：自然語言輸入，由 Kimi CLI 側解析為結構化 JSON，透過 TCP 發送。

---

## 系統架構

### 架構選型：一體化異步雙循環

| 維度 | 設計選擇 |
|------|---------|
| **服務端職責** | 軌跡生成（後台）+ Figure 渲染（前台） |
| **通信** | `tcpserver` 異步回調接收指令 |
| **渲染** | `timer` 定時器 30fps 獨立運行 |
| **計算** | `parfeval` / `backgroundPool` 後台執行軌跡計算 |
| **適用場景** | 本地演示、快速原型、單用戶 |

**核心設計原則**：
1. **渲染與計算分離**：Figure 動畫由獨立 `timer` 驅動，軌跡計算由 `parfeval` 在後台執行，兩者通過隊列交換數據
2. **通信與計算分離**：TCP 回調只負責接收指令並啟動後台任務，立即返回，不阻塞新連接
3. **單進程調試簡單**：啟動一步完成，無需維護多套代碼

> **擴展預留**：渲染層與運動學層邏輯解耦。未來若要遷移到前後端分離，只需將 `updatePlot` 替換為「通過 TCP 發送關節角序列到前端」，核心運動學代碼無需改動。

#### 異步程度說明

| 層次 | 是否異步 | 機制 | 說明 |
|------|---------|------|------|
| **TCP 通信** | ✅ 異步 | `tcpserver` 自動回調 | 數據到達時觸發，無需輪詢 |
| **動畫渲染** | ✅ 異步 | `timer` 定時器 | 獨立於主線程，固定幀率 |
| **軌跡計算** | ⚠️ 可選異步 | `parfeval` / `backgroundPool` | **必須顯式實現**，否則默認在回調線程同步執行，阻塞新指令接收 |

**結論**：方案A「可以是」完全異步的，但這取決於是否實現雙循環架構。如果不做雙循環（即回調中直接計算軌跡再啟動 timer），那只是「半異步」——動畫不卡，但新指令會被計算過程阻塞。

### 一體化架構圖

```
┌─────────────────┐     自然語言        ┌─────────────────────────────┐
│   用戶 (Kimi)   │ ──────────────────→ │  Kimi CLI (Agent 解析層)    │
└─────────────────┘                     │  • 解析意圖                  │
                                        │  • 生成 JSON 指令            │
                                        │  • 調用 PowerShell TCP 發送  │
                                        └─────────────┬───────────────┘
                                                      │ TCP (localhost)
                                                      ↓
                                        ┌─────────────────────────────┐
                                        │      RobotAgent Server      │
                                        │  ┌───────────────────────┐  │
                                        │  │   通信層 (tcpserver)   │  │
                                        │  │   指令調度器           │  │
                                        │  └───────────┬───────────┘  │
                                        │              ↓               │
                                        │  ┌───────────────────────┐  │
                                        │  │   運動學層 (Arm7R)     │  │
                                        │  │   - 軌跡生成           │  │
                                        │  │   - 逆運動學           │  │
                                        │  └───────────┬───────────┘  │
                                        │              ↓               │
                                        │  ┌───────────────────────┐  │
                                        │  │   渲染層 (Figure)      │  │
                                        │  │   - updatePlot()       │  │
                                        │  │   - timer 動畫播放     │  │
                                        │  └───────────────────────┘  │
                                        └─────────────────────────────┘
```

### 通信機制詳解
- **MATLAB 端**：`tcpserver('127.0.0.1', port)` + `configureCallback` 非阻塞回調。接收到數據後，在回調中解析指令並調度對應的軌跡/動作函數。
- **Kimi CLI 端**：通過 `Shell` 工具執行 PowerShell 一行命令，利用 `System.Net.Sockets.TcpClient` 將指令字符串寫入 TCP 流。例如：
  ```powershell
  $c = New-Object System.Net.Sockets.TcpClient('127.0.0.1',12345); $s = $c.GetStream(); $b = [Text.Encoding]::UTF8.GetBytes('{"cmd":"home"}'); $s.Write($b,0,$b.Length); $c.Close()
  ```

#### 為什麼用 JSON 作為指令格式？
雖然純文本（如 `home` 或 `move_to 500 0 800`）更簡短，但 JSON 有以下優勢：
- **結構化**：參數名明確（`"position": [x,y,z]` 而非按順序解析數字），不易出錯
- **可擴展**：新增參數不影響舊指令兼容性（如 `"duration"` 可選）
- **原生支持**：MATLAB R2016b+ 內置 `jsondecode`/`jsonencode`，PowerShell 有 `ConvertTo-Json`，零額外依賴
- **響應回傳**：`get_status` 等查詢指令需要返回結構化數據，JSON 是最自然的選擇

> 若未來需要極致簡化，可疊加一層「簡短指令」適配器（如 `h` → `home`，`m 500 0 800` → `move_to`），但底層仍保持 JSON 以確保協議清晰。

---

## 項目結構

```
robot-agent/
├── src/                             % 源代碼
│   ├── Arm7R.m                      % 原有：7R 機械臂運動學類
│   ├── RobotAgent.m                 % 新增：MATLAB TCP 服務端主類
│   │                                %   - tcpserver 生命周期管理
│   │                                %   - JSON 指令解析與調度
│   │                                %   - 動畫引擎（timer 異步播放）
│   │                                %   - 高效 Figure 更新（句柄緩存）
│   └── run_robot_agent.m            % 新增：一鍵啟動腳本
│
├── tests/                           % 測試腳本（每個 Phase 對應一個）
│   ├── test_phase1_figure.m         % Phase 1 測試：Figure 初始化與句柄
│   ├── test_phase2_tcp.m            % Phase 2 測試：TCP 通信與 JSON 解析
│   ├── test_phase3_render.m         % Phase 3 測試：渲染循環幀率與流暢度
│   ├── test_phase4_compute.m        % Phase 4 測試：後台計算與隊列推送
│   ├── test_phase5_commands.m       % Phase 5 測試：各指令正確性
│   ├── test_phase6_startup.m        % Phase 6 測試：啟動腳本與輔助工具
│   └── test_integration.m           % 集成測試：端到端指令發送與動畫
│
├── scripts/                         % 輔助腳本
│   └── send_robot_cmd.ps1           % PowerShell TCP 客戶端
│
├── docs/                            % 文檔
│   ├── README_Arm7R.md              % 原有：Arm7R API 文檔與數學原理
│   ├── ROBOT_AGENT_README.md        % 新增：使用說明與故障排查
│   └── robot_agent_cmds.json        % 新增：指令協議文檔（JSON Schema）
│
└── AGENTS.md                        % 原有：Agent 開發指南
```

### 模塊關係圖

```
┌─────────────────────────────────────────┐
│           Kimi CLI (Agent 側)            │
│  ┌─────────────┐    ┌─────────────────┐ │
│  │ 自然語言解析 │───→│ JSON 指令生成器  │ │
│  └─────────────┘    └────────┬────────┘ │
└──────────────────────────────┼──────────┘
                               │ Shell 調用
                               ↓
┌─────────────────────────────────────────┐
│      send_robot_cmd.ps1 (PowerShell)     │
│  ┌─────────────────────────────────────┐ │
│  │ System.Net.Sockets.TcpClient        │ │
│  │ 連接 127.0.0.1:PORT → 發送 JSON    │ │
│  └─────────────────────────────────────┘ │
└─────────────────────────────────────────┘
                               │ TCP
                               ↓
┌─────────────────────────────────────────┐
│      RobotAgent.m (MATLAB 服務端)        │
│  ┌─────────────┐    ┌─────────────────┐ │
│  │  tcpserver  │───→│  dataCallback   │ │
│  │  (監聽端口)  │    │  (解析 JSON)    │ │
│  └─────────────┘    └────────┬────────┘ │
│                              ↓          │
│  ┌─────────────┐    ┌─────────────────┐ │
│  │ executeCmd  │←───│   指令調度器    │ │
│  │ (分發執行)  │    └─────────────────┘ │
│  └──────┬──────┘                         │
│         ↓                               │
│  ┌─────────────────────────────────────┐ │
│  │  動畫引擎 (timer + updatePlot)      │ │
│  │  - animateTrajectory() 規劃幀序列   │ │
│  │  - timer 異步播放不阻塞 TCP         │ │
│  │  - set() 更新 plot3 句柄高效渲染    │ │
│  └─────────────────────────────────────┘ │
│         ↓                               │
│  ┌─────────────────────────────────────┐ │
│  │  Arm7R (依賴)                       │ │
│  │  - forwardKinematics()              │ │
│  │  - inverseKinematics()              │ │
│  │  - planTrajectoryCartesian()        │ │
│  │  - getJointPositions()              │ │
│  └─────────────────────────────────────┘ │
└─────────────────────────────────────────┘
```

### 文件職責對照表

| 文件名 | 職責 | 依賴 |
|--------|------|------|
| `Arm7R.m` | 運動學核心（正解、逆解、軌跡規劃、雅可比） | 無（純 MATLAB） |
| `RobotAgent.m` | 服務端主類：TCP 通信 + 指令調度 + 動畫引擎 | `Arm7R.m` |
| `run_robot_agent.m` | 啟動入口：創建對象、啟動服務、保持運行 | `RobotAgent.m`, `Arm7R.m` |
| `send_robot_cmd.ps1` | 客戶端輔助：封裝 PowerShell TCP 發送 | 無（獨立腳本） |
| `robot_agent_cmds.json` | 協議文檔：記錄所有 JSON 指令格式 | 無（文檔） |
| `ROBOT_AGENT_README.md` | 用戶手冊：啟動步驟、指令示例、排錯 | 無（文檔） |

---

## RobotAgent.m 核心設計

### 屬性
- `arm`: `Arm7R` 對象
- `fig`, `ax`, `h_plot`: Figure、Axes、plot3 句柄（用於高效動畫更新）
- `server`: `tcpserver` 對象
- `is_running`: 服務器運行狀態標誌
- `anim_speed`: 動畫倍速（默認 1.0）

### 方法分區

#### 1. 服務器生命周期
- `start(port)`: 啟動 `tcpserver`，綁定回調，打開 Figure
- `stop()`: 關閉 TCP 服務器，清理資源
- `connectionCallback(src, event)`: 客戶端連接/斷開回調（日誌輸出）
- `dataCallback(src, event)`: 數據到達回調。讀取字節流 → UTF-8 字符串 → `jsondecode` → 調度 `executeCommand`

#### 2. 指令調度器
- `executeCommand(cmd_struct)`: 根據 `cmd` 字段分發到具體方法
- 支持的指令：
  - `"home"`: 回到默認零位 `[0,0,0,0,0,0,0]`
  - `"move_to"`: 笛卡爾空間點到點移動。參數：`position` [x,y,z]、`duration`
  - `"joint_move"`: 關節空間點到點移動。參數：`joint`(1-7)、`angle`(deg或rad)、`duration`
  - `"trajectory"`: 預定義軌跡。子類型 `"circle"`、`"line"`、`"square"`。參數：`type`, `radius`/`points`, `duration`, `center`
  - `"pose"`: 直接設置末端位姿。參數：`T` (4x4) 或 `position`+`orientation`
  - `"set_speed"`: 設置動畫播放倍速
  - `"get_status"`: 返回當前關節角、末端位姿（TCP 寫回客戶端）
  - `"plot"`: 靜態重繪當前姿態

#### 3. 動畫引擎（異步雙循環設計）

**核心問題**：軌跡計算（尤其是多點逆運動學）可能耗時數百毫秒，如果與動畫播放串行執行，會導致 Figure 卡頓。

**解決方案：雙循環異步架構**

```
┌────────────────────────────────────────────┐
│         RobotAgent 主事件循環              │
│  ┌─────────────────┐  ┌─────────────────┐  │
│  │   渲染循環      │  │   計算循環      │  │
│  │   (RenderLoop)  │  │   (ComputeLoop) │  │
│  │                 │  │                 │  │
│  │  timer('fixed') │  │  parfeval()     │  │
│  │  ├── 30fps      │  │  或             │  │
│  │  ├── 讀取隊列   │  │  backgroundPool │  │
│  │  └── updatePlot │  │                 │  │
│  │                 │  │  收到指令 →     │  │
│  │  軌跡隊列       │←─┤  後台計算軌跡   │  │
│  │  [q1;q2;q3...]  │  │  完成後推入隊列 │  │
│  └─────────────────┘  └─────────────────┘  │
└────────────────────────────────────────────┘
```

**具體實現**：
- **渲染循環**：獨立的 `timer`，固定 30fps，持續運行。每次觸發時從軌跡隊列中取出下一幀關節角，調用 `updatePlot()`。隊列為空時保持最後姿態不動。
- **計算循環**：TCP 回調收到指令後，**立即返回**，不阻塞。同時啟動 `parfeval`（R2020a+）或 `backgroundPool` 在後台執行軌跡計算。計算完成後的回調將軌跡點推入隊列。
- **隊列機制**：使用 `containers.Map` 或簡單的矩陣緩衝區作為軌跡隊列，支持「覆蓋」（新指令清空舊隊列）或「追加」。

**為什麼這樣設計？**
- `tcpserver` 回調必須快速返回，否則阻塞新指令接收
- `timer` 負責渲染，獨立於計算，保證幀率穩定
- 後台計算（`parfeval`）不佔用 UI 線程，Figure 始終流暢
- `updatePlot(q)`: 高效更新圖形
  - 調用 `arm.getJointPositions(q)` 獲取關節點
  - 使用 `set(h_plot, 'XData', ..., 'YData', ..., 'ZData', ...)` 更新現有 `plot3` 對象，**不重新創建 Figure**
  - 更新末端坐標軸（如有）
  - `drawnow limitrate` 刷新畫面
- `initFigure()`: 初始化 Figure
  - 首次繪製調用 `simplePlot` 風格，但保留所有圖形對象句柄供後續更新
  - 白色背景、透視投影、可旋轉、固定坐標範圍避免跳動

#### 4. 軌跡生成器（輔助）
- `generateCircleTrajectory(center, radius, duration, steps)`: 生成圓形笛卡爾軌跡，轉關節空間
- `generateLineTrajectory(p_start, p_end, duration, steps)`: 直線軌跡
- `ikWithFallback(T_target)`: 帶容錯的逆運動學，失敗時返回最近的可行解或報錯

---

## 自然語言 → JSON 映射（Kimi CLI 側）

Kimi CLI 作為 Agent，負責將用戶輸入翻譯為標準 JSON 指令：

| 用戶輸入示例 | 解析後 JSON |
|-------------|------------|
| "回歸原位" / "home" | `{"cmd": "home", "duration": 2}` |
| "走到 x=500 y=0 z=800" | `{"cmd": "move_to", "position": [500,0,800], "duration": 3}` |
| "關節1轉90度" | `{"cmd": "joint_move", "joint": 1, "angle_deg": 90, "duration": 2}` |
| "畫個圓，半徑200，中心在(500,0,600)" | `{"cmd": "trajectory", "type": "circle", "radius": 200, "center": [500,0,600], "duration": 5}` |
| "走直線到 (600, 100, 700)" | `{"cmd": "trajectory", "type": "line", "target": [600,100,700], "duration": 4}` |
| "加速" / "速度調到2倍" | `{"cmd": "set_speed", "factor": 2.0}` |
| "現在狀態怎樣" | `{"cmd": "get_status"}` |

---

## 開發策略：測試驅動 + 自動修復

### 基本原則
1. **每個 Phase 先寫測試，再寫實現**：`test_phaseX_xxx.m` 先定義預期行為，再填充 `RobotAgent.m` 代碼
2. **測試失敗 → 自動分析 → 修改代碼 → 重測**，循環直到通過
3. **遇到無法解決的問題立即停止並詢問**：不自作主張繞過或忽略

### 何時停止並詢問
| 場景 | 處理方式 |
|------|---------|
| 測試失敗但錯誤原因不明確 | 分析 MATLAB 報錯信息、檢查環境，若 3 次嘗試仍無法定位 → 詢問 |
| 需要改變架構設計（如 `parfeval` 不可用） | 立即停止，詢問是否改為同步計算或其他方案 |
| 視覺測試結果不確定（圖片看起來合理但無法 100% 確認） | 保存圖片並描述觀察，詢問用戶確認 |
| 發現計劃中未預見的技術限制 | 立即停止，說明限制並請求指示 |

### 測試失敗自動修復流程
```
運行 test_phaseX.m
    ↓ 失敗
讀取錯誤信息 + 堆棧
    ↓
定位失敗的測試項（P?-T?）
    ↓
檢查對應實現代碼
    ↓
修改代碼
    ↓
重運行測試
    ↓ 仍然失敗（第 3 次）
停止修復，向用戶報告：
  - 失敗的測試項
  - 錯誤信息
  - 已嘗試的修復措施
  - 請求指示
```

## 實現流程（Step-by-Step）

### Phase 1: 核心骨架（RobotAgent.m 框架）

**測試目標**：Figure 能正確初始化，所有圖形對象句柄有效，關閉後可重建；通過視覺檢查確認繪圖正確。

**視覺測試策略**：
- 每個繪圖測試用例執行後，使用 `print(gcf, '-dpng', 'tests/output/test_phase1_xxx.png')` 保存 Figure 為 PNG
- 通過 `ReadMediaFile` 讀取圖片進行視覺檢查
- 檢查要點：白色背景、藍色連桿線條、關節散點、文字標註清晰、Base/EE 坐標軸可見、透視投影正確

1. **創建類定義與屬性**
   - 定義 `RobotAgent < handle` 類
   - 聲明屬性：`arm`, `fig`, `ax`, `h_link`, `h_joints`, `h_labels`, `h_axes_base`, `h_axes_ee`
   - 聲明狀態屬性：`server`, `port`, `is_running`, `is_busy`, `anim_speed`, `current_q`
   - 聲明異步組件：`render_timer`, `compute_future`, `trajectory_queue`

2. **構造函數與析構函數**
   - 初始化 `Arm7R` 對象
   - 調用 `initFigure()` 創建 Figure 並保留句柄
   - 初始化空軌跡隊列

3. **Figure 初始化（initFigure）**
   - 創建白色背景 Figure
   - 使用 `plot3` 繪製連桿，保留句柄到 `h_link`
   - 使用 `scatter3` 繪製關節，保留句柄到 `h_joints`
   - 使用 `text` 添加標註，保留句柄到 `h_labels`
   - 繪製 Base 和 EE 坐標軸，保留句柄
   - 設置視角、透視、坐標範圍

**Phase 1 測試項目（test_phase1_figure.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P1-T1 | 默認姿態 Figure 初始化 | 彈出 Figure，顯示 7R 機械臂豎直姿態 | `print` 保存 PNG → `ReadMediaFile` 視覺檢查：8 個關節點、藍色連桿、Base/EE 坐標軸 |
| P1-T2 | 不同關節角渲染（q = [pi/4, pi/4, 0, 0, 0, -pi/4, 0]） | 機械臂姿態變化，各關節位置正確 | `print` 保存 PNG → 視覺檢查姿態合理性 |
| P1-T3 | 句柄有效性檢查 | `isvalid(h_link)`, `isvalid(h_joints)` 全部返回 true | 代碼斷言驗證 |
| P1-T4 | Figure 關閉後重建 | 關閉 Figure 後調用 `initFigure`，新 Figure 正常顯示 | `print` 保存 PNG → 視覺檢查與 T1 一致 |

### Phase 2: 通信層（TCP 服務器）

**測試目標**：tcpserver 能正常啟動，PowerShell 客戶端能連接並發送 JSON，回調正確解析。

4. **啟動服務器（start）**
   - 創建 `tcpserver('127.0.0.1', port)`
   - 設置 `configureCallback` 綁定 `dataCallback`
   - 端口被佔用時自動遞增嘗試
   - 打印連接信息到命令行

5. **數據回調（dataCallback）**
   - 使用 `readline` 讀取客戶端數據
   - `jsondecode` 解析為 MATLAB struct
   - 調用 `executeCommand` 分發指令
   - `try-catch` 包裹，錯誤時通過 TCP 返回錯誤信息

6. **指令調度器（executeCommand）**
   - `switch` 語句根據 `cmd` 字段分發
   - 每個 case 驗證必要參數，調用對應方法
   - 返回 `struct('status', 'ok', 'message', ...)`

**Phase 2 測試項目（test_phase2_tcp.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P2-T1 | 服務器啟動（默認端口 12345） | 命令行顯示監聽信息，無報錯 | 檢查命令行輸出 |
| P2-T2 | 端口被佔用時自動遞增 | 端口 12345 被佔用時，自動使用 12346 | 檢查命令行輸出的實際端口號 |
| P2-T3 | PowerShell 發送 `{"cmd":"get_status"}` | MATLAB 正確解析並返回狀態 JSON | PowerShell 接收響應並打印 |
| P2-T4 | 發送無效 JSON | 返回 `{"status":"error","message":"..."}`，服務器不崩潰 | 檢查響應字符串 |
| P2-T5 | 發送缺少 `cmd` 字段的 JSON | 返回 `{"status":"error","message":"missing cmd"}` | 檢查響應字符串 |
| P2-T6 | 連續發送 10 條指令 | 全部正確接收並響應，無丟失 | 計數驗證 |

### Phase 3: 渲染循環（前台）

**測試目標**：timer 穩定運行在 30fps，手動改變 `current_q` 時 Figure 即時更新，無卡頓。

7. **啟動渲染定時器（startRenderLoop）**
   - 創建 `timer('ExecutionMode', 'fixedRate', 'Period', 1/30)`
   - TimerFcn: 讀取 `trajectory_queue` 下一幀 → 更新 `current_q` → 調用 `updatePlot`
   - 隊列為空時不做任何事，保持當前姿態

8. **高效更新圖形（updatePlot）**
   - 檢查 `isvalid(fig)`，無效則 `initFigure`
   - `arm.getJointPositions(q)` 獲取關節點
   - `set(h_link, 'XData', ...)` 更新連桿線條
   - `set(h_joints, 'XData', ...)` 更新散點
   - 循環更新 `h_labels` 位置
   - `getAllTransforms` 獲取 EE 變換，更新 `h_axes_ee`
   - `drawnow limitrate`

**Phase 3 測試項目（test_phase3_render.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P3-T1 | 啟動渲染循環，觀察 5 秒 | timer 持續運行，無錯誤，Figure 保持顯示 | 觀察命令行無報錯 |
| P3-T2 | 幀率穩定性 | 平均幀率 ≈ 30fps，波動 < 5fps | 使用 `tic/toc` 在 TimerFcn 中統計幀間隔 |
| P3-T3 | 手動改變關節角後更新 | 修改 `current_q` 後，Figure 在下一幀即時更新 | `print` 保存前後兩張 PNG → 視覺對比 |
| P3-T4 | 軌跡隊列播放 | 向隊列推送 90 幀軌跡（3 秒），觀察動畫流暢度 | `print` 每隔 1 秒保存一張 PNG → 視覺檢查姿態連續變化 |
| P3-T5 | Figure 關閉後自動重建 | 關閉 Figure，下一幀自動重建並繼續播放 | 觀察新 Figure 彈出，無報錯 |

### Phase 4: 計算循環（後台）

**測試目標**：`parfeval` 能在後台執行 IK 計算，計算期間渲染循環不卡頓，結果正確推入隊列。

9. **後台軌跡計算（computeTrajectoryAsync）**
   - 使用 `parfeval(backgroundPool, @trajectoryGenerator, 1, args...)`
   - `trajectoryGenerator` 函數：根據指令類型生成關節角矩陣
   - 計算完成後回調：將結果推入 `trajectory_queue`
   - 計算期間 `is_busy = true`，防止重複觸發

10. **軌跡生成器實現**
    - `home`: 線性插值到 `zeros(1,7)`
    - `move_to`: 保持當前姿態，位置線性插值，逐點 IK
    - `joint_move`: 單關節線性插值
    - `trajectory/circle`: XY 平面圓形軌跡，逐點 IK
    - `trajectory/line`: 直線軌跡，使用 `planTrajectoryCartesian`
    - 過濾 `NaN` 行，確保隊列中無無效數據

**Phase 4 測試項目（test_phase4_compute.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P4-T1 | `parfeval` 後台計算 100 點 circle 軌跡 | 計算成功返回 100x7 矩陣，無 NaN | 檢查返回矩陣尺寸和 `~any(isnan(...))` |
| P4-T2 | 計算期間渲染循環幀率 | 後台計算期間，渲染幀率仍保持 ≈ 30fps | 同時運行 P3-T2 的幀率統計 |
| P4-T3 | 隊列推送與清空 | 計算完成後軌跡點正確進入隊列，新指令到達時舊隊列被清空 | 檢查隊列長度變化 |
| P4-T4 | 繁忙標誌 `is_busy` | 計算開始時 `is_busy=true`，完成後 `false` | 代碼斷言驗證 |
| P4-T5 | IK 無解處理 | 不可達位姿返回錯誤，不推入隊列，機械臂原地不動 | 檢查命令行錯誤輸出，觀察 Figure 無變化 |

### Phase 5: 指令實現與輔助工具

**測試目標**：每條指令（home, move_to, joint_move, trajectory）發送後，機械臂正確運動到目標位置，正逆運動學一致。

11. **實現各指令方法**
    - `cmdHome`, `cmdMoveTo`, `cmdJointMove`, `cmdTrajectory`
    - `cmdPose`, `cmdSetSpeed`, `cmdGetStatus`
    - 統一錯誤處理：參數缺失、IK 無解、軌跡生成失敗

12. **工具函數**
    - `getField(s, field, default)` 安全讀取 struct 字段
    - `getAllTransforms(q)` 計算所有關節變換矩陣
    - `angleRound` 角度規範化（可選）

**Phase 5 測試項目（test_phase5_commands.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P5-T1 | `home` 指令 | 機械臂回到零位，末端正確 | `forwardKinematics` 驗證末端位姿，視覺檢查 PNG |
| P5-T2 | `move_to` 到 [500, 0, 800] | 末端到達目標位置，姿態保持不變 | 比較前後 `T(1:3,4)`，誤差 < 1mm |
| P5-T3 | `joint_move` 關節1 轉 90 度 | 關節1 = π/2，其他不變 | 檢查 `current_q(1) == pi/2` |
| P5-T4 | `trajectory` circle 半徑 200 | 末端走圓形路徑，閉合 | 每隔 10 幀保存 PNG → 視覺檢查路徑 |
| P5-T5 | `trajectory` line 到 [600,100,700] | 末端走直線 | 保存起點/終點 PNG → 視覺檢查 |
| P5-T6 | `set_speed` 2 倍速 | 動畫播放速度變為 2 倍 | 測量同一段軌跡的播放時間 |
| P5-T7 | `get_status` | 返回正確的關節角、末端位置、旋轉矩陣 | 解析 JSON 響應並驗證數值 |
| P5-T8 | 正逆運動學一致性 | 任意 `q` → `forwardKinematics` → `inverseKinematics` → 位姿誤差 ≈ 0 | 數值比較 `T` 矩陣 |
| P5-T9 | 錯誤指令處理 | 未知指令返回 error，缺失參數返回 error | 檢查響應 JSON 的 status 字段 |

### Phase 6: 啟動與輔助腳本

**測試目標**：一鍵啟動後服務器正常運行，PowerShell 腳本能正確發送指令並接收響應。

13. **run_robot_agent.m**
    - 添加當前目錄到 MATLAB path
    - 創建 `RobotAgent` 對象
    - 調用 `start(port)`
    - 打印使用提示

14. **send_robot_cmd.ps1**
    - 參數化：`[string]$Host="127.0.0.1"`, `[int]$Port=12345`, `[string]$Message`
    - `TcpClient` 連接 → `GetStream()` → `Write` → `Close`
    - 錯誤處理：連接失敗、超時

**Phase 6 測試項目（test_phase6_startup.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P6-T1 | `run_robot_agent` 一鍵啟動 | Figure 彈出，命令行顯示端口信息，無報錯 | 視覺檢查 Figure，檢查命令行輸出 |
| P6-T2 | PowerShell 腳本發送指令 | `send_robot_cmd.ps1 -Message '{"cmd":"home"}'` 成功執行 | PowerShell 返回 exit code 0 |
| P6-T3 | PowerShell 腳本接收響應 | 發送 `get_status` 後正確打印 JSON 響應 | 檢查 PowerShell 輸出字符串 |
| P6-T4 | 指定非默認端口 | `run_robot_agent(12346)` + PowerShell 連接 12346 | 指令正常接收 |
| P6-T5 | 服務器重啟 | 停止後重新 `start`，服務器正常恢復 | 重複 P6-T2 驗證 |

### Phase 7: 文檔

**測試目標**：文檔與代碼一致，每個指令都有正確的 JSON 示例。

15. **robot_agent_cmds.json**
    - 定義每個 `cmd` 的 schema：required 字段、類型、取值範圍
    - 示例 payload

16. **ROBOT_AGENT_README.md**
    - 快速開始：啟動、發送第一條指令
    - 支持的指令列表與示例
    - 故障排查：端口被佔用、Figure 關閉、IK 無解

**Phase 7 測試項目（test_phase7_docs.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P7-T1 | JSON Schema 驗證 | 每個 `cmd` 都有 required 字段定義 | 讀取 `robot_agent_cmds.json` 檢查結構 |
| P7-T2 | 文檔示例可執行 | README 中的每個 JSON 示例都能在 RobotAgent 中正確解析 | 逐條複製示例到 MATLAB 執行 |
| P7-T3 | 錯誤碼文檔完整 | 所有可能的 error status 都有對應說明 | 檢查 README 錯誤碼表格 |

### Phase 8: 集成測試與性能調優

**測試目標**：端到端場景通過，性能指標達標。

17. **功能測試**

**Phase 8 測試項目（test_integration.m）**：
| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P8-T1 | 端到端 home 指令 | Kimi CLI → PowerShell → MATLAB → Figure 動畫到零位 | 全鏈路執行，視覺檢查最終 PNG |
| P8-T2 | 端到端 move_to [500,0,800] | 全鏈路執行，末端到達目標 | 視覺檢查 + `get_status` 驗證 |
| P8-T3 | 端到端 circle 軌跡 | 全鏈路執行，末端畫圓 | 每隔 90 度保存一張 PNG → 視覺檢查 |
| P8-T4 | 快速連續 5 條指令 | 每條指令都被正確執行，無丟失或覆蓋異常 | 觀察動畫順序，檢查響應 |
| P8-T5 | 計算期間發送新指令 | 新指令中斷舊軌跡，機械臂轉向新任務 | 觀察動畫中斷與切換 |
| P8-T6 | 長時間運行穩定性 | 連續運行 10 分鐘，無內存洩漏、無句柄失效 | 觀察命令行無報錯，句柄仍有效 |
| P8-T7 | 性能基準 | 渲染幀率 ≈ 30fps，計算 100 點軌跡 < 2 秒，TCP 響應 < 50ms | `tic/toc` 測量並記錄 |
    - 啟動服務器，確認 Figure 顯示正常
    - 發送 `{"cmd":"home"}`，觀察動畫是否流暢
    - 發送 `{"cmd":"move_to","position":[500,0,800]}`，驗證 IK 正確性
    - 發送 `{"cmd":"trajectory","type":"circle","radius":200}`，觀察圓形軌跡
    - 快速連續發送多條指令，驗證隊列與覆蓋邏輯

18. **性能測試**
    - 監測 `timer` 實際幀率是否穩定 30fps
    - 複雜軌跡（circle, 100 點 IK）計算期間，Figure 是否保持流暢
    - TCP 回調響應時間（應 < 10ms）

---

## 啟動與使用流程

### Step 1: 啟動 MATLAB 服務器
在 MATLAB 命令行運行：
```matlab
run_robot_agent;  % 或: agent = RobotAgent(); agent.start(12345);
```
此時 Figure 窗口彈出，機械臂處於初始姿態，TCP 服務器在後台監聽。

### Step 2: 在 Kimi CLI 發送指令
用戶輸入自然語言，Kimi CLI 解析並通過 PowerShell TCP 發送。

### Step 3: 觀察 Figure
MATLAB 接收到指令後，即時規劃軌跡並播放動畫。

---

## 風險與對策

| 風險 | 對策 |
|------|------|
| `tcpserver` 回調中執行長時間動畫會阻塞新指令接收 | 動畫使用短 `drawnow` 間隔分步執行，或改用 `timer` 對象在回調外異步播放；優先方案是回調中快速啟動 `timer`，不阻塞 TCP |
| Figure 窗口被關閉後句柄失效 | 每次更新前檢查 `isvalid(fig)`，若無效則自動重新初始化 |
| 逆運動學無解（err=1） | `ikWithFallback` 捕獲錯誤，通過 TCP 向 Kimi CLI 返回錯誤信息，保持機械臂原地不動 |
| 多條指令快速堆積 | 引入簡單的指令隊列或「忙碌」標誌，當前動畫未完成時新指令排隊或覆蓋 |
| TCP 端口被佔用 | `start()` 中捕獲異常，自動嘗試備用端口（如 12346），並打印實際端口 |

---

## 兼容性說明
- **MATLAB 版本**: 需要 R2020b 或更高版本（`tcpserver` 內置函數）。用戶環境為 R2023b，完全滿足。
- **工具箱**: 僅需基礎 MATLAB（`tcpserver` 在基礎包中）。`simplePlot` 同樣無額外依賴。
- **Robotics Toolbox**: 本次完全不依賴，即使未安裝也能正常運行。

---

## 擴展預留
- 未來如需雙臂協同，可擴展 `RobotAgent` 持有兩個 `Arm7R` 對象，指令中添加 `"arm": "left"` / `"right"` 字段。
- 未來如需更復雜的自然語言理解，可將解析邏輯獨立為 `parse_instruction.m` 或遷移至 MATLAB 端執行。
