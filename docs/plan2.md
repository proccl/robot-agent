# Robot-Agent 架構重構計劃：文件隊列橋接模式

## 目標

用戶**直接運行啟動腳本**（無需在 MATLAB 命令行輸入任何指令）→ 彈出 Figure → 在 **Kimi CLI** 輸入自然語言 → **Kimi 即時生成 .m 代碼寫入文件** → **MATLAB 端自動檢測並執行** → Figure 機械臂按 **五次多項式（Quintic Polynomial）速度規劃** 的連續軌跡運動。

無需 TCP、無需 Shell 調用 MATLAB、無需 MATLAB Engine API。

---

## 前置步驟：保存 Plan 到項目目錄

**執行任何 Phase 之前，先將本 plan 文件複製到項目 docs 目錄**：

```
源: C:\Users\HW\.kimi\plans\phil-coulson-tigra-wiccan.md
目標: D:\Document\code\Matlab\robot-agent\docs\plan_refactor_filewatch.md
```

**目的**：確保在 Kimi CLI 會話上下文過長或壓縮後，仍可隨時通過 `ReadFile` 讀取 plan，避免遺忘當前 Phase 的目標與測試項。

---

## 核心機制：文件系統隊列

### 架構圖

```
┌─────────────────────────────┐         ┌─────────────────────────────┐
│   MATLAB 窗口               │         │      Kimi CLI 窗口          │
│  ┌───────────────────────┐  │         │  ┌─────────────────────┐    │
│  │ 用戶點擊運行          │  │         │  │ 用戶輸入自然語言    │    │
│  │ robotagent.m          │  │         │  │  • 解析意圖          │    │
│  │  → Figure 彈出        │  │         │  │  • 生成 .m 代碼      │    │
│  │  → timer 啟動監聽     │  │         │  │  • 寫入 incoming/    │    │
│  └───────────────────────┘  │         │  └──────────┬──────────┘    │
│              │              │         │             │ 寫文件         │
│              │ timer 0.5s   │         │             ▼               │
│              ▼              │         │  ┌─────────────────────┐    │
│  ┌───────────────────────┐  │         │  │  cmd_xxx.m          │    │
│  │ 掃描 incoming/        │◄─┼─────────┼──┤  (Quintic 軌跡代碼) │    │
│  │ 發現新文件 → run()    │  │ 文件系統 │  └─────────────────────┘    │
│  └───────────────────────┘  │         └─────────────────────────────┘
│              │              │
│              ▼              │
│  ┌───────────────────────┐  │
│  │ Figure 機械臂動畫更新 │  │
│  └───────────────────────┘  │
└─────────────────────────────┘
```

### 通信協議

- **寫入方（Kimi CLI）**：生成 .m 代碼後，寫入 `incoming/cmd_<timestamp>_<random>.m`
- **消費方（MATLAB）**：`timer` 每 0.5 秒掃描 `incoming/` 目錄，按文件名排序執行，執行後刪除
- **錯誤處理**：執行失敗的腳本移至 `incoming/failed/` 並記錄錯誤日誌

---

## 核心運動學約定

### Quintic Polynomial 速度規劃

**所有關節空間運動必須使用五次多項式（Quintic Polynomial）插值**，確保位置、速度、加速度在起點與終點均連續且為零。

**多項式形式**：
```
q(t) = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
```

**邊界條件**（t=0 與 t=T 時速度、加速度均為 0）：
| 條件 | t=0 | t=T |
|------|-----|-----|
| 位置 | q(0) = q0 | q(T) = q1 |
| 速度 | q̇(0) = 0 | q̇(T) = 0 |
| 加速度 | q̈(0) = 0 | q̈(T) = 0 |

**解析係數**：
```matlab
a0 = q0;
a1 = 0;
a2 = 0;
a3 = 10 * (q1 - q0) / T^3;
a4 = -15 * (q1 - q0) / T^4;
a5 = 6 * (q1 - q0) / T^5;
```

**離散軌跡生成**（MATLAB 代碼模板）：
```matlab
T = duration;           % 總時間（秒），用戶未指定時默認 5
fps = 30;               % 渲染幀率
steps = round(T * fps); % 總幀數
t_vec = linspace(0, T, steps);

q_traj = zeros(steps, 7);
for j = 1:7
    dq = q_target(j) - q_current(j);
    a3 = 10 * dq / T^3;
    a4 = -15 * dq / T^4;
    a5 = 6 * dq / T^5;
    q_traj(:, j) = q_current(j) + a3 * t_vec.^3 + a4 * t_vec.^4 + a5 * t_vec.^5;
end
```

### 默認時間約定

| 指令類型 | 用戶未指定 duration 時的默認值 |
|---------|-------------------------------|
| `home` | 5 秒 |
| `move_to` | 5 秒 |
| `joint_move` | 5 秒 |
| `trajectory/circle` | 5 秒 |
| `trajectory/line` | 5 秒 |
| `pose` | 5 秒 |

---

## Phase 1：提取可視化為獨立函數

> **開始前**：讀取 `D:\Document\code\Matlab\robot-agent\docs\plan_refactor_filewatch.md` 的 Phase 1 與測試項目，確認目標與驗收標準。

### 內容

從舊 `RobotAgent.m` 提取以下無狀態函數到 `src/`：

| 函數 | 輸入 | 輸出 | 職責 |
|------|------|------|------|
| `initRobotFigure(arm, q)` | `Arm7R`, `1×7` | `figure` 句柄 | 創建 Figure，繪製初始姿態，設置 UserData |
| `updateRobotFigure(fig, q)` | `figure`, `1×7` | 無 | 更新連桿、關節、標註、EE 坐標軸 |
| `animateRobot(fig, q_traj, fps)` | `figure`, `N×7`, `fps` | 無 | 逐幀播放動畫序列 |

這些函數不依賴任何全局狀態，僅通過 `figure` 句柄通信。

### 測試項目（test_phase1_figure.m）

| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P1-T1 | `initRobotFigure(Arm7R(), zeros(1,7))` | 彈出 Figure，顯示 7R 機械臂零位姿態 | `isvalid(fig)` 為 true，視覺檢查 PNG |
| P1-T2 | Figure UserData 完整性 | `fig.UserData` 包含 `arm`, `current_q` | 斷言驗證字段存在且類型正確 |
| P1-T3 | `updateRobotFigure(fig, [pi/4,pi/4,0,0,0,-pi/4,0])` | Figure 更新為新姿態，無閃爍 | `print` 保存 PNG，視覺對比前後差異 |
| P1-T4 | 句柄有效性檢查 | `isvalid(fig)` 返回 true，所有圖形對象 `isvalid` | 遍歷檢查 `h_link`, `h_joints`, `h_labels`, `h_axes_ee` |
| P1-T5 | Figure 關閉後重建 | 關閉 Figure 再次調用 `initRobotFigure`，新 Figure 正常 | 視覺檢查新 Figure 與 T1 一致 |
| P1-T6 | `animateRobot` 播放 30 幀軌跡 | 動畫流暢，30 幀後機械臂到達終點 | `tic/toc` 計時 ≈ 1 秒，末幀 FK 驗證位姿 |

---

## Phase 2：重構 robotagent（文件監聽版，一鍵啟動）

> **開始前**：讀取 `D:\Document\code\Matlab\robot-agent\docs\plan_refactor_filewatch.md` 的 Phase 2 與測試項目，確認目標與驗收標準。

### 內容

創建新的 `robotagent.m`（腳本，不是函數），用戶直接點擊運行即可：

```matlab
%% robotagent.m — 機械臂可視化與指令監聽啟動腳本
%   使用方式：在 MATLAB Editor 中直接點擊「運行」按鈕，或雙擊此文件
%   效果：自動添加路徑 → 初始化 Figure → 啟動文件監聽 timer

% 獲取本腳本所在目錄，自動添加 src 路徑
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'src'));

% 初始化機械臂與 Figure
arm = Arm7R();
current_q = zeros(1, 7);
fig = initRobotFigure(arm, current_q);
fig.UserData = struct('arm', arm, 'current_q', current_q);

% 創建指令監聽目錄
incoming_dir = fullfile(scriptDir, 'incoming');
if ~exist(incoming_dir, 'dir')
    mkdir(incoming_dir);
end

% 啟動定時器監聽
t = timer('ExecutionMode', 'fixedRate', 'Period', 0.5, ...
          'TimerFcn', @(~,~) processIncomingCommands(incoming_dir, fig));
start(t);

fprintf('========================================\n');
fprintf('RobotAgent started.\n');
fprintf('Figure: RobotAgent - 7R Arm\n');
fprintf('Watching: %s\n', incoming_dir);
fprintf('========================================\n');
```

### 測試項目（test_phase2_filewatch.m）

| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P2-T1 | 直接點擊運行 `robotagent.m` | Figure 彈出，命令行顯示監聽路徑，timer 運行，無需手動輸入任何指令 | `isvalid(fig)`, `isvalid(t)`，檢查命令行輸出 |
| P2-T2 | `incoming/` 目錄自動創建 | 目錄存在且可寫 | `exist(incoming_dir, 'dir')` |
| P2-T3 | 手動寫入測試 .m 文件 | timer 檢測到文件並執行，執行後文件被刪除 | 創建 `cmd_test.m`（內容 `disp('hello')`），觀察命令行輸出，檢查文件是否存在 |
| P2-T4 | 多個文件按時間順序執行 | 寫入 `cmd_A.m`, `cmd_B.m`，A 先執行，B 後執行 | 文件內容設置全局標誌變量，執行後檢查順序 |
| P2-T5 | 錯誤腳本處理 | 語法錯誤的 .m 文件執行失敗後移至 `incoming/failed/` | 故意寫入錯誤腳本，檢查 `failed/` 目錄 |
| P2-T6 | 執行期間 timer 不並發觸發 | 動畫播放 5 秒期間，不會同時執行第二個文件 | 寫入長耗時腳本，再寫入第二個腳本，觀察是否等待 |
| P2-T7 | `incoming/` 為空時無異常 | 無文件時 timer 正常觸發，無報錯 | 持續運行 10 秒，確認命令行無錯誤 |
| P2-T8 | MATLAB 命令行可並行使用 | robotagent 運行期間，可在命令行執行其他 MATLAB 指令 | 在命令行輸入 `a=1+2; disp(a)` 確認正常 |

---

## Phase 3：創建代碼生成器（含 Quintic 規劃）

> **開始前**：讀取 `D:\Document\code\Matlab\robot-agent\docs\plan_refactor_filewatch.md` 的 Phase 3 與測試項目，確認目標與驗收標準。

### 內容

新增 `src/quinticTrajectory.m` 與 `src/generate_robot_cmd.m`：

#### `quinticTrajectory.m`（核心軌跡生成）

```matlab
function q_traj = quinticTrajectory(q0, q1, T, fps)
    % quinticTrajectory 五次多項式關節空間軌跡規劃
    %   q0: 起始關節角 (1x7)
    %   q1: 終止關節角 (1x7)
    %   T:  總時間 (秒)，默認 5
    %   fps: 幀率，默認 30
    %   q_traj: Nx7 軌跡矩陣
    
    if nargin < 3 || isempty(T), T = 5; end
    if nargin < 4 || isempty(fps), fps = 30; end
    
    steps = max(2, round(T * fps));
    t = linspace(0, T, steps);
    
    q_traj = zeros(steps, 7);
    for j = 1:7
        dq = q1(j) - q0(j);
        a3 = 10 * dq / T^3;
        a4 = -15 * dq / T^4;
        a5 = 6 * dq / T^5;
        q_traj(:, j) = q0(j) + a3 * t.^3 + a4 * t.^4 + a5 * t.^5;
    end
end
```

#### `generate_robot_cmd.m`（代碼生成器）

```matlab
function filepath = generate_robot_cmd(cmd_struct, output_dir)
    if nargin < 2
        scriptDir = fileparts(mfilename('fullpath'));
        output_dir = fullfile(scriptDir, '..', 'incoming');
    end
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS_FFF');
    filename = sprintf('cmd_%s_%s.m', timestamp, cmd_struct.cmd);
    filepath = fullfile(output_dir, filename);
    
    code = buildCode(cmd_struct);
    fid = fopen(filepath, 'w');
    fwrite(fid, code); fclose(fid);
end
```

#### 模板中的軌跡生成片段（以 home 為例）

```matlab
%% 自動生成: home
duration = 5;  % 用戶未指定時默認 5 秒
fig = findobj('Type', 'figure', 'Name', 'RobotAgent - 7R Arm');
if isempty(fig) || ~isvalid(fig), error('Figure not found. Run robotagent first.'); end

ud = fig.UserData;
arm = ud.arm;
current_q = ud.current_q;
q_target = zeros(1, 7);

% Quintic Polynomial 軌跡規劃
q_traj = quinticTrajectory(current_q, q_target, duration, 30);

% 動畫播放
for i = 1:size(q_traj, 1)
    updateRobotFigure(fig, q_traj(i, :));
    drawnow limitrate;
    pause(1/30);
end

fig.UserData.current_q = q_target;
fprintf('[Done] home (duration=%.1fs)\n', duration);
```

### 測試項目（test_phase3_generator.m）

| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P3-T1 | `quinticTrajectory` 起點終點 | `q_traj(1,:)` ≈ `q0`, `q_traj(end,:)` ≈ `q1` | 數值比對，誤差 < 1e-6 |
| P3-T2 | `quinticTrajectory` 起點速度 | 數值微分 `diff(q_traj(1:3,:))./dt` ≈ 0 | 前三點速度 < 1e-3 rad/s |
| P3-T3 | `quinticTrajectory` 終點速度 | 數值微分最後三點速度 ≈ 0 | 末三點速度 < 1e-3 rad/s |
| P3-T4 | `quinticTrajectory` 對稱性 | `q0→q1` 與 `q1→q0` 的軌跡應關於中點對稱 | 反轉比對驗證 |
| P3-T5 | `generate_robot_cmd(struct('cmd','home'))` | 在 `incoming/` 生成 `cmd_xxx_home.m` | `exist(filepath, 'file')`，讀取內容驗證 |
| P3-T6 | home 模板生成的腳本可執行 | 執行後 Figure 回到零位，用時約 5 秒 | `tic/toc` ≈ 5s，`current_q` ≈ zeros |
| P3-T7 | move_to 模板生成的腳本可執行 | 執行後末端到達目標位置 | FK 驗證末端位置與目標誤差 < 1mm |
| P3-T8 | joint_move 模板（角度/弧度） | `angle_deg=true` 時正確轉換 | 檢查執行後關節角是否為預期值 |
| P3-T9 | trajectory_circle 模板 | 生成圓形軌跡腳本，末端路徑近似圓形 | 採樣多點 FK，計算各點到圓心距離誤差 < 5mm |
| P3-T10 | trajectory_line 模板 | 生成直線軌跡腳本，末端走直線 | 起點終點 FK 驗證，中間點共線性檢查 |
| P3-T11 | get_status 模板 | 執行後正確輸出當前關節角與末端位姿 | 比對輸出與 `forwardKinematics(current_q)` |
| P3-T12 | 用戶指定 duration=3 | 生成腳本中 `duration=3`，執行用時約 3 秒 | 讀取文件內容驗證，`tic/toc` 驗證 |
| P3-T13 | 用戶未指定 duration | 生成腳本中 `duration=5`（默認值） | 讀取文件內容驗證 |
| P3-T14 | 文件名唯一性 | 連續調用 10 次，文件名不重複 | `unique()` 驗證 |

---

## Phase 4：自然語言解析層

> **開始前**：讀取 `D:\Document\code\Matlab\robot-agent\docs\plan_refactor_filewatch.md` 的 Phase 4 與測試項目，確認目標與驗收標準。

### 內容

在 Kimi CLI 側實現自然語言到 `cmd_struct` 的映射：

| 用戶輸入示例 | 解析結果 |
|-------------|---------|
| "home" / "回原位" / "歸零" | `struct('cmd','home','duration',5)` |
| "走到 500 0 800" / "move to (500, 0, 800)" | `struct('cmd','move_to','position',[500,0,800],'duration',5)` |
| "走到 500 0 800 用 3 秒" | `struct('cmd','move_to','position',[500,0,800],'duration',3)` |
| "關節1轉90度" / "joint 1 90 deg" | `struct('cmd','joint_move','joint',1,'angle',90,'angle_deg',true,'duration',5)` |
| "畫圓半徑200" / "circle radius 200" | `struct('cmd','trajectory','type','circle','radius',200,'duration',5)` |
| "現在姿態怎樣" / "status" | `struct('cmd','get_status')` |

### 測試項目（test_phase4_nlp.m）

| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P4-T1 | "home" → 正確解析 | `cmd='home', duration=5` | 字符串匹配驗證 |
| P4-T2 | "回到原位" → 正確解析 | `cmd='home', duration=5` | 同上了 |
| P4-T3 | "走到 500 0 800" → 提取數字 | `position=[500,0,800], duration=5` | 正則提取驗證 |
| P4-T4 | "move to (500, 0, 800)" → 提取數字 | `position=[500,0,800], duration=5` | 括號與逗號格式容錯 |
| P4-T5 | "走到 500 0 800 用 3 秒" → 提取時間 | `position=[500,0,800], duration=3` | 中文時間提取 |
| P4-T6 | "關節1轉90度" → 提取關節與角度 | `joint=1, angle=90, angle_deg=true, duration=5` | 中文數字提取驗證 |
| P4-T7 | "joint 2 -45" → 負數與弧度 | `joint=2, angle=-45, angle_deg=true, duration=5` | 默認為角度單位 |
| P4-T8 | "畫圓 半徑 200 用 5 秒" → 多參數 | `type='circle', radius=200, duration=5` | 多字段提取 |
| P4-T9 | 無法識別的輸入 → 錯誤提示 | 返回錯誤信息，不生成文件 | 驗證錯誤處理路徑 |
| P4-T10 | 端到端：自然語言 → 生成文件 → MATLAB 執行 | 全鏈路貫通，Figure 正確響應 | 模擬完整流程驗證 |

---

## Phase 5：清理舊架構與文檔更新

> **開始前**：讀取 `D:\Document\code\Matlab\robot-agent\docs\plan_refactor_filewatch.md` 的 Phase 5 與測試項目，確認目標與驗收標準。

### 內容

刪除或歸檔：
- `src/RobotAgent.m`（舊 TCP 類）
- `scripts/run_robot_agent.m`（舊啟動腳本）
- `scripts/send_robot_cmd.ps1`（PowerShell TCP 客戶端）
- `send_cmd.m`, `temp_*.m`（舊 TCP 腳本）

更新：
- `AGENTS.md` → 新架構說明
- `README.md` → 新使用流程
- `docs/robot_agent_cmds.json` → 更新為代碼模板參考

### 測試項目（test_phase5_cleanup.m）

| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P5-T1 | 舊 `RobotAgent.m` 已刪除 | 文件不存在於 `src/` | `exist('src/RobotAgent.m', 'file') == 0` |
| P5-T2 | 舊 TCP 腳本已刪除 | `send_cmd.m`, `temp_*.m` 不存在 | `dir` 檢查 |
| P5-T3 | 新 `robotagent.m` 可一鍵啟動 | 在 Editor 中點擊運行，Figure 彈出，無需手動輸入 | 直接執行驗證 |
| P5-T4 | `addpath('src')` 後所有新函數可訪問 | `which initRobotFigure`, `which updateRobotFigure`, `which generate_robot_cmd`, `which quinticTrajectory` 返回路徑 | 函數路徑驗證 |
| P5-T5 | README 文檔與代碼一致 | README 中無 TCP 相關說明，有文件隊列模式說明，提及 Quintic 規劃 | 人工審查 |
| P5-T6 | AGENTS.md 與代碼一致 | AGENTS.md 描述新架構與默認 5s 約定 | 人工審查 |

---

## Phase 6：集成與穩定性測試

> **開始前**：讀取 `D:\Document\code\Matlab\robot-agent\docs\plan_refactor_filewatch.md` 的 Phase 6 與測試項目，確認目標與驗收標準。

### 內容

端到端驗證完整閉環，並測試邊界條件與長期穩定性。

### 測試項目（test_phase6_integration.m）

| 編號 | 測試內容 | 預期結果 | 驗證方法 |
|------|---------|---------|---------|
| P6-T1 | 端到端 home 指令 | Kimi CLI 解析 "home" → 生成文件 → MATLAB 執行 → Figure 回零位，耗時約 5 秒 | 全鏈路執行，FK 驗證，`tic/toc` 驗證 |
| P6-T2 | 端到端 move_to | "走到 500 0 800" 全鏈路執行，末端到達目標 | 執行前後 `forwardKinematics` 比較 |
| P6-T3 | 端到端 move_to 指定 3 秒 | 執行耗時約 3 秒，軌跡仍為 Quintic | `tic/toc` 驗證時間，速度曲線驗證起終點為 0 |
| P6-T4 | 端到端 circle | "畫圓半徑200" 全鏈路執行，末端畫圓 | 採樣多點驗證圓形路徑 |
| P6-T5 | 快速連續 5 條指令 | 5 個文件按順序執行，無丟失、無覆蓋異常 | 每條指令設置不同目標，檢查最終姿態 |
| P6-T6 | 新指令中斷舊動畫 | 動畫播放期間寫入新文件，新指令排隊等待或中斷 | 觀察行為是否符合設計 |
| P6-T7 | 長時間運行穩定性 | robotagent 連續運行 5 分鐘，定時發送 10 條指令 | 無內存洩漏、無句柄失效、命令行無報錯 |
| P6-T8 | Figure 誤關後恢復 | 關閉 Figure 後發送指令，觀察錯誤提示，重新啟動後恢復正常 | 錯誤信息明確，重啟後全鏈路恢復 |
| P6-T9 | 無效位姿容錯 | 發送不可達位姿（如 `[5000, 0, 800]`），不崩潰 | 錯誤捕獲，機械臂原地不動，錯誤日誌記錄 |
| P6-T10 | Quintic 軌跡平滑性 | 動畫過程中機械臂運動流暢，無頓挫或跳變 | 視覺觀察 + 關節角差分驗證速度連續 |
| P6-T11 | 性能基準 | 文件檢測延遲 < 1 秒，代碼生成 < 100ms，IK 計算 < 500ms | `tic/toc` 測量各環節耗時 |

---

## 執行順序與檢查清單

```
□ Step 0: 將本 plan 複製到 docs/plan_refactor_filewatch.md
    |
□ Phase 1: 提取可視化函數 (initRobotFigure, updateRobotFigure, animateRobot)
    |   開始前: ReadFile(docs/plan_refactor_filewatch.md) 確認 Phase 1 測試項
    |   完成後: 運行 test_phase1_figure.m，全部通過
    |
□ Phase 2: 重構 robotagent（文件監聽版，一鍵啟動）
    |   開始前: ReadFile(docs/plan_refactor_filewatch.md) 確認 Phase 2 測試項
    |   完成後: 運行 test_phase2_filewatch.m，全部通過
    |
□ Phase 3: 創建 Quintic 軌跡生成器 + 代碼生成器
    |   開始前: ReadFile(docs/plan_refactor_filewatch.md) 確認 Phase 3 測試項
    |   完成後: 運行 test_phase3_generator.m，全部通過
    |
□ Phase 4: 自然語言解析層
    |   開始前: ReadFile(docs/plan_refactor_filewatch.md) 確認 Phase 4 測試項
    |   完成後: 運行 test_phase4_nlp.m，全部通過
    |
□ Phase 5: 清理舊架構與文檔更新
    |   開始前: ReadFile(docs/plan_refactor_filewatch.md) 確認 Phase 5 測試項
    |   完成後: 運行 test_phase5_cleanup.m，全部通過
    |
□ Phase 6: 集成與穩定性測試
        開始前: ReadFile(docs/plan_refactor_filewatch.md) 確認 Phase 6 測試項
        完成後: 運行 test_phase6_integration.m，全部通過
```

---

## 用戶使用流程

### 1. 一鍵啟動 robotagent（MATLAB GUI）

**用戶無需在 MATLAB 命令行輸入任何指令**，只需：

1. 在 MATLAB 的 **Current Folder** 中找到 `robot-agent` 文件夾
2. 雙擊打開 `robotagent.m`（或在 Editor 中直接點擊 **運行** 按鈕 ▶）
3. 自動完成：添加路徑 → 初始化 Figure → 啟動文件監聽 timer

效果：
- Figure 彈出，顯示機械臂零位姿態
- 命令行顯示：
  ```
  ========================================
  RobotAgent started.
  Figure: RobotAgent - 7R Arm
  Watching: D:\...\robot-agent\incoming
  ========================================
  ```
- MATLAB 命令行**不阻塞**，可繼續使用（但用戶無需操作）

### 2. 發送指令（Kimi CLI）

在 **Kimi CLI（另一窗口）**輸入自然語言：
```
讓機械臂末端移動到 (500, 0, 800)，用 3 秒
```

Kimi CLI 內部處理：
1. 解析為 `struct('cmd','move_to','position',[500,0,800],'duration',3)`
2. 生成對應 .m 代碼（內含 `quinticTrajectory` 調用）
3. 寫入 `robot-agent/incoming/cmd_xxx.m`

### 3. 自動執行（MATLAB 端）

MATLAB 的 `timer` 在 0.5 秒內檢測到新文件，自動 `run()` 執行。機械臂按 **五次多項式平滑軌跡** 運動到目標位置，用時 3 秒（若未指定則為 5 秒）。

---

## 風險與對策

| 風險 | 對策 |
|------|------|
| 動畫執行期間 timer 再次觸發，導致並發 | 執行前設置 `is_busy` 標誌，忙時跳過新文件檢查 |
| 用戶在 MATLAB 中誤關 Figure | 每個生成腳本開頭檢查 `isvalid(fig)`，無效則報錯提示重新運行 `robotagent` |
| Kimi CLI 與 MATLAB 路徑不一致 | 使用相對路徑 `incoming/`，要求 Kimi CLI 在項目根目錄運行 |
| 大量指令堆積 | 限制 `incoming/` 目錄文件數量，超過 10 個時暫停接受新指令 |
| 生成代碼語法錯誤 | 模板經嚴格測試，參數使用 `sprintf` 格式化 |
| Quintic 軌跡超過關節限制 | 在 `quinticTrajectory` 中檢查中間點是否超出 `q_limit`，超過則報錯 |
