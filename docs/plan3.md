# RobotAgent 軌跡執行失敗問題總結與改進計劃（修訂版）

## 一、問題總結

當前系統在執行**笛卡爾空間連續軌跡**時頻繁失敗，具體表現：

| 指令 | 失敗現象 | 錯誤信息 |
|------|---------|---------|
| `home` (Cartesian) | 腳本歸檔到 `failed/` | `Too many unreachable points along Cartesian trajectory` |
| `relative_move z-500` (自定義 Cartesian) | 腳本歸檔到 `failed/` | `target unreachable at step 1` |
| `composite` (Cartesian) | 可能成功或失敗，依賴當前姿態 | 姿態插值導致中間點 `nan` |

## 二、根因分析

### 根因 1：`inverseKinematics` 的嚴格判斷是數學正確的，需補充註釋

`inverseKinematics` 是解析閉式解，以下兩處判斷是在驗證**數學上是否存在實數解**：

```matlab
% Line 249: 驗證 t2 是否存在實數解
if (1 - ((-obj.m + obj.i) / rou1)^2) < 0

% Line 282: 驗證 t34 是否存在實數解  
if (1 - (E/rou2)^2) < 0
```

**真正問題**：`forwardKinematics` 經過 7 個矩陣連乘後產生浮點誤差，導致輸出位姿偏離了「精確解析解對應的位姿」。當此帶有誤差的位姿傳回 `inverseKinematics` 時，數學上確實無解。這是 **FK 精度問題**，而非 IK 判斷問題。

**結論**：IK 的 `< 0` 判斷**不應修改**，但應補充註釋說明其數學意義。

### 根因 2：`planTrajectoryCartesian` 對 IK 失敗的處理策略不當

- 當前實現遇到 `nan` 時自動跳過或報錯，沒有給用戶決策機會
- 用戶明確要求：**出現無解時應停下來問我**，而非自動處理
- 用戶已安裝 **Robotics System Toolbox**，`planTrajectoryCartesian` 應優先使用 **SLERP（四元數插值）**，姿態插值本身是可靠的

### 根因 3：`processIncomingCommands` 錯誤信息過於簡陋

僅打印 `ME.message`，無法區分是「數學無解」還是「腳本邏輯錯誤」，也無法遠程定位具體步驟。

### 根因 4：成功執行的腳本被直接刪除，無歷史記錄

當前 `processIncomingCommands` 在執行成功後直接 `delete(cmd_path)`，用戶無法回溯過去執行過的指令內容。

---

## 三、改進方案

### 建立「遇到無解時暫停並通知用戶」機制 + 歷史記錄 + Figure 位姿顯示 + 更新 Skill

不改 IK 數學邏輯，改變錯誤處理策略；同時保存所有歷史腳本、在 Figure 顯示位姿、更新項目 skill。

**實施細節**：

1. **補充 `inverseKinematics` 註釋**
   - 在 Line 249 和 282 的 `if` 前加入註釋，說明這是在驗證解析解的數學存在性
   - 提醒後續開發者：若此處觸發，通常是輸入位姿的浮點精度不足

2. **新增 `incoming_history/` 目錄保存歷史腳本**
   - 修改 `processIncomingCommands.m`：執行成功後，先將腳本複製到 `incoming_history/`（按時間戳命名），再 `delete(cmd_path)`
   - 失敗的腳本仍移至 `incoming/failed/`，同時可選複製一份到 `incoming_history/`（標記為 failed）
   - 歷史目錄可用於回溯、審查、調試

3. **改寫生成腳本模板（不改 `Arm7R.m`）**
   - **`home` 指令**：直接使用 `quinticTrajectory(current_q, zeros(1,7), ...)` 回零位，簡單可靠，不強求笛卡爾直線
   - **其他移動指令**（`move_to`、`relative_move`、`composite`、`trajectory`）：使用 `planTrajectoryCartesian`（利用 Robotics System Toolbox 做 SLERP）
   - **第一步繞過 FK/IK 不自洽**：若 `q_traj(1,:)` 為 `nan`，用已知的 `current_q` 填充
   - 遍歷軌跡時，若遇到後續 `nan`：
     - 不調用 `error()`，不讓腳本崩潰
     - 將當前已播放的 `q_traj(1:i-1, :)` 正常動畫播放完畢
     - 將暫停狀態寫入 `incoming/pause_*.mat`
     - 在 MATLAB 命令窗口打印 `[PAUSE] IK unreachable at step X`

4. **建立跨進程暫停通知機制**
   - `processIncomingCommands` 掃描到 `pause_*.mat` 時，暫停釋放 `is_busy` 或標記 paused 狀態
   - **用戶決策僅通過 Kimi CLI 完成**，MATLAB 端僅顯示暫停狀態，不接受手動輸入
   - Kimi CLI 端檢測到 pause 文件後，讀取狀態並詢問用戶：
     - 「繼續」（跳過不可達點，從最後有效位姿繼續）
     - 「調整目標」（修改目標位姿後重新生成腳本）
     - 「回退」（用 quinticTrajectory 回退到關節空間插值）

5. **在 Figure 中顯示當前 4×4 位姿矩陣**
   - 修改 `initRobotFigure.m`：在 Figure 角落增加 `uicontrol('Style','text')` 或 `annotation`，用於顯示末端位姿矩陣
   - 修改 `updateRobotFigure.m`：每次更新時計算 `T_ee = arm.forwardKinematics(q)`，將 4×4 矩陣格式化為字符串並更新顯示
   - 顯示格式示例：
     ```
     T = [R11 R12 R13 Px
          R21 R22 R23 Py
          R31 R32 R33 Pz
          0   0   0   1 ]
     ```
   - 使用等寬字體（`Monospaced`）確保對齊

6. **增強 `processIncomingCommands` 錯誤日誌**
   - `fprintf` 輸出到 **MATLAB Command Window**（命令行窗口）
   - 輸出格式：`[ERR] <file>:<line> — <message>`，例如：`[ERR] cmd_xxx.m:15 — Target unreachable`
   - 同時追加寫入 `incoming/error_log.txt`，讓 Kimi CLI 端可遠程讀取診斷

7. **更新項目 skill**
   - 更新 `skills/robotagent-ops/SKILL.md`，補充：
     - 笛卡爾軌跡暫停機制的操作流程
     - `incoming_history/` 與 `incoming/failed/` 的用途
     - Figure 中位姿矩陣的讀取方法
     - 常見 `PAUSE` / `ERR` 信息的排查指南

---

## 四、建議實施步驟

1. 補充 `Arm7R.m` `inverseKinematics` 的兩處註釋（不改邏輯）
2. 創建 `incoming_history/` 目錄，修改 `processIncomingCommands.m` 的保存/刪除邏輯
3. 設計並實現 `incoming/pause_*.mat` 狀態文件格式與讀寫邏輯
4. 重寫 AI 生成腳本的模板：**`home` 用 `quinticTrajectory`**，其他移動指令用 `planTrajectoryCartesian`（繞過第一步 nan + 中途暫停機制）
5. 修改 `processIncomingCommands.m`，支持 pause 狀態檢測與詳細錯誤日誌
6. 修改 `initRobotFigure.m` 與 `updateRobotFigure.m`，增加 4×4 位姿矩陣顯示
7. 更新項目 skill：`skills/robotagent-ops/SKILL.md`
8. 測試 `home`、`relative_move`、`composite` 在「可達」與「不可達」場景下的行為

---

## 六、具體測試項目

| 編號 | 測試項目 | 前置條件 | 預期結果 |
|------|---------|---------|---------|
| T1 | `home` 指令從非零位回零（連續軌跡） | 當前姿態為 Z 水平指向 +X | 使用 `quinticTrajectory` 生成 150 幀平滑關節空間軌跡，逐幀動畫播放，**不是一步到位**；5 秒內回到 `zeros(1,7)`，Figure 位姿顯示實時更新 |
| T2 | `relative_move z-500` | 從 home 位置開始 | 末端沿直線向下 500 mm，姿態不變，`incoming_history/` 出現該腳本副本 |
| T3 | `move_to (500,0,800)` | 從 home 位置開始 | 末端沿直線移動到目標位置，保持 home 姿態，無錯誤 |
| T4 | `composite`（下降+前移+Z轉水平） | 從 home 位置開始 | 成功執行，末端沿直線移動且姿態同步旋轉，最終 Z 軸指向 +X |
| T5 | 不可達姿態觸發暫停 | 從 Z 水平姿態執行 `home` via `planTrajectoryCartesian` | 動畫播放至最後有效點後暫停，生成 `incoming/pause_*.mat`，命令行輸出 `[PAUSE]`，Kimi CLI 端收到通知 |
| T6 | 暫停後「回退」選項 | T5 暫停後選擇「回退」 | 生成 quintic 版本腳本，成功回到 home，無錯誤 |
| T7 | 歷史記錄完整性 | 連續執行 T1~T4 | `incoming_history/` 包含 4 個腳本文件，`incoming/` 為空，`incoming/failed/` 為空 |
| T8 | 錯誤日誌輸出 | 手動觸發一個錯誤腳本（如語法錯誤） | MATLAB Command Window 顯示 `[ERR] <file>:<line> — <message>`，`incoming/error_log.txt` 存在對應記錄 |
| T9 | Figure 位姿顯示更新 | 執行任意動畫指令 | Figure 角落的 4×4 矩陣實時更新，數值與 `arm.forwardKinematics(current_q)` 一致，字體對齊正確 |
| T10 | `incoming_history` 失敗標記 | 執行一個明確不可達的 `move_to`（如 `(10000,0,0)`） | 腳本歸檔到 `incoming/failed/`，`incoming_history/` 中副本帶 `failed` 標記（或同名區分） |

## 七、風險提示

- 不改 IK 數學邏輯，保留了解析解的嚴格性；代價是腳本模板更複雜，需要處理暫停-恢復流程
- 依賴用戶已有的 Robotics System Toolbox 做 SLERP，若未來環境變化需確認工具箱可用性
- `incoming_history/` 長期累積可能佔用較多磁盤空間，未來可考慮定期清理或限制保留數量
