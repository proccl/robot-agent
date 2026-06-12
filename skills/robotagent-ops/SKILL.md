---
name: robotagent-ops
description: 操作 RobotAgent 文件監聽架構機械臂系統的指南。涵蓋自然語言指令、自定義姿態調整、常見問題排查與最佳實踐。
---

# RobotAgent 操作指南

## 快速導航

| 主題 | Reference 文件 |
|------|---------------|
| 系統架構、文件目錄、日誌機制、PAUSE 處理 | [architecture-reference.md](references/architecture-reference.md) |
| 腳本模板（home / move_to / 姿態調整 / 複合指令） | [script-templates-reference.md](references/script-templates-reference.md) |
| 姿態調整算法、Rodrigues 公式、home 軸向對應 | [pose-adjustment-reference.md](references/pose-adjustment-reference.md) |
| 常見問題排查表、18 條開發經驗 | [troubleshooting-reference.md](references/troubleshooting-reference.md) |
| 函數簽名確認表（quinticTrajectory / animateRobot / planTrajectoryCartesian） | [function-signatures-reference.md](references/function-signatures-reference.md) |

## 一句話架構

RobotAgent 採用**文件監聽架構**：AI 生成 `.m` 腳本 → 投遞到 `incoming/` → MATLAB `timer` 每 0.5s 掃描並 `run()` 執行 → 結果通過 `diary` 記錄到 `logs/`。

- **啟動方式**：在 MATLAB Editor 雙擊 `robotagent.m` 或點 Run
- **核心組件**：`robotagent.m`（啟動）、`processIncomingCommands.m`（timer 回調執行）、`Arm7R.m`（運動學）

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
3. **注意**：`log_*.txt` 的文件創建時間可能早於內容對應的腳本投遞時間，**務必以日誌內容的 `[RX] cmd_...` 行確認對應關係**，不要只按文件時間排序
4. **向用戶匯報分析結果**

> ❌ **禁止行為**：投放腳本後只回覆「已寫入，請觀察」而不主動讀取 log。

---

## 快速操作

### 方式一：AI 直接投遞（自然語言）⭐ 推薦

直接告訴 AI 自然語言指令，AI 會直接生成 `.m` 文件到 `incoming/` 並執行。

**範例**：「回零位」、「z向下200」、「走到 500 0 800」、「x向前100，然後 z軸繞x軸轉至水平」

**複合指令範例**：「同時 y 向 -200，z 向 -100，繞末端 x 軸轉至水平且末端 z 指向正 x，然後向前移 500、向下移 600，再繞末端 z 軸畫 r=100 圓一圈」→ AI 會拆分為多個 `cmd_*.m` 腳本，按創建時間順序執行。

> 舊版的 `parseNaturalLanguage.m` 與 `generate_robot_cmd.m` 已在 v0.0.4 重構中移除，現只保留 AI 直接寫代碼模式。

## 支持的自然語言指令

| 指令 | 示例 | 對應 `cmd` |
|------|------|-----------|
| 回零位 | `回零位`、`home` | `home` |
| 絕對移動 | `走到 500 0 800`、`move to (500,0,800)` | `move_to` |
| 相對移動 | `z向下200`、`向下移動100`、`x向移動-50` | `relative_move` |
| 關節運動 | `關節3轉45度`、`joint 2 30 deg` | `joint_move` |
| 畫圓 | `畫圓 半徑200`、`circle radius 200` | `trajectory` (circle) |
| 帶避障移動 | `走到 300 -300 600 避開障礙物` | `move_to`（當 `obstacle_avoidance_enabled=true` 時使用 `planTrajectoryWithObstacle`） |
| 移向障礙物中心 | `移向球心` | 使用 `ud.obstacle.center` 作為目標；若目標在球內會 PAUSE |
| 沿方向移動 | `向球心方向移動 500` | 計算當前位置到球心的單位方向向量，乘以 500 |
| 查詢狀態 | `status`、`姿態` | `get_status` |

> 姿態調整指令（如「Z 軸指向 +X」）**不支持自然語言解析**，需由 AI 直接生成自定義腳本。詳見 [pose-adjustment-reference.md](references/pose-adjustment-reference.md)。

## 常用代碼片段

### Home（關節空間）
```matlab
q_traj = quinticTrajectory(current_q, zeros(1,7), 5, 30);
animateRobot(fig, q_traj, 30);
fprintf('[Done] home completed.\n');
```

### 相對移動（笛卡爾空間，保持姿態）
```matlab
T_start = arm.forwardKinematics(current_q);
T_end = T_start;
T_end(1:3,4) = T_end(1:3,4) + [0; 0; -200];  % Z-200
[~, q_traj, ~] = arm.planTrajectoryCartesian(T_start, T_end, 150, 0, 5);
% nan 處理見 script-templates-reference.md
animateRobot(fig, q_traj, 30);
fprintf('[Done] relative_move completed.\n');
```

## 常見陷阱速查

| 陷阱 | 正確做法 |
|------|---------|
| `planTrajectoryCartesian` 返回值順序錯誤 | `[~, q_traj, ~] = ...`（記憶口訣：「T 在前，q 在中」） |
| `planTrajectoryCartesian` 傳入 `current_q` | 第一參數必須是 `T_start`（4×4 位姿矩陣），用 `arm.forwardKinematics(current_q)` 獲取 |
| `assignin('base', 'arm', ...)` 後腳本找不到 `arm` | `run()` 在函數工作空間執行，改用 `arm = fig.UserData.arm;` |
| `quinticTrajectory` 傳 `steps` 作為第三參數 | 第三參數是 `T`（總時間秒），`steps` 由函數內部自動計算 |
| `animateRobot` 傳 `arm` 作為第二參數 | 簽名是 `animateRobot(fig, q_traj, fps)` |
| 腳本中覆蓋 `fig.UserData` | `initRobotFigure` 存了圖形句柄，啟動腳本只能追加字段，不能整體覆蓋 |
| 投遞後立即讀取 log 卻找不到對應記錄 | `timer` 最長 0.5s 才觸發，加上動畫 duration，需等待足夠時間再讀取 |
| 只看 log 文件時間判斷是否為最新指令 | 文件創建時間可能早於內容對應的腳本時間，應以 `[RX] cmd_...` 內容為準 |
| 避障腳本中硬編碼障礙物位置 | 用戶可能修改了 `robotagent.m` 中的 `obstacle.center` | 始終使用 `ud.obstacle.center` 動態讀取 |
| 目標點設在障礙物內部 | `manipulatorRRT` 要求起終點都無碰撞 | 將目標設在球外，或臨時關閉避障 |
| 避障開關沒打開 | `robotagent.m` 中 `enable_obstacle_avoidance` 默認為 `false` | 啟動前改為 `true`，或運行中設置 `fig.UserData.obstacle_avoidance_enabled = true` |

> 完整排查表與 18 條開發經驗見 [troubleshooting-reference.md](references/troubleshooting-reference.md)。

---

*最後更新：2026年6月13日（新增：避障架構、動態目標、方向移動、RRT 目標在碰撞中、工具箱檢測經驗、碰撞幾何限制）*
