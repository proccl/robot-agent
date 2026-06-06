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
| 常見問題排查表、9 條開發經驗 | [troubleshooting-reference.md](references/troubleshooting-reference.md) |
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
3. **向用戶匯報分析結果**

> ❌ **禁止行為**：投放腳本後只回覆「已寫入，請觀察」而不主動讀取 log。

---

## 快速操作

### 方式一：AI 直接投遞（自然語言）⭐ 推薦

直接告訴 AI 自然語言指令，AI 會直接生成 `.m` 文件到 `incoming/` 並執行。

**範例**：「回零位」、「z向下200」、「走到 500 0 800」、「x向前100，然後 z軸繞x軸轉至水平」

### 方式二：MATLAB 自然語言解析

```matlab
generate_robot_cmd(parseNaturalLanguage('回零位'), 'incoming');
```

### 方式三：直接結構體

```matlab
cmd = struct('cmd','move_to','position',[500,0,800],'duration',5);
generate_robot_cmd(cmd, 'incoming');
```

## 支持的自然語言指令

| 指令 | 示例 | 對應 `cmd` |
|------|------|-----------|
| 回零位 | `回零位`、`home` | `home` |
| 絕對移動 | `走到 500 0 800`、`move to (500,0,800)` | `move_to` |
| 相對移動 | `z向下200`、`向下移動100`、`x向移動-50` | `relative_move` |
| 關節運動 | `關節3轉45度`、`joint 2 30 deg` | `joint_move` |
| 畫圓 | `畫圓 半徑200`、`circle radius 200` | `trajectory` (circle) |
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

> 完整排查表與 9 條開發經驗見 [troubleshooting-reference.md](references/troubleshooting-reference.md)。

---

*最後更新：2026年6月6日（重構 + 新增圓軌跡模板、多段指令排隊機制、經驗10~13）*
