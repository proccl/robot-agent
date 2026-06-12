# 測試重構計劃：AI 直接寫代碼模式

## 背景

`generate_robot_cmd.m` 和 `parseNaturalLanguage.m` 已移除。現有 4 個保留測試測試底層能力，需要重命名使編號連續，並新增端到端集成測試與綜合指令測試。

## 一、現有測試重命名（編號連續化）

| 原文件名 | 新文件名 | 說明 |
|----------|----------|------|
| `test_phase5_cleanup.m` | `test_phase4_cleanup.m` | Phase 4: 文檔與清理驗證 |

## 二、新增測試：`test_phase5_e2e.m`（端到端集成測試）

測試模式：**測試腳本中用 `fprintf` 寫入完整機械臂控制代碼**（模擬 AI 生成），調用 `processIncomingCommands` 執行並驗證。

| 用例 | 模擬 AI 代碼內容 | 驗證點 |
|------|------------------|--------|
| **P5-T1: home** | `quinticTrajectory` + `updateRobotFigure` 循環 | `current_q` ≈ 零向量 |
| **P5-T2: move_to** | FK→改位置→IK→`quinticTrajectory` + 動畫 | 末端到達 `[500,0,800]` |
| **P5-T3: relative_move** | FK→改單軸偏移→IK→`quinticTrajectory` + 動畫 | 末端偏移正確 |
| **P5-T4: joint_move** | 直接改單關節→`quinticTrajectory` + 動畫 | `current_q(joint)` 正確 |
| **P5-T5: 多段排隊** | 連續投放 3 個腳本，驗證 `datenum` 順序執行 | 依次執行，結果正確 |
| **P5-T6: PAUSE 機制** | 軌跡中間插入 NaN，驗證播放到最後有效點停止 | 不報錯，停在有效點 |

## 三、新增測試：`test_phase6_complex.m`（綜合指令測試）

模擬用戶輸入複合自然語言指令，AI 生成**多個 `.m` 腳本**依次投放，驗證整個鏈路：

> **指令**：「同時 y 向 -200，z 向 -100，繞末端 x 軸轉至水平，且末端 z 方向指向正 x 方向，然後同時向前移 500，向下移 600。然後繞末端 z 軸轉 r=100 的圓轉一圈，最後回零位」

### 分解為 4 個腳本（AI 生成）

| # | 腳本內容 | 技術要點 |
|---|----------|----------|
| **腳本1** | 複合姿態+位置調整：Y-200, Z-100 + 繞末端X軸轉水平（Z指+Z→Z指+X 需繞Y軸-90°） | Rodrigues 旋轉 + IK，用 `planTrajectoryCartesian` 或逐點 IK |
| **腳本2** | 相對移動：前進 500（+X），下降 600（-Z） | 保持當前姿態，改位置後 IK |
| **腳本3** | 圓軌跡：繞末端 Z 軸，r=100 畫一圈 | 自定義逐點 IK（Template F），圓心偏移方向 = 旋轉軸的「下一個」軸（X→Y） |
| **腳本4** | 回零位 | `quinticTrajectory(current_q, zeros, duration, fps)` |

### 驗證點

- 4 個腳本按創建時間順序全部執行完成
- 每段執行後 `current_q` 合理（無 NaN，在關節限位內）
- 最終回到零位
- 圓軌跡段不報 `[PAUSE]` 或 `[ERR]`
- 日誌輸出包含 4 個 `[Done]`

## 四、執行步驟

1. 將本計劃備份到 `docs/test_refactor_plan.md`
2. 重命名 `test_phase5_cleanup.m` → `test_phase4_cleanup.m`
3. 新建 `tests/test_phase5_e2e.m`（6 個端到端用例）
4. 新建 `tests/test_phase6_complex.m`（綜合指令 4 段腳本）
5. 更新 `AGENTS.md` 測試策略表格和運行命令
6. 運行全部 6 個測試，記錄結果
7. 若有失敗，分析原因後報告用戶決策
