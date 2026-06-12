# RobotAgent 避障模塊開發計劃

## 目標

在現有文件隊列橋接架構的 RobotAgent 系統中增加避障能力：

1. 在機械臂前方生成一個半徑 `r=100 mm` 的紅色小球作為障礙物。
2. 實現機械臂末端與小球之間的干涉檢測。
3. 當用戶通過自然語言指示機械臂移動且軌跡會與障礙物碰撞時，自動生成繞過障礙物的軌跡。
4. `robotagent.m` 中提供兩個 bool 開關：
   - `obstacle_enabled`：是否顯示障礙物
   - `obstacle_avoidance_enabled`：是否啟用避障規劃
5. 在 `src/` 中新增獨立避障模塊。
6. 重組 `tests/`：新建文件夾分別存放原有 Phase 測試與新的避障測試。
7. 在 `docs/` 中備份本計劃原文。

---

## 設計決策

### 工具箱選擇

經評估，可選的避障相關工具包包括：

| 工具包 | 優勢 | 劣勢 |
|--------|------|------|
| **Robotics System Toolbox**（MathWorks 官方） | 提供 `checkCollision`、`manipulatorRRT`、`collisionSphere` 等現成函數，與 MATLAB 集成最好 | 需確認已安裝 |
| Robotics Toolbox（Peter Corke） | 功能豐富，含碰撞檢測與路徑規劃 | 第三方工具箱，需額外安裝 |
| Navigation Toolbox | 提供 RRT、PRM 等規劃器 | 主要面向移動機器人，用於 7R 機械臂需大量適配 |
| 純 MATLAB 自主實現 | 無額外依賴 | 需自行實現碰撞檢測與路徑規劃，開發量較大 |

**用戶選擇**：優先使用 **Robotics System Toolbox**。實施第一步將運行 `ver('Robotics System Toolbox')` 確認可用性；若不可用，暫停並詢問是否改為純 MATLAB 自主實現。

### 1. 障礙物表示

- 在 `initRobotFigure.m` 中初始化一個紅色球體，使用 `[X,Y,Z] = sphere(20)` + `surf` 或 `scatter3` 繪製。
- 默認位置設在機械臂前方世界坐標 `[800, 0, 0]`（位於 home 位置末端可達範圍內，且處於常見移動路徑上）。
- 半徑 `100 mm`。
- 圖形句柄存入 `fig.UserData.h_obstacle`。
- 可見性由 `fig.UserData.obstacle_enabled` 控制；關閉時隱藏小球，但保留句柄。

### 2. 碰撞檢測（基於 Robotics System Toolbox）

- 使用工具箱現成函數 `checkCollision`：
  - `checkCollision(robot_tree, q, obstacle_sphere)` 檢測給定關節角 `q` 下，機械臂（含連桿）是否與球體障礙物發生碰撞。
  - 輸出：`is_collision`（bool）、`separation_dist`（分離距離，負值表示穿透）。
- 新增 `src/checkRobotObstacleCollision.m` 作為包裝層：
  - 輸入：`robot_tree`（rigidBodyTree）、`q`（1x7 關節角）、`obstacle`（結構體，含 `center` 和 `radius`）
  - 輸出：`is_collision`（bool）、`min_dist`（最短距離）
  - 內部調用 `collisionSphere(obstacle.radius)` 創建障礙物，設置 `Pose` 為球心位置，再調用工具箱的 `checkCollision`。
  - 注意：為避免與 Robotics System Toolbox 的 `checkCollision` 函數同名衝突，文件名定為 `checkRobotObstacleCollision.m`。
- 優勢：
  - 檢測範圍覆蓋整個機械臂連桿，不僅是末端。
  - 無需自己寫幾何距離計算。
  - 可輕鬆擴展到多個障礙物（`worldObjects` 數組）。

### 3. 避障軌跡規劃（基於 Robotics System Toolbox）

用戶選擇優先使用工具箱方案，因此避障模塊基於 **Robotics System Toolbox** 實現：

- 新增 `src/planTrajectoryWithObstacle.m`：
  - 輸入：`arm`、`current_q`、`T_target`、`obstacle`、可選 `duration`
  - 輸出：`q_traj`（Nx7）、`avoided`（bool，是否發生避障）
  - 依賴：`Robotics System Toolbox`（`rigidBodyTree`, `collisionSphere`, `checkCollision`, `manipulatorRRT`）
  - 算法步驟：
    1. 在 `robotagent.m` 啟動時，將 `Arm7R` 的 DH 參數轉換為 `rigidBodyTree`，存入 `fig.UserData.robot_tree`。
    2. 用 `collisionSphere(obstacle.radius)` 表示障礙物，通過 `Pose` 屬性設置球心位置。
    3. 調用 `arm.planTrajectoryCartesian(T_start, T_target, steps, 0, duration)` 生成原始軌跡。
    4. 對原始軌跡逐點調用 `checkCollision(robot_tree, q_i, obstacle_sphere)` 檢測機械臂（含連桿）與球的碰撞。
    5. 若無碰撞，直接返回原始軌跡。
    6. 若存在碰撞，使用 `manipulatorRRT` 在關節空間規劃避障路徑：
       - 起始狀態：`current_q`
       - 目標狀態：對 `T_target` 求 IK 得到的 `q_target`
       - 碰撞對象：`obstacle_sphere`
       - 輸出：`path`（關節空間路徑）
    7. 若 `manipulatorRRT` 規劃失敗或超時，**不嘗試其他繞行方向**，直接輸出 `[PAUSE] RRT planning failed. Obstacle may block all paths.` 並保持當前位姿。
    8. 對 RRT 路徑進行平滑與插值（如需要），轉換為 `q_traj`。
    9. 再次用 `checkCollision` 驗證最終軌跡。

- **工具箱缺失時的處理**：
  - 若啟動時檢測到無 `Robotics System Toolbox`，則：
    - `obstacle_avoidance_enabled` 自動設為 `false`
    - 輸出 warning：「未檢測到 Robotics System Toolbox，避障功能已禁用」
  - 或者提供一個簡化的自主實現作為 fallback（可選，根據用戶後續決定）。

### 4. robotagent.m 開關

- 在 `robotagent.m` 頂部增加手動設定變量：
  ```matlab
  % 用戶可手動設定是否啟用避障規劃
  %   true  : 啟用（需要 Robotics System Toolbox）
  %   false : 禁用（默認）
  enable_obstacle_avoidance = false;
  ```
- 啟動時根據該變量構建 `rigidBodyTree` 並設置 `fig.UserData.obstacle_avoidance_enabled`。
- 用戶也可以在 MATLAB 命令行手動切換：
  ```matlab
  fig.UserData.obstacle_enabled = false;            % 隱藏紅球
  fig.UserData.obstacle_avoidance_enabled = false;  % 關閉避障規劃
  ```

### 5. 自然語言腳本生成

- 在 `skills/robotagent-ops/references/script-templates-reference.md` 新增「模板 G：避障移動」。
- 當 `fig.UserData.obstacle_avoidance_enabled` 為 true 時，AI 生成的 `move_to` / `relative_move` 腳本調用 `planTrajectoryWithObstacle` 而非直接 `planTrajectoryCartesian`。
- 腳本示例：
  ```matlab
  ud = fig.UserData;
  arm = ud.arm;
  current_q = ud.current_q;
  T_cur = arm.forwardKinematics(current_q);
  T_target = T_cur;
  T_target(1:3, 4) = [x; y; z];
  
  if ud.obstacle_avoidance_enabled && isfield(ud, 'obstacle')
      [q_traj, avoided] = planTrajectoryWithObstacle(arm, current_q, T_target, ud.obstacle, duration);
      if avoided, fprintf('[Info] obstacle avoided\n'); end
  else
      [~, q_traj, ~] = arm.planTrajectoryCartesian(T_cur, T_target, steps, 0, duration);
  end
  % nan 處理與播放...
  ```

### 6. 測試組織

- 重組 `tests/` 目錄：
  ```
  tests/
  ├── phases/               % 原有 Phase 測試
  │   ├── run_all_tests.m
  │   ├── test_phase1_figure.m
  │   ├── test_phase2_filewatch.m
  │   ├── test_phase3_generator.m
  │   ├── test_phase4_cleanup.m
  │   ├── test_phase5_e2e.m
  │   └── test_phase6_complex.m
  └── obstacle/             % 避障專項測試
      ├── test_obstacle_collision.m
      ├── test_obstacle_avoidance_move.m
      └── test_obstacle_disabled.m
  ```
- 更新 `run_all_tests.m` 中路徑引用。
- 新增測試：
  - `test_obstacle_collision.m`：驗證 `checkRobotObstacleCollision` 在末端進入球內時返回 true
  - `test_obstacle_avoidance_move.m`：給定障礙物擋住直線路徑，驗證 `planTrajectoryWithObstacle` 生成的軌跡不碰撞且能到達目標
  - `test_obstacle_disabled.m`：驗證開關關閉時不進行避障檢查

### 7. 文檔備份

- 將本計劃原文保存為 `docs/plan_obstacle_avoidance_v0.0.6.md`。
- 更新 `README.md` 和 `AGENTS.md`，增加避障模塊說明。

---

## 文件變更清單

### 新增文件
1. `src/checkRobotObstacleCollision.m` —— 包裝 Robotics System Toolbox 的 `checkCollision`，檢測機械臂與球體障礙物
2. `src/planTrajectoryWithObstacle.m` —— 使用 `manipulatorRRT` 規劃避障路徑
3. `src/buildRobotTree.m` —— 將 `Arm7R` 的 DH 參數轉換為 `rigidBodyTree`
3. `tests/phases/` 目錄（移入現有 `test_phase*.m` 和 `run_all_tests.m`）
4. `tests/obstacle/test_obstacle_collision.m`
5. `tests/obstacle/test_obstacle_avoidance_move.m`
6. `tests/obstacle/test_obstacle_disabled.m`
7. `docs/plan_obstacle_avoidance_v0.0.6.md`

### 修改文件
1. `robotagent.m` —— 增加障礙物開關與 `obstacle` 結構體
2. `src/initRobotFigure.m` —— 繪製紅色小球，句柄存入 UserData
3. `src/updateRobotFigure.m` —— 根據開關更新小球可見性
4. `skills/robotagent-ops/references/script-templates-reference.md` —— 新增避障模板 G
5. `README.md` / `AGENTS.md` —— 更新避障功能說明

### 刪除/移動
- 將現有 `tests/test_phase*.m` 和 `tests/run_all_tests.m` 移動到 `tests/phases/`，不刪除內容。

---

## 實施步驟

> 原則：
> 1. **每個階段結束前必須有對應測試，測試通過後才進入下一階段。**
> 2. **測試失敗時先自主調試，嘗試修復直到成功；若遇到無法解決的問題，立即停止並向用戶匯報，等待用戶決策。**

### 第一階段：備份與準備

1. **備份本計劃原文並檢查工具箱**
   - 將當前 plan 文件複製到 `docs/plan_obstacle_avoidance_v0.0.6.md`。
   - 在 MATLAB 中運行 `ver('Robotics System Toolbox')` 確認工具箱是否已安裝。
   - 若已安裝，繼續後續工具箱方案；若未安裝，暫停並向用戶報告，詢問是否改為純 MATLAB 自主實現方案。

**階段測試**：
- 確認 `docs/plan_obstacle_avoidance_v0.0.6.md` 存在且內容完整。
- 確認 `ver('Robotics System Toolbox')` 返回有效信息。

### 第二階段：基礎設施

2. **工具箱檢測與 rigidBodyTree 構建**
   - 在 `robotagent.m` 中檢測 `Robotics System Toolbox` 是否存在。
   - 編寫 `src/buildRobotTree.m`，將 `Arm7R` 的 DH 參數準確轉換為 `rigidBodyTree`。
   - 在 `robotagent.m` 啟動時調用 `buildRobotTree`，將結果存入 `fig.UserData.robot_tree`。
   - 若工具箱缺失，設置 `fig.UserData.has_robotics_toolbox = false` 並輸出 warning。

**階段測試**：
- 編寫 `tests/obstacle/test_build_robot_tree.m`：
  - 驗證 `buildRobotTree` 返回的 `rigidBodyTree` 非空。
  - 驗證 ` Arm7R.forwardKinematics(q)` 與 `robot_tree.getTransform(homeConfig, 'body7')` 結果一致（至少對 home 位姿）。
- 測試通過後再進入下一階段。

3. **可視化障礙物**
   - 修改 `src/initRobotFigure.m`：在 Figure 中繪製紅色小球（`sphere` + `surf` 或 `scatter3`）。
   - 修改 `src/updateRobotFigure.m`：根據 `fig.UserData.obstacle_enabled` 更新小球可見性。
   - 修改 `robotagent.m`：增加 `obstacle` 結構體、`obstacle_enabled` 和 `obstacle_avoidance_enabled` 開關。

**階段測試**：
- 編寫 `tests/obstacle/test_obstacle_visualization.m`：
  - 驗證 `robotagent.m` 啟動後 Figure 中存在紅色小球。
  - 驗證切換 `fig.UserData.obstacle_enabled` 時小球可見性正確變化。
- 測試通過後再進入下一階段。

### 第三階段：核心算法

4. **碰撞檢測**
   - 編寫 `src/checkRobotObstacleCollision.m`：
     - 輸入 `robot_tree`、`q`、`obstacle` 結構體
     - 使用 `collisionSphere` + `checkCollision` 檢測機械臂與球的碰撞
     - 返回 `is_collision` 和最短距離

**階段測試**：
- 編寫 `tests/obstacle/test_obstacle_collision.m` 並運行通過：
  - 末端遠離小球時返回 `is_collision = false`。
  - 末端進入小球內部時返回 `is_collision = true`。
- 測試通過後再進入下一階段。

5. **避障軌跡規劃**
   - 編寫 `src/planTrajectoryWithObstacle.m`：
     - 無碰撞時直接返回 `planTrajectoryCartesian` 結果
     - 有碰撞時使用 `manipulatorRRT` 規劃關節空間避障路徑
     - 對 RRT 路徑插值並驗證
     - 若 `manipulatorRRT` 規劃失敗或超時，直接輸出 `[PAUSE]` 並保持當前位姿

**階段測試**：
- 編寫 `tests/obstacle/test_obstacle_avoidance_move.m` 並運行通過：
  - 給定障礙物擋住直線路徑，驗證生成的軌跡不碰撞且最終到達目標。
- 測試通過後再進入下一階段。

6. **開關與降級邏輯**
   - 在 `robotagent.m` 中確保：工具箱缺失時自動禁用避障。

**階段測試**：
- 編寫 `tests/obstacle/test_obstacle_disabled.m` 並運行通過：
  - `obstacle_avoidance_enabled = false` 時，直線軌跡允許穿過障礙物。
  - 工具箱缺失時 `fig.UserData.has_robotics_toolbox = false` 且避障開關自動關閉。
- 測試通過後再進入下一階段。

### 第四階段：重組與文檔

7. **測試目錄重組**
   - 創建 `tests/phases/` 和 `tests/obstacle/`。
   - 將現有 `test_phase*.m` 和 `run_all_tests.m` 移入 `tests/phases/`。
   - 更新 `run_all_tests.m` 中的路徑引用。

**階段測試**：
- 運行 `tests/phases/run_all_tests.m`，驗證原有測試在新路徑下仍可正常運行。
- 測試通過後再進入下一階段。

8. **Skill 與文檔更新**
   - 在 `skills/robotagent-ops/references/script-templates-reference.md` 新增模板 G（基於工具箱）。
   - 更新 `README.md` 和 `AGENTS.md`，說明避障功能依賴 Robotics System Toolbox。
   - （plan 原文已在步驟 1 備份，此處不再重複）

**階段測試**：
- 確認 `script-templates-reference.md` 中新增模板 G 內容完整。
- 確認 README/AGENTS 中避障說明與實現一致。
- 測試通過後再進入下一階段。

### 第五階段：驗證與發布

9. **全量測試**
   - 運行 `tests/phases/run_all_tests.m` 確保原有測試不受影響。
   - 運行 `tests/obstacle/*.m` 確保新測試通過。
   - 在無 Robotics System Toolbox 的環境中驗證降級行為。

**階段測試**：
- 所有 Phase 測試通過。
- 所有 Obstacle 測試通過。
- 測試通過後再進入發布階段。

10. **（暫不執行）Git 提交與 Release**
    - 根據用戶要求，本次實施暫不進行 Git 提交與 Release。
    - 待後續確認後，再執行：
      - 提交為 `v0.0.6: Add obstacle avoidance module`
      - 打 tag `v0.0.6`
      - 使用 token 推送到 GitHub（參見 github-manage skill）
      - 通過 GitHub API 創建 Release

---

## 風險與回退

| 風險 | 影響 | 回退策略 |
|------|------|---------|
| Robotics System Toolbox 缺失 | `manipulatorRRT` 和 `checkCollision` 無法使用 | 實施第一步檢測到後暫停，詢問用戶是否改為純 MATLAB 自主實現；若用戶堅持工具箱方案則無法繼續 |
| RRT 規劃失敗或超時 | 無法找到避障路徑 | **不嘗試其他繞行方向**，直接輸出 `[PAUSE]` 並向用戶匯報，保持當前位姿 |
| rigidBodyTree 與 Arm7R 運動學不一致 | 碰撞檢測或規劃結果錯誤 | 仔細核對 DH 參數轉換；用已知位姿驗證正向運動學 |
| 開關誤關導致碰撞 | 動畫穿過障礙物 | 默認 `obstacle_avoidance_enabled = false`，需用戶主動開啟，降低風險 |
| 測試目錄重組破壞 `run_all_tests.m` | 原有測試無法運行 | 仔細更新路徑引用；在提交前全量運行 |
| 小球繪製影響 Figure 性能 | 初始化變慢 | 使用低精度球體（`sphere(10)`）或 `scatter3` 替代 `surf` |

---

## 驗收標準

- `robotagent.m` 啟動後，Figure 中出現紅色小球。
- `fig.UserData.obstacle_enabled` 和 `fig.UserData.obstacle_avoidance_enabled` 可控制小球顯示與避障行為。
- 當目標點與當前點之間的直線軌跡穿過小球時，啟用避障後機械臂末端會繞過小球。
- `tests/phases/run_all_tests.m` 全部通過。
- `tests/obstacle/*.m` 全部通過。
- 在無 Robotics System Toolbox 的環境中，`robotagent.m` 正確禁用避障並輸出 warning。
