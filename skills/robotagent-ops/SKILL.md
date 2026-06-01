---
name: robotagent-ops
description: 操作 RobotAgent 文件監聽架構機械臂系統的指南。涵蓋自然語言指令、自定義姿態調整、常見問題排查與最佳實踐。
---

# RobotAgent 操作指南

## 架構概述

RobotAgent 採用 **文件監聽架構**（非 TCP）：

```matlab
用戶自然語言 → AI 生成 .m 腳本 → 投遞到 incoming/
                                     ↓
MATLAB timer (0.5s) → 掃描 → run() 執行 → 刪除/歸檔 failed/
```

- **啟動方式**：在 MATLAB Editor 雙擊 `robotagent.m` 或點 Run
- **核心組件**：
  - `robotagent.m`：一鍵啟動，初始化 Figure + 啟動 timer
  - `processIncomingCommands.m`：timer 回調，按文件名排序執行
  - `generate_robot_cmd.m`：結構化指令 → 可執行 `.m` 文件
  - `parseNaturalLanguage.m`：中英自然語言 → `cmd_struct`
  - `quinticTrajectory.m`：五次多項式關節空間軌跡

## 快速操作

### 方式一：AI 直接投遞（自然語言）⭐ 推薦

直接告訴 AI 自然語言指令，AI 會直接生成 `.m` 文件到 `incoming/` 並執行。

**範例**：
- 「回零位」
- 「z向下200」
- 「走到 500 0 800」
- 「x向前100，然後 z軸繞x軸轉至水平」

### 方式二：MATLAB 自然語言解析

在 MATLAB Command Window 輸入：

```matlab
generate_robot_cmd(parseNaturalLanguage('回零位'), 'incoming');
generate_robot_cmd(parseNaturalLanguage('z向下200'), 'incoming');
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

### 姿態調整指令（需手寫腳本）

以下指令 **不支持自然語言解析**，需由 AI 直接生成自定義 `.m` 腳本：

| 動作 | 核心算法 |
|------|---------|
| X 軸水平（Y 軸不動） | 繞 Y 軸旋轉 `atan2(X_z, X_x)`，Rodrigues 公式 |
| Z 軸水平（Y 軸不動） | 繞 Y 軸旋轉 `atan2(-Z_z, X_z)`，Rodrigues 公式 |
| Z 軸反向（X 軸不動） | 繞 X 軸旋轉 180°：`R = I + 2*K^2` |
| Z 軸水平（繞 X 軸） | 繞 X 軸旋轉 `atan2(-Z_z, Z_y)`，Rodrigues 公式 |
| Z 軸指向 +X（Y 軸不動） | 繞 Y 軸旋轉 `-90°`：`R_y(-90°) * [0;0;-1] = [1;0;0]` |

### ⚠️ 關鍵洞察：home 位置下的軸向對應

當前 **home 位姿**下：
- 末端 **X 軸** = `[0, 1, 0]` → 與**世界 Y 軸完全重合**
- 末端 **Y 軸** = `[1, 0, 0]` → 與**世界 X 軸完全重合**
- 末端 **Z 軸** = `[0, 0, -1]` → 朝下

**這意味著**：
- 用戶說「繞末端 X 軸旋轉」→ **等價於繞世界 Y 軸旋轉**
- 用戶說「繞末端 Y 軸旋轉」→ **等價於繞世界 X 軸旋轉**

**繞末端 X 軸（= 世界 Y 軸）的效果**：

- Z 軸在 XZ 平面內轉動：從朝下 `[0,0,-1]` 可以轉到朝前 `[1,0,0]`（+X）或朝後 `[-1,0,0]`（-X）
- X 軸方向保持 `[0,1,0]` 不變
- **無法讓 Z 軸朝 ±Y 方向**（因為繞 Y 軸旋轉不改變 Y 分量）

**Rodrigues 公式繞任意軸旋轉**：

```matlab
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
```

## 自定義 `.m` 腳本模板

所有投遞到 `incoming/` 的 `.m` 腳本必須以固定模式開頭和結尾：

```matlab
fig = findobj('Type','figure','Name','RobotAgent - 7R Arm');
if isempty(fig)||~isvalid(fig), error('Figure not found.'); end
ud = fig.UserData; arm = ud.arm; current_q = ud.current_q;

% ===== 用戶代碼區域 =====
% ... 計算 q_target ...

q_traj = quinticTrajectory(current_q, q_target, duration, 30);
for i = 1:size(q_traj,1)
    updateRobotFigure(fig, q_traj(i,:));
    drawnow limitrate;
    pause(1/30);
end
fig.UserData.current_q = q_target;
fprintf('[Done] ...\n');
```

### 相對移動（保持姿態）

```matlab
T_cur = arm.forwardKinematics(current_q);
T_target = T_cur;
T_target(axis_idx, 4) = T_target(axis_idx, 4) + distance;
[q_target, err] = arm.inverseKinematics(T_target);
```

### 姿態調整（繞固定軸）

```matlab
% 繞 n 軸旋轉 theta（Rodrigues）
n = T_cur(1:3, axis);  % 如 X 軸、Y 軸、Z 軸
theta = ...;           % 計算旋轉角
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
T_target = T_cur;
T_target(1:3, 1:3) = R * T_cur(1:3, 1:3);
```

## 常見問題排查

| 現象 | 原因 | 解決 |
|------|------|------|
| `[ERR] Axes not found in figure` | `robotagent.m` 覆蓋了 `fig.UserData` | 關閉 Figure，重新運行 `robotagent.m` |
| `[ERR] 字符向量未正常終止` | `.m` 文件中的 `\n` 被錯誤解析為換行 | 確保 `fprintf` 字符串中的 `\n` 是字面量（無 BOM 編碼） |
| `[ERR] Target unreachable` | 目標位姿在機械臂工作空間外 | 先移動到更容易達到的位置，再放寬姿態約束 |
| 投遞後完全沒反應 | `is_busy` 卡住或 timer 停止 | 關閉 Figure，重新運行 `robotagent.m` |
| 動畫卡頓 | `drawnow limitrate` 丟幀 | 正常現象，降低 fps 或縮短軌跡點數 |

## 最佳實踐

1. **先移動再轉姿態**：固定位置調姿態容易不可達。先改變位置（如下移 100-300mm），再調姿態成功率更高。

2. **姿態調整優先順序**：
   - 繞當前 X/Y/Z 軸旋轉（Rodrigues）可嚴格保持該軸不變
   - `planTrajectoryCartesian` 的 SLERP 插值**無法保持特定軸方向**（會走姿態空間最短路徑，中間點的 Y 軸會偏離）
   - **home 位置繞末端 X 軸 = 繞世界 Y 軸**：可讓 Z 軸從朝下轉到朝 ±X，但無法朝 ±Y

3. **文件生成編碼**：
   - ❌ `Out-File -Encoding utf8`（產生 BOM）
   - ✅ `System.IO.File.WriteAllText(path, content, [System.Text.UTF8Encoding]::new($false))`

4. **組合指令**：複雜動作拆成多個 `.m` 文件，按文件名排序（`cmd_01_xxx.m`、`cmd_02_xxx.m`）

5. **IK 限制**：7R 機械臂固定 `θ1=0`，姿態自由度受限。若姿態不可達，可嘗試：
   - 改變位置後重試
   - 放寬姿態約束（允許其他軸微小變化）
   - 改用關節空間指令（`joint_move`）

## 關鍵數學

### 五次多項式軌跡
```matlab
q(t) = q0 + a3·t³ + a4·t⁴ + a5·t⁵
a3 = 10·(q1-q0)/T³,  a4 = -15·(q1-q0)/T⁴,  a5 = 6·(q1-q0)/T⁵
```

### 旋轉軸反轉（180°）
```matlab
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + 2*(K*K);  % 繞 n 軸旋轉 180°
```

---

*最後更新：2026年5月30日*
