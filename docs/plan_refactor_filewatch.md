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
