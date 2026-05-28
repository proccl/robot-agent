# 7自由度機械臂運動學庫 (Arm7R)

## 概述

這是一個MATLAB類庫，用於7自由度機械臂的運動學計算，包括正向運動學、逆向運動學、軌跡規劃等功能。

## 文件說明

- `Arm7R.m` - 主類文件，包含所有運動學功能
- `example_Arm7R.m` - 使用示例腳本
- `README_Arm7R.md` - 本說明文件

## DH參數

默認DH參數 (單位: mm):

```matlab
P_DH = [149.438, 147.9, 0, 458.09, 93.5, 360.71, 118.27, 272.42];
%        e,       k,     i, l,      m,    n,       j,       b
```

| 關節 | theta   | d       | a       | alpha   |
|------|---------|---------|---------|---------|
| 1    | π/2     | 149.44  | 0       | π/2     |
| 2    | π/2     | 147.90  | 0       | π/2     |
| 3    | 0       | 0       | 458.09  | 0       |
| 4    | 0       | -93.50  | 360.71  | 0       |
| 5    | 0       | 0       | 0       | -π/2    |
| 6    | -π/2    | 118.27  | 0       | -π/2    |
| 7    | 0       | -272.42 | 0       | π       |

## 快速開始

### 1. 創建機械臂對象

```matlab
% 使用默認DH參數
arm = Arm7R();

% 或使用自定義DH參數
P_DH = [149.438, 147.9, 0, 458.09, 93.5, 360.71, 118.27, 272.42];
arm = Arm7R(P_DH);
```

### 2. 正向運動學

```matlab
% 定義關節角 (rad)
q = [0, 0, 0, 0, 0, 0, 0];

% 計算末端位姿
T = arm.forwardKinematics(q);
% T為4x4齊次變換矩陣
```

### 3. 逆向運動學

```matlab
% 定義目標位姿
T_target = [0,  1,  0,  266.17;
            1,  0,  0, -93.5;
            0,  0, -1,  400.818;
            0,  0,  0,  1];

% 求解逆運動學
[q_sol, err] = arm.inverseKinematics(T_target);
% err = 0: 成功
% err = 1: 無解（目標不可達）
```

### 4. 軌跡規劃

#### 笛卡爾空間軌跡規劃

```matlab
% 定義起始和終止位姿
T_start = [...];  % 4x4矩陣
T_end = [...];    % 4x4矩陣

% 規劃軌跡 (100個插值點, 0到5秒)
[T_traj, q_traj, simin] = arm.planTrajectoryCartesian(T_start, T_end, 100, 0, 5);

% 輸出:
%   T_traj - 4x4x100 位姿軌跡
%   q_traj - 100x7 關節角軌跡
%   simin  - 100x8 矩陣 [time, q1, q2, q3, q4, q5, q6, q7]，Simulink可用
```

#### 關節空間軌跡規劃

```matlab
% 定義起始和終止關節角
q_start = [0, 0, 0, 0, 0, 0, 0];
q_end = [pi/4, pi/4, 0, 0, 0, -pi/4, 0];

% 規劃軌跡 (0到20秒)
simin = arm.planTrajectoryJoint(q_start, q_end, 0, 20);
% simin為Nx8矩陣: [time, q1, q2, q3, q4, q5, q6, q7]
```

### 5. 雅可比矩陣

```matlab
% 計算雅可比矩陣
J = arm.jacobian(q);

% 計算條件數 (評估奇異點接近程度)
kappa = arm.conditionNumber(q);
```

### 6. 可視化

```matlab
% 簡單繪圖 (無需額外工具箱)
arm.simplePlot(q);

% 完整可視化 (需要Robotics Toolbox)
arm.plot(q);
arm.teach();  % 交互式示教界面
```

### 7. 獲取關節位置

```matlab
% 計算各關節在世界坐標系中的位置
% 返回 9x3 矩陣: [基座; 關節1; 關節2; ...; 關節7/末端]
points = arm.getJointPositions(q);

% 使用示例: 繪製自定義軌跡
plot3(points(:,1), points(:,2), points(:,3), 'b-o');
```

## 方法原理解釋

### 1. 正向運動學 `forwardKinematics`

#### 原理
正向運動學根據關節角計算末端位姿，使用 **DH參數法** 和 **齊次變換矩陣**。

#### 數學基礎
對於每個關節，使用標準DH變換矩陣：

```
T_i = [cos(θ_i)   -sin(θ_i)cos(α_i)   sin(θ_i)sin(α_i)   a_i*cos(θ_i)
       sin(θ_i)    cos(θ_i)cos(α_i)  -cos(θ_i)sin(α_i)   a_i*sin(θ_i)
       0           sin(α_i)            cos(α_i)           d_i
       0           0                   0                  1]
```

#### 計算流程
1. 根據DH參數和關節角 θ，計算每個關節的齊次變換矩陣 T01, T12, ..., T67
2. 末端位姿通過連乘得到：`T07 = T01 × T12 × T23 × T34 × T45 × T56 × T67`
3. 最終 T07 包含：
   - 左上角 3×3：旋轉矩陣 R
   - 右上角 3×1：位置向量 P
   - 第4行：[0, 0, 0, 1]

#### 座標系說明
- Base (基座) → Joint 1 → Joint 2 → ... → Joint 7 → End-Effector (末端)
- 每個變換矩陣表示從前一個座標系到當前座標系的轉換

---

### 2. 逆向運動學 `inverseKinematics`

#### 原理
逆向運動學根據末端位姿求解關節角，使用 **解析法 (閉式解)**。

#### 求解策略
7自由度機械臂是 **冗餘機械臂** (自由度 > 任務空間維度6)，本實現採用以下策略：
- **固定 θ1 = 0**：消除冗餘，獲得唯一解
- **分步求解**：
  1. 從末端位置反推 θ2, θ6, θ7
  2. 求解 θ3, θ4, θ5 滿足姿態約束

#### 數學推導
設末端位姿 T = [R | P; 0 | 1]，其中 R 為 3×3 旋轉矩陣，P = [Px, Py, Pz] 為位置。

**步驟1：求解 θ2**
```
φ1c = -R33*b + Pz - e
φ1s = -Px*sin(θ1) + Py*cos(θ1) - b*(-sin(θ1)*R13 + cos(θ1)*R23)
ρ1 = √(φ1c² + φ1s²)
θ2 = atan2((-m+i)/ρ1, √(1-((-m+i)/ρ1)²)) - atan2(φ1s, φ1c)
```

**步驟2：求解 θ6, θ7**
```
θ6 = atan2(-sin(θ1)cos(θ2)R13 + cos(θ1)cos(θ2)R23 + sin(θ2)R33, ...)
θ7 = atan2(-sin(θ1)cos(θ2)R12 + cos(θ1)cos(θ2)R22 + sin(θ2)R32, ...)
```

**步驟3：求解 θ3, θ4, θ5**
```
θ345 = atan2(-(cos(θ1)R13 + sin(θ1)R23), -(sin(θ1)sin(θ2)R13 - cos(θ1)sin(θ2)R23 + cos(θ2)R33))
A1 = (Pz-e)cos(θ2) + sin(θ2)(sin(θ1)Px - cos(θ1)Py) + b*cos(θ6)cos(θ345) + j*sin(θ345)
B1 = sin(θ1)Py + cos(θ1)Px - k + b*cos(θ6)sin(θ345) - j*cos(θ345)
θ34 = -atan2(B1, A1) + atan2(E/ρ2, -√(1-(E/ρ2)²))
θ5 = θ345 - θ34
θ3 = atan2(cos(θ1)Px + sin(θ1)Py - k + b*cos(θ6)sin(θ345) - j*cos(θ345) - n*sin(θ34), ...)
θ4 = θ34 - θ3
```

#### 可達性判斷
在求解過程中檢查：
- `1 - ((-m+i)/ρ1)² < 0` → 不可達 (err = 1)
- `1 - (E/ρ2)² < 0` → 不可達 (err = 1)

---

### 3. 笛卡爾空間軌跡規劃 `planTrajectoryCartesian`

#### 原理
在笛卡爾空間（末端位姿空間）進行軌跡規劃，然後通過逆運動學轉換到關節空間。

#### 輸入輸出格式
```matlab
[T_traj, q_traj, simin] = arm.planTrajectoryCartesian(T_start, T_end, steps, t_start, t_end)
```

| 參數 | 尺寸 | 說明 |
|------|------|------|
| T_start | 4×4 | 起始位姿 (齊次變換矩陣) |
| T_end | 4×4 | 終止位姿 (齊次變換矩陣) |
| steps | 1×1 | 插值點數 (默認100) |
| t_start | 1×1 | 起始時間/s (默認0) |
| t_end | 1×1 | 終止時間/s (默認1) |
| **T_traj** | 4×4×steps | 位姿軌跡 |
| **q_traj** | steps×7 | 關節角軌跡 (rad) |
| **simin** | steps×8 | Simulink格式 [time, q1~q7] |

#### simin 格式詳細說明
```matlab
simin = [t₁, θ₁₁, θ₁₂, θ₁₃, θ₁₄, θ₁₅, θ₁₆, θ₁₇
         t₂, θ₂₁, θ₂₂, θ₂₃, θ₂₄, θ₂₅, θ₂₆, θ₂₇
         ...
         tₙ, θₙ₁, θₙ₂, θₙ₃, θₙ₄, θₙ₅, θₙ₆, θₙ₇]
```
- 第1列：時間 (s)
- 第2~8列：關節角 θ₁~θ₇ (rad)
- 行數 = steps (插值點數)

#### 位置規劃 - 線性插值 (Lerp)
對於位置 P = [x, y, z]，在起點 P_start 和終點 P_end 之間線性插值：
```
P(t) = P_start + s × (P_end - P_start),  s ∈ [0, 1]
```
其中 `s = (i-1)/(steps-1)` 為歸一化參數。

#### 姿態規劃 - 球面線性插值 (SLERP)
姿態用旋轉矩陣 R 表示，為避免萬向節鎖死和保證最短路徑，使用 **四元數球面插值**：

**方法一（有Robotics System Toolbox）：**
```matlab
quat_start = rotm2quat(R_start)  % 旋轉矩陣轉四元數
quat_end = rotm2quat(R_end)
quat(t) = quatinterp(quat_start, quat_end, s, 'slerp')  % 球面插值
R(t) = quat2rotm(quat(t))  % 四元數轉旋轉矩陣
```

**方法二（無工具箱）：軸角線性插值**
```
[axis_start, angle_start] = rotm2axang(R_start)  % 轉軸角
[axis_end, angle_end] = rotm2axang(R_end)
angle(t) = angle_start + s × (angle_end - angle_start)
axis(t) = axis_start + s × (axis_end - axis_start)  % 線性插值後歸一化
R(t) = Rodrigues(axis(t), angle(t))  % 羅德里格斯公式
```

#### Rodrigues旋轉公式
給定單位旋轉軸 k 和旋轉角 θ：
```
R = I + sin(θ)K + (1-cos(θ))K²
```
其中 K 為 k 的斜對稱矩陣：
```
K = [0    -k_z   k_y
     k_z   0    -k_x
    -k_y   k_x   0  ]
```

#### 求解流程
1. 生成時間向量 `time_vec = linspace(t_start, t_end, steps)`
2. 位置線性插值生成 P_traj
3. 姿態SLERP生成 R_traj
4. 組合位姿矩陣 T_traj = [R_traj | P_traj; 0 | 1]
5. 對每個 T_traj(:,:,i) 調用逆運動學求解 q_traj(i,:)
6. 組合 simin = [time_vec, q_traj]

#### 特點
- **末端走直線**：位置是線性變化
- **姿態平滑**：SLERP保證角速度恆定
- **Simulink可用**：simin格式與關節空間規劃一致
- **可能遇到奇異點**：逆解失敗時返回 NaN

---

### 4. 關節空間軌跡規劃 `planTrajectoryJoint`

#### 原理
直接在關節空間進行規劃，每個關節獨立插值。

#### 線性插值
對於每個關節 θ_j，在起點 θ_start(j) 和終點 θ_end(j) 之間線性插值：
```
θ_j(t) = θ_start(j) + (t-t_start)/(t_end-t_start) × (θ_end(j) - θ_start(j))
```

#### 輸出格式
生成 Simulink 可用的軌跡數據矩陣：
```
simin = [time, θ1, θ2, θ3, θ4, θ5, θ6, θ7]
```
- 時間列：從 t_start 到 t_end，1ms 一個點 (1000Hz)
- 關節角列：對應時刻的關節角度 (rad)

#### 特點
- **計算簡單**：無需逆運動學
- **末端路徑不確定**：可能不是直線
- **無奇異點問題**：直接在關節空間規劃
- **適合Simulink**：輸出格式可直接用於仿真

---

### 5. 雅可比矩陣 `jacobian`

#### 原理
雅可比矩陣 J 描述了關節速度與末端速度之間的映射關係：
```
[V; ω] = J × q_dot
```
其中 V 為線速度 (3×1)，ω 為角速度 (3×1)，q_dot 為關節角速度 (7×1)。

#### 計算方法
**方法一（有Robotics Toolbox）：**
使用 `robot.jacob0(q)` 直接計算。

**方法二（數值微分）：**
```
J(:,i) = (f(q + δe_i) - f(q)) / δ
```
其中：
- f(q) 為正向運動學函數
- e_i 為第 i 個單位向量
- δ = 1e-6 為微分步長

對於線速度部分：
```
J(1:3,i) = (P(q+δe_i) - P(q)) / δ
```

對於角速度部分（從旋轉矩陣導數提取）：
```
dR = (R(q+δe_i) - R(q)) / δ
ω = [dR(3,2)-dR(2,3); dR(1,3)-dR(3,1); dR(2,1)-dR(1,2)] / 2
```

#### 條件數 `conditionNumber`
雅可比條件數 κ(J) = ||J|| × ||J⁺|| 用於評估機械構型：
- κ ≈ 1：構型良好，各向同性
- κ → ∞：接近奇異點，某些方向不可達或速度放大

---

### 6. 可視化方法

#### `simplePlot` (無需工具箱)
繪製機械臂骨架圖：
1. 計算各關節在世界座標系中的位置
2. 用 `plot3` 連接各點形成連桿
3. 添加關節標註和坐標系標示
4. 設置合適的視角和坐標範圍

**圖形特性：**
- 白色背景，無標題
- 初始視角：view(45, 25)
- 透視投影：camproj('perspective')
- 啟用滑鼠左鍵旋轉：rotate3d on
- 標籤字體：9 號
- 末端坐標軸：長度 80mm，線寬 2
- Base 坐標軸：長度 50mm，線寬 1.5

#### `plot` / `teach` (需Robotics Toolbox)
使用 Robotics Toolbox 的專業繪圖功能，包含：
- 3D實體模型渲染
- 交互式示教界面
- 動畫播放

---

## 兩種軌跡規劃方法對比

| 特性 | 笛卡爾空間規劃 | 關節空間規劃 |
|------|---------------|-------------|
| **插值對象** | 末端位姿 (位置+姿態) | 關節角度 |
| **位置方法** | 線性插值 (Lerp) | 線性插值 (Lerp) |
| **姿態方法** | SLERP 或軸角插值 | 無（姿態由關節角決定）|
| **末端路徑** | 直線 | 曲線（不確定）|
| **計算複雜度** | O(steps × invK) | O(steps) |
| **奇異點問題** | 可能遇到 | 不會遇到 |
| **Simulink輸出** | simin (steps×8) | simin (N×8) |
| **使用場景** | 需要精確控制末端路徑 | 快速規劃、Simulink仿真 |

## 依賴項

### 核心功能 (無需額外工具箱)
- 正向運動學
- 逆向運動學
- 關節空間軌跡規劃
- 簡單繪圖 (`simplePlot`)

### 可選工具箱

#### Peter Corke的Robotics Toolbox
- 用於 `plot()` 和 `teach()` 方法
- 用於 `jacobian()` 方法（否則使用數值微分）
- 下載: https://petercorke.com/toolboxes/robotics-toolbox/

#### MATLAB Robotics System Toolbox
- 用於球面線性插值 (SLERP) 在 `planTrajectoryCartesian` 中
- 如果未安裝，自動使用軸角線性插值代替

## 完整示例

運行 `example_Arm7R.m` 查看完整使用示例：

```matlab
run('example_Arm7R.m');
```

或運行測試文件：

```matlab
test_Arm7R;
```

## 注意事項

1. **單位**: 所有長度單位為毫米(mm)，角度單位為弧度(rad)
2. **坐標系**: 使用標準DH參數約定
3. **初始姿態**: 默認初始顯示姿態為 `q0 = [π/2, π/2, 0, 0, 0, -π/2, 0]`
4. **關節限制**: 默認關節角範圍為 [-π, π]
5. **冗餘性**: 7R機械臂有1個冗餘自由度，逆解中固定θ1=0

## 參考

原始代碼來自: `D:\Document\code\Matlab\7Rmatlab`
- `MAIN.m` - 主程序
- `forK.m` - 正向運動學
- `invK.m` - 逆向運動學
- `T_DH.m` - DH變換矩陣
