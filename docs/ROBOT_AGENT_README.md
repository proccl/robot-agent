# RobotAgent 使用说明

## 快速开始

### 启动服务器

```matlab
cd('D:\Document\code\Matlab\robot-agent');
run_robot_agent;  % 默认端口 12345
```

或指定端口：

```matlab
run_robot_agent(12346);
```

### 发送第一条指令

通过 PowerShell：

```powershell
.\scripts\send_robot_cmd.ps1 -Port 12345 -Message '{"cmd":"home","duration":2}'
```

通过 MATLAB tcpclient：

```matlab
c = tcpclient('127.0.0.1', 12345);
writeline(c, '{"cmd":"home","duration":2}');
resp = readline(c);
```

## 支持的指令

| 指令 | 示例 JSON | 说明 |
|------|----------|------|
| home | `{"cmd":"home","duration":2}` | 回到零位 |
| move_to | `{"cmd":"move_to","position":[300,0,700],"duration":2}` | 笛卡尔移动到目标位置 |
| joint_move | `{"cmd":"joint_move","joint":1,"angle":90,"angle_deg":true,"duration":1}` | 单关节转动 |
| trajectory | `{"cmd":"trajectory","type":"circle","radius":200,"duration":3}` | 执行轨迹 |
| set_speed | `{"cmd":"set_speed","factor":2.0}` | 设置动画倍速 |
| get_status | `{"cmd":"get_status"}` | 获取当前状态 |
| plot | `{"cmd":"plot"}` | 刷新图形 |

## 故障排查

| 问题 | 可能原因 | 解决 |
|------|---------|------|
| 端口被占用 | 上次运行未正确关闭 | 自动尝试 12346, 12347... |
| Figure 被关闭 | 用户误关窗口 | 自动重建 |
| IK 无解 | 目标位置不可达 | 调整目标位置 |
| 动画卡顿 | 计算阻塞渲染 | 已用 timer 异步渲染 |
