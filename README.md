# Git Demo V02
# DOF_2_SH_ADD_PID

A MATLAB/Simulink simulation project for evaluating PID-based semi-active suspension control strategies on a 2-DOF vehicle model.

一个基于 MATLAB/Simulink 的车辆悬架仿真项目，用于评估和对比不同 PID 控制策略在二自由度模型下的性能表现。

---

## 📁 Project Structure | 项目结构

DOF_2_SH_ADD_PID/
├── data/ # Input data (road excitation, simulation time)
├── models/ # Simulink models (DOF2_P_SH_ADD_PID_*.slx)
├── scripts/ # MATLAB scripts (e.g., control logic, evaluation)
├── results/ # Output results (e.g., plots, processed data)
└── README.md # Project introduction

---

## 📌 Features | 项目特点

- ✅ 采用 2-DOF 悬架模型，涵盖簧上与簧下系统
- ✅ 实现 Skyhook、加速度反馈、PID 控制等策略
- ✅ 支持多种输入激励（如等级路面）
- ✅ 输出关键性能指标（SVA、SWS、DTD 等）及控制力分析
- ✅ 支持频域与时域的综合评估

---

## 🚀 How to Use | 使用说明

1. 打开 MATLAB 并设置项目路径为本仓库根目录
2. 加载模型文件（如 `DOF2_P_SH_ADD_PID_4_0620.slx`）
3. 运行脚本或启动 Simulink 仿真
4. 查看 `results/` 文件夹中的输出图像与数据

---

## 📊 Performance Indicators | 性能指标

- **SMA**：Sprung mass acceleration（簧上加速度）→ 乘坐舒适性
- **SWS**：Suspension working stroke（悬架动行程）→ 结构安全性
- **DTD**：Dynamic tire displacement（轮胎动位移）→ 操稳性
- **Control Force**：控制力峰值与功率消耗评估

---

## 📎 Related Files | 相关文件说明

| 文件名                      | 说明                              |
|---------------------------|-----------------------------------|
| `xr.mat`                  | 路面激励输入                      |
| `time.mat`                | 仿真时间轴                        |
| `fuzzy_pid_lookup.mat`    | 模糊PID查找表（如有）             |
| `DOF2_P_SH_ADD_PID_*.slx` | Simulink 模型                     |
| `time_domain_data.mat`    | 时域输出结果                      |
| `theta.mat`               | 控制器参数                        |

---

## 🧠 Author & License

Author: AprilStriver  
License: MIT (or specify your own)

---

