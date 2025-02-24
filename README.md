# Tracked-Vehicle-Obstacle-Avoidance-RL  
**🚀 基于PPO算法的履带式车辆2D避障与越障强化学习系统**  


---

### 📌 项目简介  
本项目基于**近端策略优化（PPO）算法**，开发了一个履带式车辆的2D自主避障与越障系统。核心功能包括：  
- 在包含随机障碍物的仿真环境中训练履带小车  
- 实现车辆运动学建模与实时避障决策  
- 支持训练过程中断恢复与策略可视化测试  
- 提供纯脚本控制基准（Normal目录）用于对比验证  

---

### 📁 文件结构  
```  
├── Agent85.mat                # 预训练完成的PPO智能体模型  
├── matlab_agent.mat           # A2C网络架构定义（需配合加载使用）  
├── MyEnvironment.m            # 自定义RL环境类（观测/奖励函数定义）  
├── newCarSimulator.m          # 履带车辆运动学仿真核心模块  
├── Start.m                    # 预训练模型测试入口  
├── contiueTrain.m             # 从检查点恢复训练的脚本  
├── testModel.m                # 策略可视化测试（生成轨迹/障碍图）  
├── Obstacle.m                 # 障碍物生成与碰撞检测工具  
├── testenv.m                  # 主训练脚本（PPO算法入口）  
└── Normal/                    # 无RL控制的基准脚本实现  
```  

---

### ⚙️ 安装与依赖  
**必需环境**：  
- MATLAB 2024a  
- Reinforcement Learning Toolbox  

**配置说明**：  
```bash  
# 克隆仓库后直接在MATLAB中打开项目  
git clone https://github.com/yourusername/Tracked-Vehicle-Obstacle-Avoidance-RL.git  
```  

---

### 🚦 快速开始  
**测试预训练模型**：  
运行Start.m即可，**注意运行前需要加载matlab_agent.mat**，剩下的不用管，Start.m脚本会帮你加载

---

### 🧠 训练与测试  
#### 新训练流程  
```matlab  
>> run('testenv.m')  % 启动PPO训练  
```  

#### 继续训练  
运行contiueTrain.m即可，**注意运行前需要加载matlab_agent.mat**
#### 评估指标  
![untitled6](https://github.com/user-attachments/assets/abb26fb0-a8a1-4519-aaf5-dd262814ec77)

![308079d7763ece9933f21ddf53465ea](https://github.com/user-attachments/assets/6d69f3c6-808a-46cb-a38f-bba9ac214c07)

---

### 📊 结果与演示  
#### 训练曲线示例  
![untitled2](https://github.com/user-attachments/assets/4cc7c427-4a68-42cb-89c6-5b4a18ed8e3d)

![c](https://github.com/user-attachments/assets/6a2cc585-b2b9-4070-9192-200d4f806421)


#### 仿真演示  

---
