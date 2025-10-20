# 连续安全攸关控制的移动机器人仿真

🌐 [English Version](./README.md)

本仓库提供以下论文的仿真代码与实现细节：  

**《Continuous Safety-Critical Control of Mobile Robots With Set-Valued Feedback in Body-Fixed Coordinate Frame》**  
*IEEE Transactions on Automatic Control, 2025.*  
[DOI: 10.1109/TAC.2025.3614544](https://doi.org/10.1109/TAC.2025.3614544)  

论文正式版本可在 [IEEE Xplore](https://ieeexplore.ieee.org/document/11180029) 获取。  

**关键词：** 杂乱环境、Lipschitz 连续性、安全攸关控制。

---

## 摘要

本文研究了移动机器人在复杂障碍环境中的安全攸关控制问题。  
障碍物被表示为一个闭集，采用方向–距离函数来描述障碍物测量过程。  

基于实时障碍物测量结果，安全攸关控制器需要同时满足以下目标：

- 保持机器人与障碍物之间的安全距离；  
- 确保机器人的速度（作为控制输入）尽可能跟踪给定的期望速度信号。  

本文的主要贡献包括：

- 在机体坐标系下提出了一种障碍物测量模型的正则化方法；  
- 基于正则化模型与二次规划（QP）框架，系统设计了一种 Lipschitz 连续的安全攸关控制器。  

具体而言：

- 提出了一种新型的 Lipschitz 截断正则化项，以替代传统的 Moreau–Yosida 二次正则化；  
- 证明了：对于原始方向–距离函数的每个局部最小距离及其对应方向，存在一个方向邻域，使得正则化后的方向–距离函数值不超过该局部最小距离。  

这一性质对于利用有限数量约束构造在所有方向上均安全的控制输入可行集至关重要。  
在此基础上，通过 QP 框架将安全约束与名义速度指令融合。  
进一步地，引入特定的正基（positive basis）以满足线性独立约束资格，从而保证所设计控制器的 Lipschitz 连续性。  

所提出方法的有效性通过数值仿真与实际实验得到了验证。

---

## 仓库结构

本仓库包含用于复现论文结果的 MATLAB 仿真代码。  
代码在 **Windows 11 + MATLAB 2023b** 环境下开发和测试。

### 1. 标准场景仿真 (`main_MY.m`)

- 对应论文第 IV 节。  
- 场景：单个移动机器人在三角形障碍物间导航。  
- 比较的控制器：
  - **本文提出的安全攸关控制器（PC）**
  - **常规安全攸关控制器（UC）**
  - **基于Moreau–Yosida正则化的安全攸关控制器（MYC）**

**仿真结果：**

![MY_SIM](https://github.com/user-attachments/assets/0f8f7741-1a4d-4d69-8a15-d7609ea6d71a)

---

### 2. 鲁棒性仿真 (`main_uncertainty_MY.m`)

- 用于验证所提控制器在存在**执行器动态**时的鲁棒性。  
- 模型将移动机器人视为“名义系统 + 执行系统”的级联系统。

**仿真结果：**

![MY_UNCERTAINTY_SIM](https://github.com/user-attachments/assets/2f7910d2-93f3-4a00-bc2a-616af99abe12)

---

### 主要文件说明

| 文件名 | 功能说明 |
|--------|-----------|
| `sim_map.bmp` | 仿真所用的单色地图，可修改以改变仿真场景。 |
| `regularization.m` | 本文提出的 Lipschitz 截断正则化方法。 |
| `regularization_MY.m` | Moreau–Yosida 正则化方法实现。 |
| `measurements_2D.m` | 基于地图与机器人位置的集合值反馈模拟函数。 |
| `half_plane.m` | 绘制半空间约束的辅助函数。 |
| `graphic_MY.m` | 仿真结果可视化程序。 |

---

## 引用

如果您在研究中使用了本代码或相关结果，欢迎引用以下论文：

```bibtex
@article{Wu-TAC-2025,
title={Continuous Safety-Critical Control of Mobile Robots With Set-Valued Feedback in Body-Fixed Coordinate Frame},
author={Wu, Si and Liu, Tengfei and Zhang, Weidong and Ding, Jinliang and Jiang, Zhong-Ping and Chai, Tianyou},
journal={IEEE Transactions on Automatic Control},
year={2025},
doi={10.1109/TAC.2025.3614544},
publisher={IEEE}
}
