# Continuous Safety-Critical Control of Mobile Robots

🌐 [中文说明](./README_zh.md)

This repository provides the simulation code for the following paper:  

**"Continuous Safety-Critical Control of Mobile Robots With Set-Valued Feedback in Body-Fixed Coordinate Frame"**  
*IEEE Transactions on Automatic Control, 2025.*  
[DOI: 10.1109/TAC.2025.3614544](https://doi.org/10.1109/TAC.2025.3614544)  

© IEEE 2025. Personal use of this material is permitted.  
The official version is available on [IEEE Xplore](https://ieeexplore.ieee.org/document/11180029).  

**Keywords:** Cluttered Environment, Lipschitz Continuity, Safety-Critical Control.

---

## Abstract

This paper investigates a safety-critical control problem for a mobile robot navigating cluttered environments.  
Obstacles are represented as a closed set, and a direction-distance function is used to model the obstacle measurement process.  

Based on real-time obstacle measurements, a safety-critical controller is expected to:

- maintain a safe distance between the robot and obstacles, and  
- ensure that the robot’s velocity, considered as the control input, follows a given velocity command signal as closely as possible.  

This paper contributes:

- a regularization method for the obstacle-measurement model in the body-fixed frame, and  
- a systematic design of a Lipschitz continuous safety-critical controller based on the regularized obstacle-measurement model and quadratic programming (QP).  

Specifically:

- a novel Lipschitz-truncated regularization term is used to replace the conventional quadratic regularization in Moreau-Yosida regularization;  
- it is guaranteed that, for each local minimum distance given by the original direction-distance function and its corresponding direction, there exists a neighborhood around that direction where the value of the regularized direction-distance function does not exceed the local minimum distance.  

This property is crucial for constructing a feasible set of control input that ensures safety in all directions using only a finite number of constraints. 
The QP framework is then employed to integrate these safety constraints with nominal velocity commands.  
A specific positive basis is employed to meet the linear independence constraint qualification, ensuring the Lipschitz continuity of the QP-based safety-critical controller.  

The effectiveness of the proposed design is demonstrated through numerical simulations and physical experiments.

---

## Repository Structure

This repository contains simulation code for reproducing the results in the paper. All code was developed and tested on **Windows 11 with MATLAB 2023b**.

### Simulations

1. **Nominal Scenario** (`main_MY.m`)  
   - Corresponds to Section IV of the paper.  
   - One mobile robot navigating among three triangular obstacles.  
   - Controllers compared:
     - **Proposed Safety-Critical Controller (PC)**
     - **Usual Safety-Critical Controller (UC)**
     - **Moreau-Yosida Regularized Controller (MYC)**  

   **Simulation Results:**

   ![MY_SIM](https://github.com/user-attachments/assets/0f8f7741-1a4d-4d69-8a15-d7609ea6d71a)

2. **Robustness Scenario** (`main_uncertainty_MY.m`)  
   - Evaluates robustness under **actuation dynamics**.  
   - The mobile robot is modeled as a **cascade of nominal system and actuation system**.  

   **Simulation Results:**

   ![MY_UNCERTAINTY_SIM](https://github.com/user-attachments/assets/2f7910d2-93f3-4a00-bc2a-616af99abe12)


### Key Files

| File | Description |
|------|-------------|
| `sim_map.bmp` | Monochrome map used for simulations. Can be modified to change the scenario. |
| `regularization.m` | Proposed Lipschitz-truncated regularization method. |
| `regularization_MY.m` | Moreau-Yosida regularization method. |
| `measurements_2D.m` | Simulates set-valued feedback using the map and robot position. |
| `half_plane.m` | Function for plotting half-spaces. |
| `graphic_MY.m` | Visualization of simulation results. |

---

## Citation

This repository accompanies the following paper. We would appreciate it if you could cite this work when using the code or related results:

```bibtex
@article{Wu-TAC-2025,
title={Continuous Safety-Critical Control of Mobile Robots With Set-Valued Feedback in Body-Fixed Coordinate Frame},
author={Wu, Si and Liu, Tengfei and Zhang, Weidong and Ding, Jinliang and Jiang, Zhong-Ping and Chai, Tianyou},
journal={IEEE Transactions on Automatic Control},
year={2025},
doi={10.1109/TAC.2025.3614544},
publisher={IEEE}
}
