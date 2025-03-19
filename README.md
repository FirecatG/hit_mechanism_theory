# HIT机械原理大作业

**功能**：计算四杆机构上某点的位置速度，加速度分量，并完成相关仿真
**作者**：FirecatG
---
## 一、完成情况&使用方法

#### 1.计算
- **方法（1）：（要求ROS环境）通过ROS执行launch文件来计算各点运动状态**
```
	mkdir -p four_links_ws/src
	cd four_links_ws/src
	catkin_init_workspace
	git clone git@github.com:FirecatG/HIT_machanism_theory.git
  rosdep install --from-paths . --ignore-src -y
	cd ..
	catkin build
	source devel/setup.bash
```
  **启动转动仿真（每一度计算一次）**
	`roslaunch hit_machanism_theory main`
  **启动计算**
	`roslaunch hit_machanism_theory calculate_only`
  **启动每30度一次计算**
	`roslaunch hit_machanism_theory publish_per_30_degree`


- **方法（2）：（无需ROS环境）使用python制图**
	`python3 scripts/chart.py`

#### 2.仿真
- SolidWorks Motion运动仿真与制图
- ROS环境下Rviz与Gazebo实现运动与控制仿真

---

## 二、依赖环境
- **Ubuntu 版本**：20.04/18.04
- **ROS 版本**：Noetic/Melodic
- **Python 版本**：3.8/3.6
