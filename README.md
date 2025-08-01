# gazebo小车直线赛道项目说明

## 项目总体结构

项目位于 `src` 目录，主要包含以下几个功能模块：

- **`fsac_autonomous/`**：核心功能模块，包括控制、路径规划、感知、建图等。
- **`fsac_track_worlds/`**：赛道仿真环境模块，包含 Gazebo 世界文件和相关配置。
- **`fsd_common_msgs/`**：自定义消息模块，定义了项目中使用的 ROS 消息类型。
- **`racecar_description/`**：车辆模型描述模块，包含 URDF 文件和车辆相关配置。

---

## 模块详细说明

### 1. **`fsac_autonomous/` 模块**

这是项目的核心模块，包含以下子模块：

#### 1.1 **`control/`**
- **功能**：实现车辆的控制逻辑。
- **主要内容**：
  - 使用 PID 控制算法实现车辆的速度和转向控制。
  - 包含调试功能，例如发布车辆的实际轨迹（红色线）和目标点（蓝色球）。
  - 通过 `publish_control_line` 和 `publish_debug_markers` 方法可视化控制效果。

#### 1.2 **`mapping/`**
- **功能**：动态建图模块，实时构建赛道地图。
- **主要内容**：
  - 接收来自感知模块的锥筒检测数据，生成赛道边界和中心线。
  - 发布建图结果，包括锥筒分类（红、蓝、黄）、赛道边界和完整地图。
  - 提供建图进度和质量的反馈，支持路径规划模块的运行。

#### 1.3 **`perception/`**
- **功能**：感知模块，处理激光雷达数据并检测赛道锥筒。
- **主要内容**：
  - `lidar_processor.py`：处理激光雷达点云数据，进行聚类分析并发布锥筒检测结果。
  - `cone_detector.py`：对锥筒进行分类（红、蓝、黄）并发布可视化结果。
  - 提供锥筒的位置信息和分类结果，供建图模块使用。

#### 1.4 **`planning/`**
- **功能**：路径规划模块，生成车辆的行驶路径。
- **主要内容**：
  - 根据建图模块提供的赛道信息，生成最优路径（包括纠偏曲线和直线路径）。
  - 支持路径的实时重规划，确保车辆能够适应动态环境。
  - 提供路径的可视化功能，显示规划的路径点和目标点。

#### 1.5 **`slam/`**
- **功能**：SLAM 模块，用于车辆的定位和地图构建。
- **主要内容**：
  - 提供车辆的位置信息（`map -> odom` 的 TF 变换）。
  - 支持动态建图模块的运行。

#### 1.6 **`utils/`**
- **功能**：工具模块，提供通用功能。
- **主要内容**：
  - 包含数学计算、坐标变换等辅助功能。
  - 为其他模块提供支持。

#### 1.7 **配置与启动文件**
- **`config/`**：
  - 包含 RViz 配置文件（`fsac_visualization.rviz`）和路径规划参数（`straight_line_params.yaml`）。
- **`launch/`**：
  - 启动文件（`test.launch`），用于启动整个系统，包括感知、建图、路径规划和控制模块。

---

### 2. **`fsac_track_worlds/` 模块**

- **功能**：提供赛道仿真环境。
- **主要内容**：
  - 包含 Gazebo 世界文件（`worlds/`），定义了赛道的布局和环境。
  - 启动文件（`fsac_track.launch`），用于加载 Gazebo 仿真环境。
  - 提供赛道的加速段、减速段和锥筒布局信息。

---

### 3. **`fsd_common_msgs/` 模块**

- **功能**：定义自定义的 ROS 消息类型。
- **主要内容**：
  - 包含锥筒检测消息（`ConeDetections`）、车辆状态消息（`CarState`）等。
  - 为感知、建图和控制模块提供统一的数据接口。

---

### 4. **`racecar_description/` 模块**

- **功能**：描述车辆的物理模型和传感器配置。
- **主要内容**：
  - **`urdf/`**：包含车辆的 URDF 文件，定义了车辆的几何结构、关节和传感器位置。
  - **`scripts/`**：包含与车辆模型在 Gazebo 中进行初始化的脚本。

---

## 运行步骤

1. **编译项目**：
   ```bash
   catkin_make
   ```

2. **设置环境变量**：
   ```bash
   source devel/setup.bash
   ```

3. **启动系统**：
   ```bash
   roslaunch fsac_autonomous test.launch
   ```

---

通过以上步骤，可以启动整个系统，包括感知、建图、路径规划和控制模块，可以通过rviz进行小车轨迹的可视化，并在 Gazebo 仿真环境中运行车辆。