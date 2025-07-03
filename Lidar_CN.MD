# OrbbecSDK ROS1 雷达驱动

此 ROS1 驱动程序支持您使用Orbbec 单线/多线 LiDAR，本文档提供安装说明、使用指南以及其他重要信息，帮助您快速上手使用该驱动程序。

## 1. 安装

### 1.1 先决条件

在使用 OrbbecSDK ROS1 LiDAR驱动程序之前，请确保您的系统上安装了以下依赖项：

- **ROS1**: 有效安装 ROS1（Noetic、Melodic 或其他受支持的发行版）。
  - 如果您需要帮助，请参阅 [ROS1 安装指南](http://wiki.ros.org/ROS/Installation).

### 1.2 安装 deb 依赖项

```bash
# Assuming you have sourced the ROS environment, same below
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs \
libdw-dev
```

### 1.3 安装 udev 规则

```bash
cd ~/ros_ws
catkin_make
source ./devel/setup.bash
roscd orbbec_camera
sudo bash ./scripts/install_udev_rules.sh
```

### 2. 入门

启动雷达节点

* 第一个终端

```bash
cd ~/ros_ws
source ./devel/setup.bash
roslaunch orbbec_camera lidar.launch
```

* 第二个终端

```bash
cd ~/ros_ws
source ./devel/setup.bash
rviz
```

1.打开Rviz。

2.添加一个 `PointCloud2`或者 `LaserScan`显示。

3.`PointCloud2`选择 `/lidar/cloud/points`话题，`LaserScan`选择 `/lidar/scan/points`话题。

4.将 `Fixed Frame`设置为 `lidar_lidar_frame`，以正确对齐数据。

* `PointCloud2`示例可视化：

![module in rviz2](./docs/images/lidar1.png)

* `LaserScan`示例可视化：

![module in rviz2](./docs/images/lidar0.png)

## 2. 用法

### 2.1 运行驱动程序

要启动驱动程序，请启动提供的 ROS1 启动文件：

```bash
source install/setup.bash
# Launch the driver with point cloud data
roslaunch orbbec_camera lidar.launch lidar_format:=LIDAR_POINT
# Or Launch the driver with sphere point cloud data
roslaunch orbbec_camera lidar.launch lidar_format:=LIDAR_SPHERE_POINT
# Or Launch the driver with laser scan data
roslaunch orbbec_camera lidar.launch lidar_format:=LIDAR_SCAN
```

此命令将启动与 Orbbec LiDAR 设备接口的节点。运行此命令前，请确保 LiDAR 硬件已正确连接。

### 2.2 获取已连接雷达的设备信息

```bash
rosrun orbbec_camera list_devices_node
```

此命令将列出已连接的 LiDAR 设备，并显示它们各自的 IP 地址和端口。您可以使用这些信息来配置驱动程序以连接到特定设备。

### 2.3 检查雷达支持哪些配置

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

### 2.4 参数和配置

`lidar.launch`文件包含驱动程序的默认参数。您可以通过修改启动文件或创建自定义配置文件来自定义这些设置。关键参数包括：

- **device_type**：启动的设备类型。可选的值：`lidar`、`camera`。该参数设置为 `lidar`则启动雷达设备，设置为 `camera`则启动相机设备。
- **camera_name**：启动节点命名空间。
- **device_num**：设备数量。如果需要启动多个设备，则必须填写此项。
- **upgrade_firmware**：固件升级功能。输入参数是固件路径。
- **connection_delay**：重新打开设备的延迟时间（以毫秒为单位）。并且在热插拔时立即重新打开设备可能会导致固件崩溃。
- **publish_tf**：启用 TF 发布。
- **tf_publish_rate**：TF 发布频率。
- **lidar_format**：雷达的数据格式。可选值：`LIDAR_POINT`、`LIDAR_SPHERE_POINT` 、`LIDAR_SCAN`
- **lidar_rate**：雷达的扫描速率。
- **repetitive_scan_mode**：重复扫描模式参数。
- **filter_level**：添加过滤等级参数。
- **vertical_fov**：垂直角度参数。
- **min_angle**：雷达扫描范围的最小角度，以度为单位（例如 `-135.0`）。默认值：`-135.0`。
- **max_angle**：雷达扫描范围的最大角度，以度为单位（例如 `135.0`）。默认值：`135.0`。
- **min_range**：雷达可测量的最小距离，以米为单位。默认值：`0.05`。
- **max_range**：雷达可测量的最大距离，以米为单位。默认值：`30.0`。
- **echo_mode**：雷达的回波模式。可选值：`Last Echo`，`First Echo`
- **enumerate_net_device**：启用自动枚举网络设备。
- **net_device_ip**：网络设备的IP地址。
- **net_device_port**：网络这边的端口号。
- **log_level**：SDK日志级别，默认值为 `none`，可选值为 `debug`、`info`、`warn`、`error`、`fatal`
- **time_domain**：设备的时间戳类型。可选值为 `device`、`global`、`system`
- **enable_heartbeat**：启用心跳功能，默认为 `false`。如果设置为 `true`，摄像头节点将向固件发送心跳信号；如果需要硬件日志记录，也应设置为 `true`。

## pointcloud data 详细说明

Livox pointcloud2 （PointXYZRT） 点云格式，如下：

```
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # lidar reflectivity
uint8   tag             # lidar tag
```
