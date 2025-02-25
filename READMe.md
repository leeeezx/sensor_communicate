# 三轴力传感器 ROS2 包

这是一个用于ROS2的力传感器包，可以读取和发布三轴力传感器的数据。该包支持同时读取三个不同串口的力传感器数据，并将其作为ROS2话题进行发布。

## 功能特点

- 支持三轴力传感器数据采集与发布
- 提供两种工作模式：
  - ~~三轴合并模式：将三个轴的数据合并为一个消息发布（失败）~~
  - 三轴分离模式：为每个轴单独创建发布者，独立发布数据（成功）
- 使用ASCII通信模式与传感器进行串口通信
- 可靠的数据采集机制和错误处理

## 硬件要求

- 三个单轴力传感器，通过串口（如USB转串口）连接到计算机
- 默认串口设备名: `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyUSB2`

## 依赖

- ROS2（测试于Foxy/Humble）
- Python 3
- pyserial (python3-serial)
- rclpy
- std_msgs

## 安装

1. 首先确保您已经安装并配置了ROS2环境。

2. 将此包克隆到您的ROS2工作空间的src目录下：
   ```bash
   cd ~/your_ros2_ws/src
   git clone https://your-repository-url/sensor.git
   ```

3. 安装依赖项：
   ```bash
   pip3 install pyserial
   ```

4. 构建工作空间：
   ```bash
   cd ~/your_ros2_ws
   colcon build --symlink-install
   ```

5. 加载环境：
   ```bash
   source ~/your_ros2_ws/install/setup.bash
   ```

## 使用方法

### 运行传感器节点

启动力传感器节点（默认使用分离模式）步骤如下：

新建终端：ctrl+alt+t

```bash
ros2 run sensor force_sensor
```

### plotjuggler查看/保存数据

建议使用**plotjuggler**进行数据可视化和保存

启动命令（如果已安装plotjuggler）

```bash
source install/setup.sh

ros2 run plotjuggler plotjuggler
```



### 查看发布的数据

在分离模式下，可以订阅以下话题查看各轴数据：

```bash
# 查看X轴数据
ros2 topic echo /force_sensor_x

# 查看Y轴数据
ros2 topic echo /force_sensor_y

# 查看Z轴数据
ros2 topic echo /force_sensor_z
```

~~在合并模式下，可以订阅以下话题查看三轴数据：~~

```bash
ros2 topic echo /force_sensor_data
```

## 自定义配置

如需修改串口配置或工作模式，请编辑 `sensor/force_sensor_node.py` 文件：

- ~~要使用合并模式，取消注释 `三轴一起` 部分的代码，并注释掉 `三轴分开` 部分的代码~~
- 要修改串口设备名称，修改相应的 `port_name` 参数

## 传感器通信参数

默认的传感器通信参数：
- 波特率：115200
- 数据位：8
- 校验位：无 (N)
- 停止位：1
- 超时：1秒

## 故障排除

如果遇到无法访问串口的问题，请确保:

1. 串口设备存在：
   ```bash
   ls -l /dev/ttyUSB*
   ```

2. 用户有权限访问串口设备：
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   添加权限后，需要注销并重新登录。

3. 检查串口连接和传感器电源是否正常。

## 许可

[添加您的许可证信息]

## 维护者

- 维护者：le
- 邮箱：2597769148@qq.com
