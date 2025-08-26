# 相机数据接收测试指南

## 问题描述

前端页面无法接收到相机数据，需要调试相机回调函数。相机驱动使用 `PixelFormat: 3`，对应 `BayerGB12Packed` 格式。

## 解决方案

### 1. 重新设计的相机回调函数

已优化 `src/ros_bridge/subscribers/camera_subscriber.py` 中的回调函数：

- **多种转换方法**: 针对 `BayerGB12Packed` 格式提供5种不同的转换方法
- **详细日志**: 添加详细的调试日志，便于问题定位
- **错误处理**: 改进错误处理机制，自动重试和恢复
- **测试图像**: 当转换失败时生成测试图像，确保前端能显示内容

### 2. 测试工具

#### 2.1 简单相机测试器 (`test_camera_simple.py`)

**功能**:
- 订阅相机话题并接收图像数据
- 尝试多种方法处理 `BayerGB12Packed` 格式
- 保存接收到的图像到文件
- 实时显示图像（如果可能）

**使用方法**:
```bash
cd /home/ycs/work/ikinghandbot/ros_monitor_backend
python3 test_camera_simple.py
```

#### 2.2 话题调试器 (`debug_camera_topics.py`)

**功能**:
- 检查相机话题状态
- 监控话题发布者数量
- 显示接收到的图像基本信息

**使用方法**:
```bash
cd /home/ycs/work/ikinghandbot/ros_monitor_backend
python3 debug_camera_topics.py
```

#### 2.3 启动脚本 (`run_camera_test.sh`)

**功能**:
- 自动检查ROS环境
- 检查工作空间状态
- 检查相机话题和驱动状态
- 启动相机测试器

**使用方法**:
```bash
cd /home/ycs/work/ikinghandbot/ros_monitor_backend
./run_camera_test.sh
```

## 测试步骤

### 步骤1: 检查环境

```bash
# 检查ROS环境
echo $ROS_DISTRO

# 检查工作空间
ls /home/ycs/work/ikinghandbot/devel/setup.bash

# 检查相机驱动
ps aux | grep mvs_ros_driver
```

### 步骤2: 检查话题状态

```bash
# 列出所有话题
rostopic list

# 检查相机话题
rostopic list | grep camera

# 检查话题信息
rostopic info /left_camera/image
```

### 步骤3: 运行测试

```bash
# 方法1: 使用启动脚本（推荐）
./run_camera_test.sh

# 方法2: 直接运行测试器
python3 test_camera_simple.py

# 方法3: 运行话题调试器
python3 debug_camera_topics.py
```

## 预期结果

### 成功情况

1. **话题状态**: 相机话题显示为活跃状态
2. **数据接收**: 能看到图像数据接收的日志
3. **图像处理**: 至少有一种转换方法成功
4. **文件保存**: 在当前目录生成测试图像文件
5. **前端显示**: 前端页面能显示相机图像

### 失败情况

1. **话题不存在**: 检查相机驱动是否启动
2. **订阅失败**: 检查ROS节点状态
3. **转换失败**: 查看具体错误信息，可能需要调整转换方法
4. **权限问题**: 检查文件写入权限

## 故障排除

### 常见问题

1. **相机驱动未启动**
   ```bash
   # 启动相机驱动
   roslaunch mvs_ros_driver left_camera.launch
   ```

2. **话题名称不匹配**
   ```bash
   # 查看实际的话题名称
   rostopic list
   # 修改测试脚本中的话题名称
   ```

3. **依赖库缺失**
   ```bash
   # 安装OpenCV
   pip install opencv-python
   # 安装numpy
   pip install numpy
   ```

4. **权限问题**
   ```bash
   # 给脚本添加执行权限
   chmod +x run_camera_test.sh
   ```

### 调试技巧

1. **查看详细日志**: 修改日志级别为DEBUG
2. **检查数据格式**: 使用 `rostopic echo` 查看原始数据
3. **验证图像质量**: 检查保存的图像文件
4. **监控系统资源**: 使用 `htop` 监控CPU和内存使用

## 后续优化

1. **图像压缩**: 实现自适应JPEG质量调整
2. **格式支持**: 添加更多图像格式的支持
3. **性能优化**: 实现多线程图像处理
4. **错误恢复**: 改进自动重连和错误恢复机制

## 联系支持

如果遇到问题，请提供以下信息：
- 错误日志
- 相机配置参数
- 系统环境信息
- 测试结果截图

