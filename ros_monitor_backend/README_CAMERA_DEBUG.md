# 相机系统调试指南

## 问题描述

前端页面无法显示相机数据，需要调试从ROS话题到前端显示的整个数据流。

## 问题分析

### 1. 话题配置问题
- **实际发布的话题**: `/left_camera/image/compressed` (CompressedImage格式)
- **后端配置的话题**: `/left_camera/image/compressed` ✅
- **订阅器类型**: 应该使用 `CompressedCameraSubscriber`

### 2. 数据流中断点
- ROS话题订阅 → 后端处理 → WebSocket广播 → 前端接收 → 图像显示

### 3. 可能的问题
- 订阅器创建失败
- 图像格式处理错误
- WebSocket消息类型不匹配
- 前端订阅逻辑错误

## 调试工具

### 1. 压缩图像相机调试测试器
```bash
./run_compressed_camera_debug.sh
```
**功能**: 测试ROS话题订阅和图像解码
**输出**: 保存调试图像到 `debug_images_*` 目录

### 2. 后端相机数据接收测试
```bash
./run_backend_test.sh
```
**功能**: 验证后端是否能接收ROS话题数据
**输出**: 显示话题订阅状态和帧数

### 3. 相机系统完整调试器
```bash
./run_system_debug.sh
```
**功能**: 测试整个系统从ROS到前端的完整数据流
**输出**: 综合状态报告和错误诊断

## 调试步骤

### 步骤1: 检查ROS话题
```bash
# 检查话题是否存在
rostopic list | grep camera

# 检查话题信息
rostopic info /left_camera/image/compressed

# 检查话题数据
rostopic echo /left_camera/image/compressed -n 1
```

### 步骤2: 测试话题订阅
```bash
cd /home/ycs/work/ikinghandbot/ros_monitor_backend
./run_compressed_camera_debug.sh
```
**预期结果**: 能看到图像保存到 `debug_images_*` 目录

### 步骤3: 测试后端接收
```bash
./run_backend_test.sh
```
**预期结果**: 显示话题订阅成功，帧数递增

### 步骤4: 检查后端状态
```bash
curl -s http://localhost:8000/api/v1/system/status | python3 -m json.tool
```
**关键信息**:
- `ros_connection.running`: true
- `subscriber_count`: > 0
- `data_topics`: 包含相机话题

### 步骤5: 测试完整系统
```bash
./run_system_debug.sh
```
**预期结果**: 显示完整的系统状态和错误诊断

## 常见问题解决

### 问题1: 订阅器创建失败
**症状**: 日志显示 "Failed to create compressed camera subscriber"
**解决**: 检查ROS环境和工作空间设置

### 问题2: 图像解码失败
**症状**: 日志显示 "压缩图像解码失败"
**解决**: 检查图像格式，确认是JPEG还是PNG

### 问题3: WebSocket连接失败
**症状**: 前端显示连接错误
**解决**: 检查后端服务是否运行，端口是否被占用

### 问题4: 前端无法接收数据
**症状**: 后端有数据但前端无显示
**解决**: 检查WebSocket消息类型和订阅逻辑

## 调试输出说明

### 调试图像目录
- `debug_images_left_camera/`: 左相机调试图像
- `debug_images_right_camera/`: 右相机调试图像
- 每10帧保存一次图像，用于验证数据接收

### 日志级别
- `INFO`: 正常操作信息
- `WARNING`: 警告信息，需要关注
- `ERROR`: 错误信息，需要立即处理
- `DEBUG`: 详细调试信息

### 状态检查
- **ROS连接**: 检查节点是否正常运行
- **话题订阅**: 检查订阅器是否创建成功
- **数据接收**: 检查帧数是否递增
- **后端状态**: 检查API是否响应正常

## 性能优化建议

### 1. 图像处理
- 调整 `max_width` 参数控制图像尺寸
- 调整 `jpeg_quality` 参数控制压缩质量
- 使用帧率限制避免过度处理

### 2. 网络传输
- 监控WebSocket连接状态
- 检查数据传输延迟
- 优化消息序列化

### 3. 内存管理
- 及时释放不需要的图像数据
- 监控内存使用情况
- 避免内存泄漏

## 联系支持

如果问题仍然存在，请提供以下信息：
1. 调试工具的输出日志
2. 系统状态API的响应
3. 前端控制台的错误信息
4. 后端服务的日志文件
