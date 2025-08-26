#!/usr/bin/env python3
"""
环境检查和修复脚本
检查ROS监控系统所需的所有依赖和环境配置
"""

import sys
import subprocess
import importlib
import os
import platform
from pathlib import Path

def run_command(cmd, check=True):
    """运行命令并返回结果"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if check and result.returncode != 0:
            print(f"❌ 命令失败: {cmd}")
            print(f"错误: {result.stderr}")
            return False, result.stderr
        return True, result.stdout
    except Exception as e:
        return False, str(e)

def check_python_version():
    """检查Python版本"""
    print("🐍 检查Python版本...")
    version = sys.version_info
    if version.major == 3 and version.minor >= 8:
        print(f"✅ Python版本: {version.major}.{version.minor}.{version.micro}")
        return True
    else:
        print(f"❌ Python版本过低: {version.major}.{version.minor}.{version.micro}")
        print("   需要Python 3.8+")
        return False

def check_ros_environment():
    """检查ROS环境"""
    print("\n🤖 检查ROS环境...")
    
    # 检查ROS_DISTRO环境变量
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"✅ ROS版本: {ros_distro}")
    else:
        print("❌ ROS_DISTRO环境变量未设置")
        return False
    
    # 检查ROS安装
    ros_setup = f"/opt/ros/{ros_distro}/setup.bash"
    if os.path.exists(ros_setup):
        print(f"✅ ROS安装路径: {ros_setup}")
    else:
        print(f"❌ ROS安装路径不存在: {ros_setup}")
        return False
    
    # 检查rospy模块
    try:
        import rospy
        print("✅ rospy模块可用")
    except ImportError:
        print("❌ rospy模块不可用")
        return False
    
    return True

def check_ros_topics():
    """检查ROS话题"""
    print("\n📡 检查ROS话题...")
    
    success, output = run_command("rostopic list", check=False)
    if success:
        topics = output.strip().split('\n')
        camera_topics = [t for t in topics if 'camera' in t or 'image' in t]
        
        if camera_topics:
            print("✅ 找到相机话题:")
            for topic in camera_topics:
                print(f"  - {topic}")
        else:
            print("⚠️  未找到相机话题")
        
        print(f"✅ 总话题数: {len(topics)}")
        return True
    else:
        print("❌ 无法获取ROS话题列表")
        print("   请确保ROS Master正在运行")
        return False

def check_python_packages():
    """检查Python包依赖"""
    print("\n📦 检查Python包依赖...")
    
    required_packages = [
        'fastapi',
        'uvicorn',
        'websockets',
        'pydantic',
        'opencv-python',
        'numpy',
        'python-multipart',
        'structlog'
    ]
    
    missing_packages = []
    
    for package in required_packages:
        try:
            if package == 'opencv-python':
                importlib.import_module('cv2')
                print(f"✅ {package} (cv2)")
            else:
                importlib.import_module(package.replace('-', '_'))
                print(f"✅ {package}")
        except ImportError:
            print(f"❌ {package}")
            missing_packages.append(package)
    
    if missing_packages:
        print(f"\n⚠️  缺少包: {', '.join(missing_packages)}")
        print("   运行以下命令安装:")
        print(f"   pip install {' '.join(missing_packages)}")
        return False
    
    return True

def check_working_directory():
    """检查工作目录"""
    print("\n📁 检查工作目录...")
    
    current_dir = Path.cwd()
    print(f"当前目录: {current_dir}")
    
    # 检查是否在正确的目录
    if 'ros_monitor_backend' not in current_dir.name:
        print("⚠️  建议在ros_monitor_backend目录下运行此脚本")
    
    # 检查虚拟环境
    venv_path = current_dir / '.venv'
    if venv_path.exists():
        print(f"✅ 虚拟环境存在: {venv_path}")
        
        # 检查虚拟环境是否激活
        if 'VIRTUAL_ENV' in os.environ:
            print("✅ 虚拟环境已激活")
        else:
            print("⚠️  虚拟环境未激活")
            print("   运行: source .venv/bin/activate")
    else:
        print("❌ 虚拟环境不存在")
        print("   运行: python3 -m venv .venv")
    
    return True

def check_network_ports():
    """检查网络端口"""
    print("\n🌐 检查网络端口...")
    
    ports_to_check = [8000, 5173]
    
    for port in ports_to_check:
        success, output = run_command(f"lsof -i :{port}", check=False)
        if success and output.strip():
            print(f"⚠️  端口 {port} 被占用:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"    {line}")
        else:
            print(f"✅ 端口 {port} 可用")
    
    return True

def check_system_info():
    """检查系统信息"""
    print("\n💻 检查系统信息...")
    
    print(f"操作系统: {platform.system()} {platform.release()}")
    print(f"架构: {platform.machine()}")
    print(f"Python路径: {sys.executable}")
    
    # 检查内存
    try:
        import psutil
        memory = psutil.virtual_memory()
        print(f"内存: {memory.total // (1024**3)} GB")
        print(f"可用内存: {memory.available // (1024**3)} GB")
    except ImportError:
        print("内存: 无法获取 (需要psutil包)")
    
    return True

def main():
    """主函数"""
    print("🔍 ROS监控系统环境检查")
    print("=" * 50)
    
    checks = [
        check_python_version,
        check_ros_environment,
        check_ros_topics,
        check_python_packages,
        check_working_directory,
        check_network_ports,
        check_system_info
    ]
    
    results = []
    for check in checks:
        try:
            result = check()
            results.append(result)
        except Exception as e:
            print(f"❌ 检查失败: {e}")
            results.append(False)
    
    print("\n" + "=" * 50)
    print("📊 检查结果汇总:")
    
    passed = sum(results)
    total = len(results)
    
    for i, (check, result) in enumerate(zip(checks, results)):
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{i+1:2d}. {check.__name__}: {status}")
    
    print(f"\n总计: {passed}/{total} 项检查通过")
    
    if passed == total:
        print("🎉 环境检查通过！可以启动系统")
        return True
    else:
        print("⚠️  环境检查未完全通过，请解决上述问题")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)