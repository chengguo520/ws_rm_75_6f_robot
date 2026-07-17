RM75-6F 官方资料最小归档
========================

目的
----
这个目录保存本仓库学习和实物调试需要的最小官方资料。

原始资料目录在本机：
/home/ros/机械臂

这里没有完整复制官方 Windows 上位机、DLL、STEP、全部历史版本资料，因为那些文件体积较大，
且不是本 ROS 工作区复现仿真/实物调试的必要输入。

已归档文件
----------
docs/realman_rm75_user_manual_v3_3.pdf
  来源：
  /home/ros/机械臂/RM-75系列机械臂/（1）用户手册/睿尔曼RM75系列机器人用户手册V3.3.pdf

  用途：
  学习 RM75 系列机械臂本体、安全、上电、网络、示教器、基础使用。

docs/realman_robot_cpp_api_v4_0_2.pdf
  来源：
  /home/ros/机械臂/睿尔曼机械臂接口函数说明(c++)V4.0.2.pdf

  用途：
  查询 C++ API，包括 socket 连接、运动接口、拖动示教、力位混合、六维力读取。

rm_base_cmake_example/
  来源：
  /home/ros/机械臂/rm_base_cmake_example

  用途：
  保留厂家 Linux x86 C++ SDK 示例、头文件和 libRM_Base.so.1.0.0。
  本仓库的实物力控包 rm_75_real_force_control 会优先链接这里的最小 SDK。

为什么不复制完整 /home/ros/机械臂
--------------------------------
完整目录中包含：
- Windows 上位机 exe/dll
- 多个重复版本的 Qt/VTK DLL
- 大型 STEP/STL 模型
- 历史会话导出文件
- WSL 配置/状态文件

这些文件不是从 0 学习 ROS/Gazebo/MoveIt/实物力控的必要内容。
如果后续某个实验确实需要更多官方资料，再按需复制并在本文件中记录来源。
