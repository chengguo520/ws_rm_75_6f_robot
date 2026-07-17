# RM75-6F 学习导航

这个目录保存 RM75-6F 从 ROS/Gazebo 基础到柔顺擦桌子、再到真实机器人接口的学习记录。
现有实验笔记已被其他文档和代码路径引用，因此这里不批量改名；以本页作为稳定入口，按顺序学习即可。

## 当前建议主线：仿真柔顺擦桌子

当前没有实物调试计划时，优先完成这一条主线。`sim_09` 使用的是 Gazebo 中的虚拟接触力，
不是物理六维力传感器；真实接触力闭环属于后续阶段。

1. [从 0 开始完整路线](rm75_6f_from_zero_complete_learning_guide.txt)
2. [仿真实验总览：场景、视觉接近、抓取](rm75_6f_sim_experiments_guide.txt)
3. [虚拟外力导纳：先理解 M/D/K 与平衡位姿](rm75_6f_admittance_virtual_force_experiment.txt)
4. [连续发布关节命令：理解为何需要流式控制](rm75_6f_admittance_streaming_command_experiment.txt)
5. [IK 位置映射：理解导纳输出如何变为末端目标](rm75_6f_admittance_ik_position_experiment.txt)
6. [Jacobian 速度映射：进入连续笛卡尔柔顺控制](rm75_6f_admittance_jacobian_velocity_experiment.txt)
7. [关节命令链路最小验证](rm75_6f_joint_command_sanity_experiment.txt)
8. [桌面直线与 TCP 标定](rm75_6f_waypoint_table_line_calibration_experiment.txt)
9. [sim_09：直线擦拭与 z 轴导纳](rm75_6f_admittance_table_wiping_experiment.txt)
10. [M/D/Fd 控制变量调参记录](rm75_sim09_m_d_fd_tuning_20260710.txt)
11. [XY 平滑换向与误差拆分](rm75_sim09_xy_smooth_turn_20260711.txt)
12. [曲面高度与擦拭速度鲁棒性验证](rm75_sim09_robustness_20260710.txt)

完成第 12 步后，当前仿真阶段的结论是：M=1.5 kg、D=65 N s/m、Fd=6 N 在已测关键工况中通过。
下一步应在 Gazebo 中加入真实碰撞与 F/T 传感器，先记录测得法向力，再替换虚拟力输入。

## 辅助学习资料

- [后续学习路线](rm75_6f_next_learning_steps.txt)：项目级学习目标与阶段划分。
- [Gazebo 世界、相机、夹爪链路](rm75_6f_sim_world_camera_gripper_pipeline.txt)：仿真场景扩展时阅读。
- [sim_06 分层与升级计划](rm75_sim06_code_layering_notes.txt)、[升级迭代计划](rm75_sim06_upgrade_iteration_plan.txt)：回顾导纳实验的演进。
- [点位保存工具](rm75_6f_waypoint_tool_tutorial.txt) 与 [点位数据](rm75_6f_waypoints.yaml)：学习示教点与复现。
- [Git 教程](git_tutorial_for_rm75_6f_robot.txt) 与 [工程流程简记](flow.txt)：日常工程操作参考。

## 真实机器人资料：单独阅读，不与仿真主线混用

以下资料为未来连接实物时使用。实物上必须先小速度、小范围、只读验证，不直接复用仿真中的速度和力参数。

1. [ROS 与实物启动说明](rm75_6f_real_robot_ros_notes.txt)
2. [真实机器人柔顺控制学习计划](rm75_6f_real_compliance_learning_plan.txt)
3. [六维力只读调试](rm75_6f_real_force_read_only_debug.txt)
4. [真实力控 API 说明](rm75_6f_real_force_compliance_api_notes.txt)
5. [真实力控代码走读](rm75_6f_real_force_control_code_walkthrough.txt)
6. [拖动示教排错](rm75_6f_real_drag_teach_debug.txt)
7. [实物柔顺操作指南](rm75_6f_real_robot_compliance_operation_guide.txt)
8. [示教器教程](rm75_6f_teach_pendant_tutorial.txt)

## 版本控制范围

- 本页和 `study/` 下的 `.txt`、`.yaml` 学习资料会随 Git 推送到 GitHub。
- `study/data/sim09_runs/` 下的 CSV、PNG 是可再生实验输出，受 `.gitignore` 排除，不上传。
- 历史对话导出文件不作为学习主线入口；保留与否应单独决定，避免把无关聊天附件混入公开仓库。
