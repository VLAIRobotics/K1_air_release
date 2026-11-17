# xarm_ros2

统一入口：`bash start_xarm_ros2.sh --help`。

双臂假硬件可视化：

```bash
bash start_xarm_ros2.sh mode:=bimanual scene:=moveit hardware:=fake controller:=trajectory
```

详细安装、真机 CAN 参数、控制器和排障见 [ROS 2 使用文档](../../../../docs/用户使用文档/02_xarm_ros2使用文档.md)。

- xarm：元包。
- xarm_hardware：ros2_control 到 CAN SDK 的硬件接口。
- xarm_bringup：基础控制启动和控制器配置。
- xarm_bimanual_moveit_config：双臂 MoveIt 与 RViz。

launch 文件作为底层实现保留，客户统一使用上述脚本；脚本不再接受旧命令格式。包内 test 是开发回归源文件，正常启动不执行；客户编译可设置 `-DBUILD_TESTING=OFF`。
