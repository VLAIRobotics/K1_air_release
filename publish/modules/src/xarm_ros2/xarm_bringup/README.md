# XArm 基础控制启动包

客户统一从 `publish/modules` 使用启动脚本：

```bash
bash src/xarm_ros2/start_xarm_ros2.sh mode:=single scene:=control hardware:=fake controller:=trajectory can_interface:=can0 single_side:=left rviz:=true
```

双臂规划可视化：

```bash
bash src/xarm_ros2/start_xarm_ros2.sh mode:=bimanual scene:=moveit hardware:=fake controller:=trajectory
```

`launch/xarm.launch.py`、`launch/xarm.bimanual.launch.py` 是统一入口内部使用的单臂/双臂基础控制实现。控制器参数在 `config/v10_controllers/`。不要与 MoveIt 完整入口重复启动。

完整参数、真机接线与排障见 [ROS 2 使用文档](../../../../../docs/用户使用文档/02_xarm_ros2使用文档.md)。
