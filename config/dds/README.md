Use the matching CycloneDDS config on each machine to force unicast discovery between:

- Laptop/container: `192.168.123.222`
- G1: `192.168.123.164`

Set on the laptop/container:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/rodrigo/git-repo/GR00T-WholeBodyControl/config/dds/cyclonedds_laptop_192.168.123.222.xml
```

Set on the G1:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/rodrigo/git-repo/GR00T-WholeBodyControl/config/dds/cyclonedds_g1_192.168.123.164.xml
```

Then restart all ROS 2 processes:

- teleop policy loop
- G1 control loop
- data exporter

Useful checks:

```bash
echo $CYCLONEDDS_URI
ros2 service list | grep WBCPolicy/robot_config
ros2 topic list | grep -E 'G1Env/env_state_act|Gr00tKeyboardListener'
```
