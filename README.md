# UR5e Shape Detection & Pick-and-Place

Voice-commanded pick-and-place using a UR5e robot arm and Xbox 360 Kinect camera.
Say a command like `square 1` and the robot picks it and drops it in a fixed box.

---

## Setup

1. **Connect the hardware**
   - Plug in the Xbox 360 Kinect via USB
   - Connect the UR5e to the network (IP: `192.168.1.102`)
   - Make sure the Robotiq gripper is attached and powered

2. **Start the External Control program on the UR5e teach pendant**
   - On the pendant go to: `Program → URCaps → External Control`
   - Press Play

3. **Source the workspace** (in every new terminal)
   ```bash
   source ~/ur_driver/install/setup.bash
   ```

4. **Build** (only needed after code changes)
   ```bash
   cd ~/ur_driver
   colcon build
   source install/setup.bash
  
## Launch

### Real hardware
ros2 launch ur5e_grip_run_cpp full_system.launch.py use_fake_hardware:=false


### Simulation (no robot needed)
ros2 launch ur5e_grip_run_cpp full_system.launch.py use_fake_hardware:=true

## Voice Commands

Once everything is running, say:
```
square 1
circle 2
triangle 1
rectangle 1
```
The robot picks the object and places it in the drop box.

## for testing 
# Terminal 1:(after building and sourcing) camera test :
ros2 launch kinect_picking kinect_picking.launch.py

# Terminal 2
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 use_fake_hardware:=true

# Terminal 3
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_fake_hardware:=true

# Terminal 4
ros2 run ur5e_grip_run_cpp detection_bridge

# Terminal 5
ros2 run ur5e_grip_run_cpp grasp_executor

# Terminal 6
ros2 run ur5e_grip_run_cpp ik_commander --ros-args --params-file ~/ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml


