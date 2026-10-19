# Roboost

Mecanum robot with two ESP32 boards that talk micro-ROS (ROS 2 Jazzy) over WiFi.

![Roboost Demo](res/Roboost-Demo.gif "Roboost Demo")

| Path | Contents |
| --- | --- |
| `firmware/` | PlatformIO project, one env per board: `motor_board`, `sensor_board` |
| `ros2/roboost_bringup/` | URDF, bringup launch, RViz config, odometry relay |
| `.devcontainer/` | ROS 2 Jazzy + PlatformIO container, micro-ROS agent as a sidecar |

## Bringup

1. **Dev container**: open the repo in VS Code and *Reopen in Container* (Linux host). This also starts the
   micro-ROS agent on UDP port 8888 and builds the ROS workspace. For RViz run `xhost +local:` on the host once.

2. **WiFi**: `cp firmware/include/wifi_config.example.h firmware/include/wifi_config.h` and fill in SSID,
   password and `AGENT_IP` (the IP of the machine running the container).

3. **Flash** each board over USB (one plugged in at a time):

   ```bash
   cd firmware
   pio run -e motor_board -t upload
   pio run -e sensor_board -t upload
   ```

   LED blinking = waiting for the agent, solid = connected (`pio device monitor` shows the log).
   Keep the robot still while the sensor board boots, it calibrates the gyro.

4. **Run**:

   ```bash
   ros2 launch roboost_bringup bringup.launch.py rviz:=true   # add slam:=true to map
   ros2 run teleop_twist_keyboard teleop_twist_keyboard        # hold shift to strafe
   ```

5. **First drive, wheels off the ground**: forward must turn every wheel forward. A wheel turns backwards:
   flip its `invert`. A wheel runs away at full speed: swap its `enc_a`/`enc_b`. Both, plus pins, geometry and
   PID gains, are in `firmware/src/motor_board/config.h`; sensor pins in `firmware/src/sensor_board/config.h`.

No dev container? Any ROS 2 Jazzy install works, run the agent with
`docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888`.

## Topics

| Topic | Type | Source | QoS |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | you (stops after 0.5 s silence) | best effort subscriber |
| `/joint_states` | `sensor_msgs/JointState` (effort = PWM duty) | motor board, 20 Hz | best effort |
| `/wheel_odom` | `nav_msgs/Odometry` | motor board, 20 Hz | best effort |
| `/odom`, TF `odom -> base_footprint` | `nav_msgs/Odometry` | `odom_tf` relay | reliable |
| `/scan` | `sensor_msgs/LaserScan`, 360 beams | sensor board, ~5.5 Hz | best effort |
| `/imu/data_raw` | `sensor_msgs/Imu` (no orientation) | sensor board, 50 Hz | best effort |
| `/imu/temperature`, `/battery` | `Temperature`, `BatteryState` | sensor board, 1 Hz | best effort |

The boards only publish best effort: a reliable micro-ROS publisher blocks until the agent acknowledges each
message, which stalls everything on a lossy WiFi link. `firmware/microros.meta` raises the MTU to 2 KB so a full
scan still fits into a single best-effort packet. Kinematics, PID and the lidar parser have host tests:
`pio test -e native`.
