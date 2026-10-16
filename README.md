# ground_truth_localization

System ROS 2 Humble node that republishes a simulator's exact
`nav_msgs/msg/Odometry` from `/ground_truth/odom` as `/odom`. It copies the
incoming pose, body-frame twist, covariances, and timestamp without estimating
motion. It publishes the corresponding `odom -> base_link` dynamic TF and,
by default, an identity `map -> odom` static TF.

The input pose must be expressed in `map`; its `child_frame_id` must be
`base_link`. The node checks both frames and skips mismatches. Output `odom`
coordinates are numerically equal to input `map` coordinates. By default, the
node publishes an identity `map -> odom` transform, forcing the frames to
coincide. If that publication is disabled, another source must supply the same
identity transform; this node does not convert the pose through a nonidentity
transform.

## Build and run

Use a shell without the Isaac Sim conda environment. If conda is active,
deactivate it first:

```bash
unset LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select ground_truth_localization \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch ground_truth_localization ground_truth_localization.launch.py
```

The launch file defaults to `source_topic:=/ground_truth/odom`,
`odom_topic:=/odom`, `map_frame_id:=map`, `odom_frame_id:=odom`,
`base_frame_id:=base_link`, `publish_tf:=true`,
`publish_map_to_odom:=true`, and `use_sim_time:=true`. Set any of these as a
launch argument. The executable declares the same topic, frame, and transform
parameters for direct `ros2 run` use.

Run `python3 src/ground_truth_localization/test/verify_sim_integration.py`
from a third system ROS 2 shell while Isaac Sim and this node are running. It
checks the live DDS topics, matching Odometry data and timestamps, and both TF
edges. It starts neither process.
