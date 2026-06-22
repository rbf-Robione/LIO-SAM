# NavSatFix to Odometry Converter

This node converts `sensor_msgs/NavSatFix` (GPS) messages into `nav_msgs/Odometry` messages.

## Features

- Converts GPS coordinates (lat/lon/alt) to local Cartesian coordinates (x/y/z)
- Automatically uses the first GPS fix as the origin
- Performs accurate conversion using the WGS84 ellipsoid model
- Forwards GPS covariance to odometry covariance
- Estimates velocity from position differences
- Validates GPS fix quality

## Usage

### Starting the node

```bash
# Source the ROS 2 workspace
source install/setup.bash

# Run the node
ros2 run lio_sam lio_sam_navsat_to_odom
```

### Starting with a launch file

```bash
ros2 launch lio_sam navsat_to_odom.launch.py
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/gps/fix` | Topic to receive GPS messages from |
| `output_topic` | string | `/gps/odometry` | Topic to publish odometry messages to |
| `frame_id` | string | `odom` | Odometry frame ID |
| `child_frame_id` | string | `base_link` | Child frame ID |
| `use_first_fix_as_origin` | bool | `true` | Use the first GPS fix as origin |

## Topics

### Subscribe
- `input_topic` (`sensor_msgs/NavSatFix`): GPS fix messages

### Publish
- `output_topic` (`nav_msgs/Odometry`): Converted odometry messages

## Coordinate System

The node converts GPS coordinates to the ENU (East-North-Up) coordinate system:
- **X**: East direction
- **Y**: North direction
- **Z**: Up direction

## Examples

### Running with custom parameters

```bash
ros2 run lio_sam lio_sam_navsat_to_odom --ros-args \
  -p input_topic:=/my_gps/fix \
  -p output_topic:=/my_odom \
  -p frame_id:=map \
  -p child_frame_id:=gps_link
```

### Viewing topics

```bash
# Show GPS messages
ros2 topic echo /gps/fix

# Show odometry messages
ros2 topic echo /gps/odometry
```

## Notes

- The node uses the first valid GPS fix as the origin
- GPS fix quality must be `STATUS_FIX` or better
- Velocity is computed via simple finite differencing (only for intervals shorter than 1 second)
- An identity quaternion is used for orientation since GPS provides no heading information
