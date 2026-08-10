# Node interface

## Behavior

- Upon receiving a camera image, it detects circular markers using the WhyCode algorithm and publishes the marker array.
- Optionally overlays debug visuals if enabled.
- Parameters are dynamically reconfigurable via the ROS parameter interface, with validation.
- Read only parameters are: `calib_file`, `img_transport`, `img_base_topic`, `info_topic`, `id_bits`, `id_samples`, `hamming_dist`.
- Calibration information can be loaded from or saved to a YAML file.
- The default node name is `/whycode_node` using the [`whycode_launch.py`](../launch/whycode_launch.py)

## Parameters

| Name                              | Type   | Description                                                   | Default              |
|-----------------------------------|--------|---------------------------------------------------------------|----------------------|
| `calib_file`                      | string | Coordinate calibration file path (Read-only)                  |                      |
| `img_transport`                   | string | Subscribed image topic transport type (Read-only)             | "compressed"         |
| `img_base_topic`                  | string | Base name of the subscribed image topic (Read-only)           |                      |
| `info_topic`                      | string | Camera info topic associated with the image topic (Read-only) |                      |
| `id_bits`                         | int    | Number of encoded bits (Read-only)                            |                      |
| `id_samples`                      | int    | Number of samples along the perimeter (Read-only)             | 360                  |
| `hamming_dist`                    | int    | Encoded ID hamming distance (Read-only)                       | 1                    |
| `draw_coords`                     | bool   | Draw coordinate systems in debug image                        | true                 |
| `draw_segments`                   | bool   | Highlight found markers in debug image                        | true                 |
| `use_gui`                         | bool   | Enable any drawings in debug image                            | true                 |
| `identify`                        | bool   | Enable WhyCode detection                                      | true                 |
| `coords_method`                   | int    | Coordinates transformation method                             | 0                    |
| `num_markers`                     | int    | Number of markers to detect                                   | 1                    |
| `min_size`                        | int    | Minimum marker size in pixels                                 | 100                  |
| `circle_diameter`                 | double | Marker outer diameter in meters                               | 0.122                |
| `calib_dist_x`                    | double | Distance between markers in X direction                       | 1.0                  |
| `calib_dist_y`                    | double | Distance between markers in Y direction                       | 1.0                  |
| `initial_circularity_tolerance`   | double | Initial circularity test tolerance (percent)                  | 100.0                |
| `final_circularity_tolerance`     | double | Final circularity test tolerance (percent)                    | 2.0                  |
| `area_ratio_tolerance`            | double | Tolerance of black and white area ratios (percent)            | 40.0                 |
| `center_distance_tolerance_ratio` | double | Concentricity ratio tolerance (percent)                       | 10.0                 |
| `center_distance_tolerance_abs`   | double | Absolute concentricity tolerance (pixels)                     | 5.0                  |

### Example CLI Calls

```bash
ros2 param set /whycode_node <param-name> <param-value>
ros2 param get /whycode_node <param-name>
```

## Subscribed Topics

| Topic              | Type                         | Description                                  |
|--------------------|------------------------------|----------------------------------------------|
| `<img_base_topic>` | `sensor_msgs/msg/Image`      | `image_transport` input image for processing |
| `<info_topic>`     | `sensor_msgs/msg/CameraInfo` | Associated camera info                       |

## Published Topics

| Topic           | Type                                 | Description                                    |
|-----------------|--------------------------------------|------------------------------------------------|
| `~/debug_image` | `sensor_msgs/msg/Image`              | `image_transport` debug image (if GUI enabled) |
| `~/markers`     | `whycode_interfaces/msg/MarkerArray` | Array of detected markers                      |
| `~/discovery`   | `whycode_interfaces/msg/Discovery`   | Discovery signal message                       |

## Services

| Service Name         | Type                                    | Description                                    |
|----------------------|-----------------------------------------|------------------------------------------------|
| `~/set_calib_method` | `whycode_interfaces/srv/SetCalibMethod` | Select calibration method (AUTO=0, MANUAL=1)   |
| `~/set_calib_path`   | `whycode_interfaces/srv/SetCalibPath`   | Load or save calibration file (SAVE=0, LOAD=1) |
| `~/select_marker`    | `whycode_interfaces/srv/SelectMarker`   | Select marker based on image pixel coordinate  |

### Example CLI Calls

```bash
ros2 service call /whycode_node/set_calib_method whycode_interfaces/srv/SetCalibMethod "{method: 0}"  # AUTO
ros2 service call /whycode_node/set_calib_method whycode_interfaces/srv/SetCalibMethod "{method: 1}"  # MANUAL

ros2 service call /whycode_node/set_calib_path whycode_interfaces/srv/SetCalibPath "{action: 0, path: '/path/to/calib.yaml'}"  # SAVE
ros2 service call /whycode_node/set_calib_path whycode_interfaces/srv/SetCalibPath "{action: 1, path: '/path/to/calib.yaml'}"  # LOAD

ros2 service call /whycode_node/select_marker whycode_interfaces/srv/SelectMarker "{point: {x: 100.0, y: 200.0, z: 0.0}}"  # (x,y) px
