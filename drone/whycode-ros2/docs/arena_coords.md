# Arena coordinate system

To create an arena-specific coordinate system, you have to have four markers located in the corners of a rectangular arena and a [calibrated](https://docs.ros.org/en/ros2_packages/jazzy/api/camera_calibration/doc/tutorial_mono.html) camera.
Then, you have to set the node parameters `calib_dist_x` and `calib_dist_y` to set the distance between the markers in the x and y direction.
Either through the config file passed via the launch file

```yaml
...
calib_dist_x: 0.5
calib_dist_y: 0.4
```

or in runtime using the `ros2 param` tool.

```bash
ros2 param set /whycode_node calib_dist_x 0.5
ros2 param set /whycode_node calib_dist_y 0.4
```

Then run autocalibration, which uses the four outermost markers for the coordinate system calculation.

```bash
ros2 service call /whycode_node/set_calib_method whycode_interfaces/srv/SetCalibMethod "{method: 0}"  # AUTO
```

Afterwards, select the desired coordinate calibration, e.g. using just 2D planar coordinates of the robotic arena.

```bash
ros2 param set /whycode_node coords_method 1  # 2D coordinate
```

Verify that the reported coordinates are reported in the desired form.

```bash
ros2 topic echo /whycode_node/markers
```

If you are satisfied, you can save the calibration into a file that can be later used to avoid another calibration.

```bash
ros2 service call /whycode_node/set_calib_path whycode_interfaces/srv/SetCalibPath "{action: 0, path: '<path-to-yaml>'}"  # SAVE
ros2 service call /whycode_node/set_calib_path whycode_interfaces/srv/SetCalibPath "{action: 1, path: '<path-to-yaml>'}"  # LOAD
```
