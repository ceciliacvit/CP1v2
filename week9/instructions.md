# TODOs for week9

## Install Camera Driver

1. Connect to the TB.
1. Install camera drivers:
    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-camera-ros ros-humble-image-transport-plugins v4l-utils
    ```
1. Create camera calibration file:
    ```bash
    mkdir -p /home/ubuntu/.ros/camera_info/
    touch /home/ubuntu/.ros/camera_info/imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml
    nano /home/ubuntu/.ros/camera_info/imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml
    ```
    Paste the content below to the file:
    ```bash
    image_width: 640
    image_height: 480
    camera_name: imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480
    frame_id: camera
    camera_matrix:
      rows: 3
      cols: 3
      data: [322.0704122808738, 0, 199.2680620421962, 0, 320.8673986158544, 155.2533082600705, 0, 0, 1]
    distortion_model: plumb_bob
    distortion_coefficients:
      rows: 1
      cols: 5
      data: [0.1639958233797625, -0.271840030972792, 0.001055841660100477, -0.00166555973740089, 0]
    rectification_matrix:
      rows: 3
      cols: 3
      data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    projection_matrix:
      rows: 3
      cols: 4
      data: [329.2483825683594, 0, 198.4101510452074, 0, 0, 329.1044006347656, 155.5057121208347, 0, 0, 0, 1, 0]
    ```

## Exercise

1. Get the the main [file](week9/robot_vision.m) and the [file](week9/TurtleBotVisualise.m) to visualise the robot and video stream.
1. Run TB node:
    ```bash
    ros2 launch turtlebot3_bringup robot.launch.py
    ```
1. Run camera node:
    ```bash
    ros2 run camera_ros camera_node --ros-args -p format:='RGB888' -p width:=640 -p height:=480
    ```
1. Finish Exercise 1 and Exercise 2.