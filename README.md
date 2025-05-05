# PS5 Controller Robot Arm Project

This project allows you to control a robot using a PS5 DualSense controller.

## File Structure

- **main.py:** Main application file handling the event loop, controller events, and UI updates.
- **joystick_handler.py:** Processes controller input, updates robot wheel commands, and publishes arm joint messages.
- **ws_client.py:** Manages the WebSocket connection to the ROSBridge server.
- **ui.py:** Implements the Pygame-based UI for displaying application data.
- **utils.py:** Contains helper functions (e.g., trigger value mapping, velocity limits).
- **README.md:** This documentation file.
- **mapping_tester.py:** For testing controller mapping.

## Requirements

- Python 3.12+
- A PS5 DualSense Wireless Controller
- A running ROSBridge server

## Controller Layout
![Controller Layout](https://github.com/alianlbj23/pros_ps5_general/blob/main/pic/joystick.jpg?raw=true)

## Setup

1. **Install Dependencies:**
   Install Pygame via pip:
   ```bash
   pip install -r .\requirements.txt
   ```

2. **Run ROSBridge Server:**
   Launch the ROSBridge server (adjust the command as per your ROS setup):
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

## Usage

1. **Start the Application:**
   ```bash
   python main.py
   ```

2. **Controlling the Robot:**

   - **Wheel Control:**
     - **Button 11:** Move forward
     - **Button 12:** Move backward
     - **Button 13:** Rotate counterclockwise
     - **Button 14:** Rotate clockwise
     - **Button 9 (L1):** Decrease speed
     - **Button 10 (R1):** Increase speed

     > *Wheel commands are published using the ROS message type `std_msgs/Float32MultiArray`.*

   - **Arm Control:**
     - **Button 1 (circle):** Increase the current joint's angle by 10°
     - **Button 2 (square):** Decrease the current joint's angle by 10°
     - **Button 3 (X):** Switch to the previous joint
     - **Button 0 (triangle):** Switch to the next joint
     - **Button 8 (right joystick):** Reset all joints to the preset angle

     > *Arm commands are published using the ROS message type `trajectory_msgs/msg/JointTrajectoryPoint`.*

3. **IP Input Mode:**
   - Press `I` to enter IP input mode for setting the ROSBridge server IP.
   - Press `Q` to disconnect and quit the application.

# config.csv
This CSV file is used to configure various aspects of the robot control system. It contains both **global** settings and individual **joint** definitions. The CSV file must include a header row with the following columns:
- **type**: Indicates the type of configuration.
  - Use "global" for general parameters.
  - Use "joint" for each individual joint's settings.
- **param**: The name of the parameter.
- **value1**: The primary value (e.g., port number, angle, topic name, etc.).
- **value2**: Additional value (if needed).

Test the mapping of your controller: 
  ```bash
  python mapping_tester.py
  ```
Change the values in config.csv to the corresponding ID of your controller

## Global Parameters

Global settings are defined on rows where `type` is **global**. Below is a description of each global parameter:

- **rosbridge_port**
  The port number used to connect to the rosbridge server.
  *Example*: `9090`

- **joints_count**
  The total number of joints for the robot arm.
  *Example*: `6`

- **angle_step**
  The default angle step (in degrees) used when adjusting the joint angles.
  *Example*: `15`

- **arm_topic**
  The topic name for controlling the robot arm.
  *Example*: `/robot_arm`

- **speed_step**
  The increment or decrement value for speeds.
  *Example*: `5`

- **front_wheel_topic**
  The topic name for the front wheels message.
  *Example*: `/car_C_front_wheel`

- **rear_wheel_topic**
  The topic name for the rear wheels message.
  *Example*: `/car_C_rear_wheel`

- **front_wheel_range**
  The range (in the format `start-end`) indicating which portion of the command array applies to the front wheels.
  *Example*: `0-2`

- **rear_wheel_range**
  The range (in the format `start-end`) indicating which portion of the command array applies to the rear wheels.
  *Example*: `2-4`

- **reset_arm_angle**
  The angle (in degrees) used to reset all joint angles when requested.
  *Example*: `30`

- **left_stick_horizontal**
  Axis ID for the left stick's horizontal movement (left-right)
  *Example*: `0`

- **left_stick_vertical**
  Axis ID for the left stick's vertical movement (up-down)
  *Example*: `1`

- **right_stick_horizontal**
  Axis ID for the right stick's horizontal movement (left-right)
  *Example*: `2`

- **right_stick_vertical**
  Axis ID for the right stick's vertical movement (up-down)
  *Example*: `3`

- **min_joystick_value**
  A minimum value for recognizing the joystick as moved to prevent drifting
  *Example*: `0.1`

## Joint Parameters

Each joint is described on rows where `type` is **joint**. The fields are:

- **param**: The joint number (as an identifier).
- **value1**: The lower limit of the joint (in degrees).
- **value2**: The upper limit of the joint (in degrees).

For example, a row with:
```
joint,1,0,180
```
indicates that joint #1 has a lower limit of 0° and an upper limit of 180°.


## Troubleshooting

- **Missing UI Indicator:**
  Ensure that the `arm_index` and `arm_angles` values are correctly updated and passed into `ui.draw()`.
- **ROS Connection Errors:**
  Check your ROSBridge server status and confirm the IP and port settings.

## License

This project is released under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contributing

Contributions are welcome! Feel free to open issues or submit pull requests for improvements or bug fixes.
