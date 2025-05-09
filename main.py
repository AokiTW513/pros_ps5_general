import csv
import pygame
from ui import UI
from ws_client import RosbridgeClient
from joystick_handler import JoystickHandler

def load_rosbridge_port(filename="config.csv"):
    try:
        with open(filename, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row["type"] == "global" and row["param"] == "rosbridge_port":
                    return int(row["value1"])
    except Exception as e:
        print("Error loading rosbridge_port from CSV:", e)
    return 9090  # 預設值

def publish_wheel(ws_client, cmd, front_topic, rear_topic, front_range, rear_range):
    # 建立後輪與前輪的完整訊息（std_msgs/Float32MultiArray）
    rear_msg = {
        "layout": {
            "dim": [{
                "label": "rear_wheels",
                "size": front_range[1] - front_range[0],  # 可依實際需求調整
                "stride": front_range[1] - front_range[0]
            }],
            "data_offset": 0
        },
        "data": cmd[rear_range[0]:rear_range[1]]
    }
    front_msg = {
        "layout": {
            "dim": [{
                "label": "front_wheels",
                "size": rear_range[1] - rear_range[0],
                "stride": rear_range[1] - rear_range[0]
            }],
            "data_offset": 0
        },
        "data": cmd[front_range[0]:front_range[1]]
    }
    
    ws_client.publish(rear_topic, rear_msg)
    ws_client.publish(front_topic, front_msg)

def main():
    pygame.init()
    clock = pygame.time.Clock()
    ui = UI()
    # 從 CSV 中讀取 rosbridge_port
    rosbridge_port = load_rosbridge_port()
    ws_client = RosbridgeClient(rosbridge_port=rosbridge_port)
    joystick_handler = JoystickHandler()

    joysticks = {}

    # 初始狀態：輸入 IP 模式
    input_mode = True
    ip_input = ""
    connection_error = ""
    rosbridge_ip = ""

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                # 當處於 IP 輸入模式時，累積使用者輸入
                if input_mode:
                    if event.key == pygame.K_RETURN:
                        if ip_input:
                            rosbridge_ip = ip_input
                            if ws_client.connect(rosbridge_ip):
                                connection_error = ""
                                # 連線成功後 advertise topic
                                ws_client.advertise_topic(joystick_handler.rear_wheel_topic, "std_msgs/Float32MultiArray")
                                ws_client.advertise_topic(joystick_handler.front_wheel_topic, "std_msgs/Float32MultiArray")
                                # Advertise arm topic
                                ws_client.advertise_topic(joystick_handler.arm_topic, "trajectory_msgs/JointTrajectoryPoint")
                            else:
                                connection_error = "Connection failed"
                        input_mode = False
                        ip_input = ""
                    elif event.key == pygame.K_BACKSPACE:
                        ip_input = ip_input[:-1]
                    else:
                        ip_input += event.unicode
                else:
                    if event.key == pygame.K_i:
                        input_mode = True
                        ip_input = ""
                    elif event.key == pygame.K_q:
                        ws_client.disconnect()
                        running = False

            if not input_mode:
                if event.type == pygame.JOYBUTTONDOWN:
                    joystick_handler.process_button_press(
                        event.button,
                        wheel_publish_callback=lambda cmd: publish_wheel(ws_client, cmd,
                            joystick_handler.front_wheel_topic,
                            joystick_handler.rear_wheel_topic,
                            joystick_handler.front_wheel_range,
                            joystick_handler.rear_wheel_range),
                        arm_publish_callback=lambda arm_msg: ws_client.publish(joystick_handler.arm_topic, arm_msg)
                    )
                # elif event.type == pygame.JOYAXISMOTION:
                #     joystick_handler.process_axis_motion(
                #         event.axis, 
                #         event.value, 
                #         wheel_publish_callback=lambda cmd: publish_wheel(ws_client, cmd,
                #             joystick_handler.front_wheel_topic,
                #             joystick_handler.rear_wheel_topic,
                #             joystick_handler.front_wheel_range,
                #             joystick_handler.rear_wheel_range)
                #     )
                else:
                    pass

             # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")
        
        #continuously pull joystick data instead of waiting for events (for 0s)
        if pygame.joystick.get_count() > 0:
            joystick_handler.process_joystick_continous(
                            joysticks, 
                            wheel_publish_callback=lambda cmd: publish_wheel(ws_client, cmd,
                            joystick_handler.front_wheel_topic,
                            joystick_handler.rear_wheel_topic,
                            joystick_handler.front_wheel_range,
                            joystick_handler.rear_wheel_range)
                    )

        connection_status = "Connected" if ws_client.ws else "Disconnected"
        ui.draw(
            joystick_handler.velocity,
            joystick_handler.angle_step_deg,
            rosbridge_ip,
            connection_status,
            connection_error,
            input_mode,
            ip_input,
            joystick_handler.arm_index,
            joystick_handler.arm_angles,
            joystick_handler.wheel_speed,
            joystick_handler.isUnity
        )
        clock.tick(30)

    ws_client.disconnect()
    pygame.quit()

if __name__ == "__main__":
    main()
