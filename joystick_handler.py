import pygame
import time
import math
import csv
from utils import map_trigger_value, vel_limit, angle_limit

class JoystickHandler:
    def __init__(self):
        
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick detected.")
            self.joystick = None
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()  # 初始化搖桿
            print(f"Detected Joystick: {self.joystick.get_name()}")

        # 預設值
        self.velocity = 10.0
        self.arm_joints_count = 7 # 手臂數量
        self.arm_angles = [0.0] * self.arm_joints_count
        self.arm_realangles = [0.0] * self.arm_joints_count
        self.arm_angles_Unity_offset = [10, -60, -70, -90, -90, -90,-70]
        self.joint_limits = [(0.0, math.radians(180)) for _  in range(self.arm_joints_count)]
        self.joint_limits_unity = [(0.0, math.radians(180)) for _  in range(self.arm_joints_count)]
        self.arm_index = 0
        self.arm_topic = "/robot_arm"
        self.angle_step_deg = 10.0    # 每次增/減的角度 (預設 10 度)
        self.speed_incr = 5.0         # 每次加減的速度值 (預設 5)
        self.angle_step_deg_change = 5.0 # 每次增/減角度變化的角度 (預設 5 度)
        
        # 預設前後輪 topic 與 cmd 讀取範圍
        self.front_wheel_topic = "/car_C_front_wheel"
        self.rear_wheel_topic  = "/car_C_rear_wheel"
        self.front_wheel_range = (0, 2)   # 默認讀取 cmd[0:2]
        self.rear_wheel_range  = (2, 4)   # 默認讀取 cmd[2:4]

        self.reset_arm_angle = [90, 10, 160, 90, 90, 90 ,70]  # 初始化 reset_arm_angle 屬性

        # 下面註解是以前讀取用的，現在改成可以直接修改csv檔就可以重設預設值了
        #self.arm_defaults = [90, 10, 160, 90, 90, 90 ,70]

        #controller joystick axis
        self.left_stick_horizontal = 0
        self.left_stick_vertical = 1
        self.right_stick_horizontal = 2
        self.right_stick_vertical = 3

        #controller joystick button
        self.front_button = 11 #前進
        self.back_button = 12 #後退
        self.left_button = 13 #左轉
        self.right_button = 14 #右轉
        self.stop_button = 8 #停止
        self.resetArm_button = 10 #重設所有手臂角度為 CSV 設定的值
        self.deceleration_button = 4 #減速
        self.acceleration_button = 5 #加速
        self.armAnglePlus_button = 1 #增加當前關節角度
        self.armAngleMinus_button = 2 #減少當前關節角度
        self.nextArm_button = 0 #下一個關節
        self.previousArm_button = 3 #上一個關節
        self.armAngleStepDegPlus_button = 7 #增加關節角度變化
        self.armAngleStepDegMinus_button = 6 #減少關節角度變化
        self.isUnityButton = 9
        self.isUnity = False

        self.wheel_speed = [0, 0, 0, 0] #wheel speed for gui

        #minimal joystick value to prevent drifting
        self.min_joystick_value = 0.1


        # 從 CSV 載入設定
        self.load_config("config.csv")

        # 先在進去後重設所有手臂角度，不然角度都會為0
        self.arm_realangles = [math.radians(deg) for deg in self.reset_arm_angle]
        self.clip_arm_angles()

    def load_config(self, filename="config.csv"):
        """
        讀取 CSV 檔案，格式範例如下（含表頭）：

        type,param,value1,value2
        global,rosbridge_port,9090,
        global,joints_count,7,
        global,angle_step,10,
        global,arm_topic,/robot_arm,
        global,speed_step,5,
        global,front_wheel_topic,/car_C_front_wheel,
        global,rear_wheel_topic,/car_C_rear_wheel,
        global,front_wheel_range,0-2,
        global,rear_wheel_range,2-4,
        global,reset_arm_angle,"90, 10, 160, 90, 90, 90, 70",
        global,angle_step_deg_change,1.0,
        joint,1,0,180
        joint,2,0,180
        joint,3,0,180
        joint,4,0,180
        joint,5,0,180
        joint,6,0,180
        global,left_stick_horizontal,0,
        global,left_stick_vertical,1,
        global,right_stick_horizontal,3,
        global,right_stick_vertical,4,
        global,min_joystick_value,0.1,
        global,fornt_button,11,
        global,back_button,12,
        global,left_button,13,
        global,right_button,14,
        global,stop_button,8,
        global,resetArm_button,10,
        global,deceleration_button,4,
        global,acceleration_button,5,
        global,armAnglePlus_button,1,
        global,armAngleMinus_button,2,
        global,nextArm_button,3,
        global,previousArm_button,0,
        global,armAngleStepDegPlus_button,11,
        global,armAngleStepDegMinus_button,12,
        """
        try:
            with open(filename, "r", newline='') as f:
                reader = csv.DictReader(f)
                global_params = {}
                joint_rows = []
                jointunity_rows = [] 
                for row in reader:
                    if row["type"] == "global":
                        global_params[row["param"]] = row["value1"]
                    elif row["type"] == "joint":
                        joint_rows.append(row)
                    elif row["type"] == "jointunity":
                        jointunity_rows.append(row)
            # 全域參數讀取
            if "joints_count" in global_params:
                self.arm_joints_count = int(global_params["joints_count"])
            if "angle_step" in global_params:
                self.angle_step_deg = float(global_params["angle_step"])
            if "arm_topic" in global_params and global_params["arm_topic"]:
                self.arm_topic = global_params["arm_topic"]
            if "speed_step" in global_params:
                self.speed_incr = float(global_params["speed_step"])
            if "front_wheel_topic" in global_params and global_params["front_wheel_topic"]:
                self.front_wheel_topic = global_params["front_wheel_topic"]
            if "rear_wheel_topic" in global_params and global_params["rear_wheel_topic"]:
                self.rear_wheel_topic = global_params["rear_wheel_topic"]
            if "front_wheel_range" in global_params:
                try:
                    parts = global_params["front_wheel_range"].split("-")
                    self.front_wheel_range = (int(parts[0]), int(parts[1]))
                except:
                    self.front_wheel_range = (0, 2)
            #新的讀取角度預設值
            if "reset_arm_angle" in global_params and global_params["reset_arm_angle"]:
                val = global_params["reset_arm_angle"]
                if isinstance(val, str) and "," in val:
                    self.reset_arm_angle = [float(x.strip()) for x in val.split(",")]
                else:
                    self.reset_arm_angle = float(val)
            else:
                self.reset_arm_angle = 0.0
            # 下面註解是上面那個讀取預設值用的以前的版本，不過只能讀取一個，現在上面改成可以讀取全部，並把它用","來分割掉，變成List
            # if "reset_arm_angle" in global_params and global_params["reset_arm_angle"]:
            #     self.reset_arm_angle = float(global_params["reset_arm_angle"])
            # else:
            #     self.reset_arm_angle = 0.0
            if "rear_wheel_range" in global_params:
                try:
                    parts = global_params["rear_wheel_range"].split("-")
                    self.rear_wheel_range = (int(parts[0]), int(parts[1]))
                except:
                    self.rear_wheel_range = (2, 4)
            
            if "left_stick_horizontal" in global_params:
                self.left_stick_horizontal = int (global_params["left_stick_horizontal"])
            if "left_stick_vertical" in global_params:
                self.left_stick_vertical = int (global_params["left_stick_vertical"])
            if "right_stick_horizontal" in global_params:
                self.right_stick_horizontal = int (global_params["right_stick_horizontal"])
            if "right_stick_vertical" in global_params:
                self.right_stick_vertical = int (global_params["right_stick_vertical"])
            if "min_joystick_value" in global_params:
                self.min_joystick_value = float (global_params["min_joystick_value"])

            #角度變化預設值
            if "angle_step_deg_change" in global_params:
                self.angle_step_deg_change = float (global_params["angle_step_deg_change"])
            #按鈕設定
            if "front_button" in global_params:
                self.front_button = int(global_params["front_button"])
            if "back_button" in global_params:
                self.back_button = int(global_params["back_button"])
            if "left_button" in global_params:
                self.left_button = int(global_params["left_button"])
            if "right_button" in global_params:
                self.right_button = int(global_params["right_button"])
            if "stop_button" in global_params:
                self.stop_button = int(global_params["stop_button"])
            if "resetArm_button" in global_params:
                self.resetArm_button = int(global_params["resetArm_button"])
            if "deceleration_button" in global_params:
                self.deceleration_button = int(global_params["deceleration_button"])
            if "acceleration_button" in global_params:
                self.acceleration_button = int(global_params["acceleration_button"])
            if "armAnglePlus_button" in global_params:
                self.armAnglePlus_button = int(global_params["armAnglePlus_button"])
            if "armAngleMinus_button" in global_params:
                self.armAngleMinus_button = int(global_params["armAngleMinus_button"])
            if "nextArm_button" in global_params:
                self.nextArm_button = int(global_params["nextArm_button"])
            if "previousArm_button" in global_params:
                self.previousArm_button = int(global_params["previousArm_button"])
            if "armAngleStepDegPlus_button" in global_params:
                self.armAngleStepDegPlus_button = int(global_params["armAngleStepDegPlus_button"])
            if "armAngleStepDegMinus_button" in global_params:
                self.armAngleStepDegMinus_button = int(global_params["armAngleStepDegMinus_button"])
            if "isUnityButton" in global_params:
                self.isUnityButton = int(global_params["isUnityButton"])
            if "arm_angles_Unity_offset" in global_params and global_params["arm_angles_Unity_offset"]:
                val = global_params["arm_angles_Unity_offset"]
                if isinstance(val, str) and "," in val:
                    self.arm_angles_Unity_offset = [float(x.strip()) for x in val.split(",")]
                else:
                    self.arm_angles_Unity_offset = float(val)
            else:
                self.arm_angles_Unity_offset = 0.0
            
            jointunity_rows.sort(key=lambda x: int(x["param"]))  # 根據 joint 編號排序
            self.joint_limits_unity = []
            for i in range(self.arm_joints_count):
                if i < len(jointunity_rows):
                    lower_deg = float(jointunity_rows[i]["value1"])
                    upper_deg = float(jointunity_rows[i]["value2"])
                    self.joint_limits_unity.append((math.radians(lower_deg), math.radians(upper_deg)))
                else:
                    self.joint_limits_unity.append((0.0, math.radians(180)))
            
            # 讀取各關節上下限
            joint_rows.sort(key=lambda x: int(x["param"]))  # 根據 joint 編號排序
            self.joint_limits = []
            for i in range(self.arm_joints_count):
                if i < len(joint_rows):
                    lower_deg = float(joint_rows[i]["value1"])
                    upper_deg = float(joint_rows[i]["value2"])
                    self.joint_limits.append((math.radians(lower_deg), math.radians(upper_deg)))
                else:
                    self.joint_limits.append((0.0, math.radians(180)))
            self.arm_realangles = [0.0] * self.arm_joints_count
            self.arm_index = 0
            print(f"Loaded config: {self.arm_joints_count} joints, angle step {self.angle_step_deg} deg, speed step {self.speed_incr},")
            print(f"arm topic: {self.arm_topic}, front wheel topic: {self.front_wheel_topic}, rear wheel topic: {self.rear_wheel_topic}")
            print(f"front wheel range: {self.front_wheel_range}, rear wheel range: {self.rear_wheel_range}")
        except FileNotFoundError:
            print(f"Config CSV '{filename}' not found, using defaults.")
        except Exception as e:
            print("Error loading config CSV:", e)

    def clip_arm_angles(self):
        """將各關節角度限制在上下限之間（弧度）"""
        if self.isUnity:
            for i in range(self.arm_joints_count):
                lower, upper = self.joint_limits_unity[i]
                self.arm_realangles[i] = max(lower, min(self.arm_realangles[i], upper))
                print("Joint" + str([i]) + " : " + str(self.arm_realangles[i]))
                print("CSVJoint" + str([i]) + " : " + str(self.joint_limits_unity[i]))
        else:
            for i in range(self.arm_joints_count):
                lower, upper = self.joint_limits[i]
                self.arm_realangles[i] = max(lower, min(self.arm_realangles[i], upper))
                print("Joint" + str([i]) + " : " + str(self.arm_realangles[i]))
                print("CSVJoint" + str([i]) + " : " + str(self.joint_limits[i]))

    def set_joint_count(self, count):
        """指定關節數量（不從檔案時使用）"""
        self.arm_joints_count = count
        self.arm_realangles = [0.0] * count
        self.joint_limits = [(0.0, math.radians(180)) for _ in range(count)]
        self.arm_index = 0

    def changeAngleWhenUnity(self):
        if self.isUnity:
            for i in range(len(self.arm_realangles)):
                self.arm_realangles[i] += math.radians(self.arm_angles_Unity_offset[i])
        else:
            for i in range(len(self.arm_realangles)):
                self.arm_realangles[i] -= math.radians(self.arm_angles_Unity_offset[i])

    def process_button_press(self, button, wheel_publish_callback, arm_publish_callback):
        # 轉換步進角度為弧度
        step_radians = math.radians(self.angle_step_deg)

        if button == self.front_button:  # 前進
            wheel_publish_callback([self.velocity, self.velocity, self.velocity, self.velocity])
        elif button == self.back_button:  # 後退
            wheel_publish_callback([-self.velocity, -self.velocity, -self.velocity, -self.velocity])
        elif button == self.left_button:  # 左轉
            wheel_publish_callback([-self.velocity, self.velocity, -self.velocity, self.velocity])
        elif button == self.right_button:  # 右轉
            wheel_publish_callback([self.velocity, -self.velocity, self.velocity, -self.velocity])
        elif button == self.stop_button:   # 停止
            wheel_publish_callback([0.0, 0.0, 0.0, 0.0])
        elif button == self.resetArm_button:   # Start鍵：重設所有手臂角度為 CSV 設定的值
            self.arm_realangles = [math.radians(deg) for deg in self.reset_arm_angle]
            if self.isUnity:
                for i in range(len(self.arm_realangles)):
                    self.arm_realangles[i] += math.radians(self.arm_angles_Unity_offset[i])
            self.clip_arm_angles()
            arm_publish_callback({"positions": self.arm_realangles})
        elif button == self.isUnityButton:
            self.isUnity = not self.isUnity
            if self.isUnity:
                for i in range(len(self.arm_realangles)):
                    self.arm_realangles[i] += math.radians(self.arm_angles_Unity_offset[i])
            else:
                for i in range(len(self.arm_realangles)):
                    self.arm_realangles[i] -= math.radians(self.arm_angles_Unity_offset[i])
            print(self.isUnity)
            #下面是原本的代碼，會把所有手臂角度轉成CSV設定的值，不過應該可能只能設定一個
            #reset_val = math.radians(self.reset_arm_angle)
            #self.arm_angles = [reset_val] * self.arm_joints_count
            #reset_val = math.radians(90)
            #self.arm_angles = [math.radians(deg) for deg in self.arm_defaults]
        elif button == self.deceleration_button:   # L1：減速
            self.velocity -= self.speed_incr
            self.velocity = vel_limit(self.velocity)
        elif button == self.acceleration_button:  # R1：加速
            self.velocity += self.speed_incr
            self.velocity = vel_limit(self.velocity)
        elif button == self.armAnglePlus_button:   # B：增加當前關節角度
            self.arm_realangles[self.arm_index] += step_radians
            self.clip_arm_angles()
            arm_publish_callback({"positions": self.arm_realangles})
            print("position = " + str(self.arm_realangles[self.arm_index]))
        elif button == self.armAngleMinus_button:   # X：減少當前關節角度
            self.arm_realangles[self.arm_index] -= step_radians
            self.clip_arm_angles()
            arm_publish_callback({"positions": self.arm_realangles})
            print("position = " + str(self.arm_realangles[self.arm_index]))
        elif button == self.previousArm_button:   # Y：上一個關節
            self.arm_index = max(self.arm_index - 1, 0)
        elif button == self.nextArm_button:   # A：下一個關節
            self.arm_index = min(self.arm_index + 1, self.arm_joints_count - 1)
        elif button == self.armAngleStepDegPlus_button:   # R3：增加角度步進值
            self.angle_step_deg += self.angle_step_deg_change
            self.angle_step_deg = angle_limit(self.angle_step_deg)
        elif button == self.armAngleStepDegMinus_button:   # L3：減少角度步進值
            self.angle_step_deg -= self.angle_step_deg_change
            self.angle_step_deg = angle_limit(self.angle_step_deg)

        if self.isUnity:
            for i in range(len(self.arm_realangles)):
                self.arm_angles[i] = self.arm_realangles[i]
                self.arm_angles[i] -= math.radians(self.arm_angles_Unity_offset[i])
        else:
            for i in range(len(self.arm_realangles)):
                self.arm_angles[i] = self.arm_realangles[i]
        time.sleep(0.01)

    def process_axis_motion(self, axis, value, wheel_publish_callback):
        if axis in [2, 5]:
            mapped_value = map_trigger_value(value)
            # 如需要，可根據 mapped_value 更新 self.velocity
            # self.velocity = mapped_value        

    def process_joystick_continous(self, joysticks, wheel_publish_callback):
        for joystick in joysticks.values():

            axis_vertical = 0
            axis_horizontal = 0
            axis_rotational = 0

            #get left stick horizontal axis
            if abs(joystick.get_axis(self.left_stick_horizontal)) > self.min_joystick_value:
                axis_horizontal = joystick.get_axis(self.left_stick_horizontal)
            #get left stick vertical axis
            if abs(joystick.get_axis(self.left_stick_vertical)) > self.min_joystick_value:
                axis_vertical = -joystick.get_axis(self.left_stick_vertical)
            #get right stick horizontal axis
            if abs(joystick.get_axis(self.right_stick_horizontal)) > self.min_joystick_value:
                axis_rotational = joystick.get_axis(self.right_stick_horizontal)

            frontLeft = axis_vertical + axis_horizontal + axis_rotational
            frontRight = axis_vertical - axis_horizontal - axis_rotational        
            rearLeft = axis_vertical - axis_horizontal + axis_rotational
            rearRight = axis_vertical + axis_horizontal - axis_rotational

            finalWheelSpeed = [frontLeft * self.velocity, frontRight * self.velocity, rearLeft * self.velocity, rearRight * self.velocity]
            wheel_publish_callback(finalWheelSpeed)
            self.wheel_speed = finalWheelSpeed


    def get_joystick(self):
        return self.joystick
