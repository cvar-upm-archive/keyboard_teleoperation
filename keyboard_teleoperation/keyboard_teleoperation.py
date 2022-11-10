"""Keyboard Teleoperation"""

import sys
import threading
import rclpy

import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface
import motion_reference_handlers.utils as mh_utils


def main():
    """entrypoint"""

    drone_id, is_verbose, use_sim_time = sys.argv[1:]
    is_verbose = is_verbose.lower() == 'true'
    use_sim_time = use_sim_time.lower() == 'true'

    rclpy.init()

    uav = DroneInterface(drone_id, verbose=is_verbose,
                         use_sim_time=use_sim_time)
    kt = keyboardTeleoperation(uav, False)
    while kt.execute_window(kt.window):
        pass

    rclpy.shutdown()


class keyboardTeleoperation:
    """Keyborad Teleoperation"""

    def __init__(self, drone_interface: DroneInterface, thread=False):
        self.uav = drone_interface
        self.drone_id = self.uav.drone_id

        self.control_mode = "-SPEED-"
        self.value_list = [1.0, 1.0, 0.10, 1.0, 1.0, 0.10, 0.20, 0.20, 0.50]
        self.localization_opened = False

        self.pose_frame_id = mh_utils.get_tf_name(self.uav, 'odom')
        self.twist_frame_id = mh_utils.get_tf_name(self.uav, 'odom')

        if thread:
            self.t = threading.Thread(target=self.tick_window, daemon=True)
            self.t.start()
        else:
            self.window = self.make_window()

    def execute_window(self, window):

        font = ("Terminus Font", 14)
        font_menu = ("Ubuntu Mono", 18, 'bold')
        event, values = window.read(timeout=50)

        if event == sg.WIN_CLOSED:
            if (self.localization_opened):
                self.localization_window.close()
            return False

        if event == "Localization":
            self.localization_window = sg.Window("Localization", use_default_focus=False, size=(330, 200), 
                layout=[[sg.Text("Position", font=font_menu)],
                [sg.Text("x:", font=font), sg.Text("{:0.2f}".format(round(self.uav.position[0], 2)), font=font, key="-LOCALIZATION_X-"), sg.Text(",", font=font),
                sg.Text("y:", font=font), sg.Text("{:0.2f}".format(round(
                    self.uav.position[1], 2)), font=font, key="-LOCALIZATION_Y-"), sg.Text(",", font=font),
                sg.Text("z:", font=font), sg.Text("{:0.2f}".format(round(self.uav.position[2], 2)), font=font, key="-LOCALIZATION_Z-")],
                [sg.Text(
                    "Orientation", font=font_menu)],
                [sg.Text("r:", font=font), sg.Text("{:0.2f}".format(round(self.uav.orientation[0], 2)), font=font, key="-LOCALIZATION_R-"), sg.Text(",", font=font),
                sg.Text("p:", font=font), sg.Text("{:0.2f}".format(round(
                    self.uav.orientation[1], 2)), font=font, key="-LOCALIZATION_P-"), sg.Text(",", font=font),
                sg.Text("y:", font=font), sg.Text("{:0.2f}".format(round(self.uav.orientation[2], 2)), font=font, key="-LOCALIZATION_YW-")],
                [sg.Button("Exit", font=font, pad=((240, 0), (20, 0)))]])

            self.localization_opened = True

        if event == "Settings":
            settings_window = sg.Window("Settings", use_default_focus=False,
                layout=[[sg.Text("Speed Control Values", font=font_menu)],
                [sg.Text("Speed value:", font=font), sg.InputText(str(
                    self.value_list[0]), font=font, key="-VALUE0-", size=(5, 3), background_color="white"), sg.Text("m/s", font=font)],
                [sg.Text("Vertical value:", font=font), sg.InputText(str(
                    self.value_list[1]), font=font, key="-VALUE1-", size=(5, 3), background_color="white"), sg.Text("m/s", font=font)],
                [sg.Text("Turn speed value:", font=font), sg.InputText(str(
                    self.value_list[2]), font=font, key="-VALUE2-", size=(5, 3), background_color="white"), sg.Text("rad/s", font=font)],
                [sg.Text(
                    "", font=font)],
                [sg.Text(
                    "Position control values:", font=font_menu)],
                [sg.Text("Position value:", font=font), sg.InputText(str(
                    self.value_list[3]), font=font, key="-VALUE3-", size=(5, 3), background_color="white"), sg.Text("m", font=font)],
                [sg.Text("Altitude value:", font=font), sg.InputText(str(
                    self.value_list[4]), font=font, key="-VALUE4-", size=(5, 3), background_color="white"), sg.Text("m", font=font)],
                [sg.Text("Turn angle value:", font=font), sg.InputText(str(self.value_list[5]), font=font,
                                                                    key="-VALUE5-", size=(5, 3), background_color="white"), sg.Text("rad", font=font)],
                [sg.Text(
                    "", font=font)],
                [sg.Text(
                    "Attitude control values:", font=font_menu)],
                [sg.Text("Pitch angle value:", font=font), sg.InputText(str(self.value_list[6]), font=font,
                                                                        key="-VALUE6-", size=(5, 3), background_color="white"), sg.Text("rad", font=font)],
                [sg.Text("Roll angle value:", font=font), sg.InputText(str(self.value_list[7]), font=font,
                                                                    key="-VALUE7-", size=(5, 3), background_color="white"), sg.Text("rad", font=font)],
                [sg.Text("Attitude duration:", font=font), sg.InputText(str(self.value_list[8]), font=font,
                                                                        key="-VALUE8-", size=(5, 3), background_color="white"), sg.Text("s", font=font)],
                [sg.Text(
                    "", font=font)],
                [sg.Button("Save", font=font), sg.Button("Exit", font=font, pad=((150, 0), (0, 0)))]])

            while (True):
                e, v = settings_window.read()

                if e == sg.WIN_CLOSED or e == "Exit":
                    settings_window.close()
                    break

                for idx, value in enumerate(list(v.values())):
                    try:
                        self.value_list[idx] = float(value)
                    except ValueError:
                        print("Invalid Input, setting to 0.0")
                        self.value_list[idx] = 0.0
                        settings_window["-VALUE" +
                                        str(idx) + "-"].update(value="{:0.2f}".format(0.00))

                if e == "Save":
                    jdx = 0
                    for idx, value in enumerate(self.value_list):
                        window["-INPUTTEXT" +
                               str(jdx+1) + "-"].update(value="{:0.2f}".format(value))
                        window["-INPUTTEXT" +
                               str(jdx+2) + "-"].update(value="{:0.2f}".format(value))
                        if ((idx != 1) and (idx != 2) and (idx != 4) and (idx != 5) and (idx != 6) and (idx != 7)):

                            window["-INPUTTEXT" +
                                   str(jdx+3) + "-"].update(value="{:0.2f}".format(value))
                            window["-INPUTTEXT" +
                                   str(jdx+4) + "-"].update(value="{:0.2f}".format(value))
                            jdx = jdx + 4
                        else:
                            jdx = jdx + 2

        if event == "+TEXT FOCUS OUT+":
            if self.control_mode == "-SPEED-":
                window["-SPEED-"].set_focus(True)
            elif self.control_mode == "-POSE-":
                window["-POSE-"].set_focus(True)
            elif self.control_mode == "-ATTITUDE-":
                window["-ATTITUDE-"].set_focus(True)

        elif event == "-SPEED-":
            self.control_mode = event
            window["-SPEED-"].set_focus(True)
            window["-HEADER_SPEED-"].update(visible=True)
            window["-HEADER_POSE-"].update(visible=False)
            window["-HEADER_ATTITUDE-"].update(visible=False)
            window["-SP_CONTROL-"].update(visible=True)
            window["-POS_CONTROL-"].update(visible=False)
            window["-AT_CONTROL-"].update(visible=False)
            window["-P_CONTROL-"].update(visible=False)
            window["-COL5-"].update(visible=False)
            window["-COL4-"].update(visible=True)
            window["-COL7-"].update(visible=False)
            window["-COL6B-"].update(visible=True)
            window["-COL6-"].update(visible=False)

        elif event == "-POSE-":
            self.control_mode = event
            window["-POSE-"].set_focus(True)
            window["-HEADER_SPEED-"].update(visible=False)
            window["-HEADER_POSE-"].update(visible=True)
            window["-HEADER_ATTITUDE-"].update(visible=False)
            window["-SP_CONTROL-"].update(visible=False)
            window["-POS_CONTROL-"].update(visible=True)
            window["-AT_CONTROL-"].update(visible=False)
            window["-COL5-"].update(visible=False)
            window["-COL4-"].update(visible=False)
            window["-P_CONTROL-"].update(visible=False)
            window["-COL7-"].update(visible=True)
            window["-COL6B-"].update(visible=False)
            window["-COL6-"].update(visible=True)

        elif event == "-ATTITUDE-":
            self.control_mode = event
            window["-ATTITUDE-"].set_focus(True)
            window["-HEADER_SPEED-"].update(visible=False)
            window["-HEADER_POSE-"].update(visible=False)
            window["-HEADER_ATTITUDE-"].update(visible=True)
            window["-SP_CONTROL-"].update(visible=False)
            window["-POS_CONTROL-"].update(visible=False)
            window["-AT_CONTROL-"].update(visible=True)
            window["-P_CONTROL-"].update(visible=True)
            window["-COL5-"].update(visible=True)
            window["-COL4-"].update(visible=False)
            window["-COL7-"].update(visible=False)
            window["-COL6B-"].update(visible=False)
            window["-COL6-"].update(visible=True)
        else:
            input = event.split(":")
            if (input[0] == "t"):
                window["-key_pressed-"].update(value=input[0])
                try:
                    threading.Thread(target=self.take_off, daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            elif (input[0] == "y"):
                window["-key_pressed-"].update(value=input[0])
                try:
                    threading.Thread(target=self.land, daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            elif (input[0] == "h"):
                window["-key_pressed-"].update(value=input[0])
                try:
                    threading.Thread(target=self.hover, daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            if (self.control_mode == "-SPEED-"):
                if (input[0] == "Up"):
                    window["-key_pressed-"].update(value="↑")
                    lineal = [self.value_list[0], 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "Down"):
                    window["-key_pressed-"].update(value="↓")
                    lineal = [-self.value_list[0], 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "Left"):
                    window["-key_pressed-"].update(value="←")
                    lineal = [0.0, self.value_list[0], 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "Right"):
                    window["-key_pressed-"].update(value="→")
                    lineal = [0.0, -self.value_list[0], 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                if (input[0] == "w"):
                    window["-key_pressed-"].update(value=input[0])
                    lineal = [0.0, 0.0, self.value_list[1]]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "s"):
                    window["-key_pressed-"].update(value=input[0])
                    lineal = [0.0, 0.0, -self.value_list[1]]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "a"):
                    window["-key_pressed-"].update(value=input[0])
                    lineal = [0.0, 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, self.value_list[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "d"):
                    window["-key_pressed-"].update(value=input[0])
                    lineal = [0.0, 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            lineal, -self.value_list[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

            elif (self.control_mode == "-POSE-"):

                if (input[0] == "Up"):
                    window["-key_pressed-"].update(value="↑")
                    position = [self.uav.position[0] + self.value_list[3],
                                self.uav.position[1], self.uav.position[2]]
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, self.uav.orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "Down"):
                    window["-key_pressed-"].update(value="↓")
                    position = [self.uav.position[0] - self.value_list[3],
                                self.uav.position[1], self.uav.position[2]]
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, self.uav.orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "Left"):
                    window["-key_pressed-"].update(value="←")
                    position = [self.uav.position[0], self.uav.position[1] +
                                self.value_list[3], self.uav.position[2]]
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, self.uav.orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "Right"):
                    window["-key_pressed-"].update(value="→")
                    position = [self.uav.position[0], self.uav.position[1] -
                                self.value_list[3], self.uav.position[2]]
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, self.uav.orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                if (input[0] == "w"):
                    window["-key_pressed-"].update(value=input[0])
                    position = [self.uav.position[0], self.uav.position[1],
                                self.uav.position[2] + self.value_list[4]]
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, self.uav.orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "s"):
                    window["-key_pressed-"].update(value=input[0])
                    position = [self.uav.position[0], self.uav.position[1],
                                self.uav.position[2] - self.value_list[4]]
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, self.uav.orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "a"):
                    window["-key_pressed-"].update(value=input[0])
                    position = [self.uav.position[0],
                                self.uav.position[1], self.uav.position[2]]
                    euler = self.uav.orientation
                    yaw = euler[2] + self.value_list[5]
                    #orientation = [self.uav.orientation.x, self.uav.orientation.y, self.uav.orientation.z, self.uav.orientation.w]

                    #orientation = quaternion_from_euler(euler[0], euler[1], euler[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, yaw,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

                elif (input[0] == "d"):
                    window["-key_pressed-"].update(value=input[0])
                    position = [self.uav.position[0],
                                self.uav.position[1], self.uav.position[2]]
                    euler = self.uav.orientation
                    yaw = euler[2] - self.value_list[5]
                    #orientation = [self.uav.orientation.x, self.uav.orientation.y, self.uav.orientation.z, self.uav.orientation.w]

                    #orientation = quaternion_from_euler(euler[0], euler[1], euler[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            position, yaw,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

            if input[0] == "Delete":
                window["-key_pressed-"].update(value=input[0])
                try:
                    threading.Thread(target=self.emergency_stop, daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            if (self.localization_opened):
                self.execute_localization_window(self.localization_window)

        return True

    def execute_localization_window(self, window):
        e, v = window.read(timeout=1)

        window["-LOCALIZATION_X-"].update(
            value="{:0.2f}".format(round(self.uav.position[0], 2)))
        window["-LOCALIZATION_Y-"].update(
            value="{:0.2f}".format(round(self.uav.position[1], 2)))
        window["-LOCALIZATION_Z-"].update(
            value="{:0.2f}".format(round(self.uav.position[2], 2)))

        window["-LOCALIZATION_R-"].update(
            value="{:0.2f}".format(round(self.uav.orientation[0], 2)))
        window["-LOCALIZATION_P-"].update(
            value="{:0.2f}".format(round(self.uav.orientation[1], 2)))
        window["-LOCALIZATION_YW-"].update(
            value="{:0.2f}".format(round(self.uav.orientation[2], 2)))

        # localization_window.refresh()
        if e == sg.WIN_CLOSED or e == "Exit":
            self.localization_opened = False
            window.close()

    def make_window(self):
        sg.theme("DarkBlack1")
        font = ("Terminus Font", 14)
        font_menu = ("Ubuntu Mono", 18, 'bold')
        col1_layout = [
            [sg.Text("t", font=font)],
            [sg.Text("y", font=font)],
            [sg.Text("h", font=font)],
            [sg.Text("del", font=font)],
            [sg.Text("r", font=font)]]
        col2_layout = [
            [sg.Text("Take off", font=font)],
            [sg.Text("Land", font=font)],
            [sg.Text("Hover", font=font)],
            [sg.Text("Emergency Stop", font=font)],
            [sg.Text("Reset orientation", font=font)]]

        col3_layout = [
            [sg.Text("↑", font=font)],
            [sg.Text("↓", font=font)],
            [sg.Text("←", font=font)],
            [sg.Text("→", font=font)],
            [sg.Text("", font=font)]]
        col4_layout = [
            [sg.Text("Increase forward speed", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT1-"), sg.Text("m/s", font=font)],
            [sg.Text("Increase backward speed", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT2-"), sg.Text("m/s", font=font)],
            [sg.Text("Increase speed to the right", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT3-"), sg.Text("m/s", font=font)],
            [sg.Text("Increase speed to the left", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT4-"), sg.Text("m/s", font=font)],
            [sg.Text("", font=font)]
        ]

        col5_layout = [
            [sg.Text("Pitch", font=font), sg.Text("0.20", font=font, key="-INPUTTEXT17-"), sg.Text(
                "rad during", font=font), sg.Text("0.50", font=font, key="-INPUTTEXT21-"), sg.Text("s", font=font)],
            [sg.Text("Pitch -", font=font), sg.Text("0.20", font=font, key="-INPUTTEXT18-"), sg.Text(
                "rad during", font=font), sg.Text("0.50", font=font, key="-INPUTTEXT22-"), sg.Text("s", font=font)],
            [sg.Text("Roll", font=font), sg.Text("0.20", font=font, key="-INPUTTEXT19-"), sg.Text(
                "rad during", font=font), sg.Text("0.50", font=font, key="-INPUTTEXT23-"), sg.Text("s", font=font)],
            [sg.Text("Roll -", font=font), sg.Text("0.20", font=font, key="-INPUTTEXT20-"), sg.Text(
                "rad during", font=font), sg.Text("0.50", font=font, key="-INPUTTEXT24-"), sg.Text("s", font=font)],
            [sg.Text("", font=font)]
        ]

        col6_layout = [
            [sg.Text("Increase altitude", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT13-"), sg.Text("m", font=font)],
            [sg.Text("Decrease altitude", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT14-"), sg.Text("m", font=font)],
            [sg.Text("Turn counter-clockwise", font=font), sg.Text("0.10",
                                                                   font=font, key="-INPUTTEXT15-"), sg.Text("rad", font=font)],
            [sg.Text("Turn clockwise", font=font), sg.Text(
                "0.10", font=font, key="-INPUTTEXT16-"), sg.Text("rad", font=font)]
        ]

        col6B_layout = [
            [sg.Text("Increase vertical speed", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT5-"), sg.Text("m/s", font=font)],
            [sg.Text("Decrease vertical speed", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT6-"), sg.Text("m/s", font=font)],
            [sg.Text("Turn speed counter-clockwise", font=font), sg.Text(
                "0.10", font=font, key="-INPUTTEXT7-"), sg.Text("m/s", font=font)],
            [sg.Text("Turn speed clockwise", font=font), sg.Text(
                "0.10", font=font, key="-INPUTTEXT8-"), sg.Text("rad/s", font=font)]
        ]

        col7_layout = [
            [sg.Text("Increase forward position", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT9-"), sg.Text("m", font=font)],
            [sg.Text("Increase backward position", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT10-"), sg.Text("m", font=font)],
            [sg.Text("Increase position to the right", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT11-"), sg.Text("m", font=font)],
            [sg.Text("Increase position to the left", font=font), sg.Text(
                "1.00", font=font, key="-INPUTTEXT12-"), sg.Text("m", font=font)],
            [sg.Text("", font=font)]
        ]

        col8_layout = [
            [sg.Text("w", font=font)],
            [sg.Text("s", font=font)],
            [sg.Text("a", font=font)],
            [sg.Text("d", font=font)]
        ]

        col_button_layout = [
            [sg.Button("Speed mode", font=font, key="-SPEED-", focus=True)],
            [sg.Button("Pose mode", font=font, key="-POSE-")],
            [sg.Button("Attitude mode", font=font, key="-ATTITUDE-")]
        ]

        self.layout = [[sg.Button("Settings", font=font_menu), sg.Text("|", font=font_menu), sg.Button("Localization", font=font_menu), sg.Text("|", font=font_menu), sg.Text("Teleoperation mode: Speed mode", justification="left", font=font_menu, key="-HEADER_SPEED-", visible=True, pad=((78, 0), (0, 0))),
                        sg.Text("Teleoperation mode: Pose mode", justification="left",
                                font=font_menu, key="-HEADER_POSE-", visible=False, pad=((78, 0), (0, 0))),
                        sg.Text("Teleoperation mode: Attitude mode", justification="left", font=font_menu, key="-HEADER_ATTITUDE-", visible=False, pad=((78, 0), (0, 0)))],
                       [sg.HSeparator(pad=(0, 10))],
                       [sg.Text("BASIC MOTIONS", pad=((10, 280), (10, 0)), font=font_menu), sg.Text("SPEED CONTROL", pad=((0, 0), (10, 0)), font=font_menu, key="-SP_CONTROL-"), sg.Text("ATTITUDE CONTROL",
                                                                                                                                                                                         pad=((0, 0), (10, 0)), font=font_menu, visible=False, key="-AT_CONTROL-"), sg.Text("POSE CONTROL", pad=((0, 0), (10, 0)), font=font_menu, visible=False, key="-POS_CONTROL-")],
                       [sg.Column(col1_layout, element_justification='left'), sg.Column(col2_layout, element_justification='left', pad=((0, 215), (0, 0))),
                        sg.Column(col3_layout, element_justification='left', justification="left"), sg.Column(col4_layout, element_justification='left', justification="left", key="-COL4-"), sg.Column(col5_layout, element_justification='left', visible=False, key="-COL5-"), sg.Column(col7_layout, element_justification='left', visible=False, key="-COL7-")],
                       [sg.Text("TELEOPERATION MODE SELECTION", pad=((10, 100), (10, 0)), font=font_menu), sg.Text(
                           "POSE CONTROL", pad=((0, 0), (10, 0)), font=font_menu, key="-P_CONTROL-", visible=False)],
                       [sg.Column(col_button_layout, element_justification='left', pad=((0, 270), (0, 0))), sg.Column(
                           col8_layout, element_justification='left', key="-COL8-"), sg.Column(col6_layout, element_justification='left', key="-COL6-", visible=False), sg.Column(col6B_layout, element_justification='left', key="-COL6B-")],
                       [sg.HSeparator(pad=(0, 10))],
                       [sg.Text("Last key pressed:", font=font_menu), sg.Text("", font=font_menu, key="-key_pressed-")]]

        return sg.Window("Keyboard Teleoperation", self.layout, return_keyboard_events=True, use_default_focus=False, resizable=True, finalize=True)

    def tick_window(self):
        window = self.make_window()
        while self.execute_window(window):
            pass

    def shutdown(self):
        self.t.join()

    def take_off(self):
        self.uav.arm()
        self.uav.offboard()
        self.uav.takeoff(1.0, 1.0)

    def land(self):
        self.uav.land(0.5)

    def hover(self):
        self.uav.hover_motion_handler.send_hover()

    def move_at_speed(self, lineal, yaw_speed):
        self.uav.speed_motion_handler.send_speed_command_with_yaw_speed(
            lineal, self.twist_frame_id, yaw_speed)

    def go_to_pose(self, position, orientation):
        self.uav.position_motion_handler.send_position_command_with_yaw_angle(
            position, None, self.pose_frame_id, self.twist_frame_id, orientation)

    def emergency_stop(self):
        self.uav.emergency_stop()


if __name__ == '__main__':
    main()
