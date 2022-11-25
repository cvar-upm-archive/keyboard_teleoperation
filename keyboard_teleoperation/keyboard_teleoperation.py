"""Keyboard Teleoperation"""

import sys
import threading
import rclpy

import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface
import motion_reference_handlers.utils as mh_utils
from keyboard_interface import KeyboardInterface


def main():
    """entrypoint"""

    drone_id, is_verbose, use_sim_time = sys.argv[1:]
    is_verbose = is_verbose.lower() == 'true'
    use_sim_time = use_sim_time.lower() == 'true'
    uav_list = list()
    rclpy.init()
    if ',' in drone_id:
        drone_id_list = drone_id.split(",")
        for id in drone_id_list:  uav_list.append(DroneInterface(id, verbose=is_verbose, use_sim_time=use_sim_time)) 
    else:
        uav_list.append(DroneInterface(drone_id, verbose=is_verbose, use_sim_time=use_sim_time))

    kt = keyboardTeleoperation(uav_list, False)
    while kt.execute_main_window(kt.window):
        pass

    rclpy.shutdown()


class keyboardTeleoperation:
    """Keyborad Teleoperation"""

    def __init__(self, list_drone_interface: list[DroneInterface], thread=False):
        self.uav_list = list_drone_interface
        self.drone_id_list = list()
        
        for uav in self.uav_list: self.drone_id_list.append([uav.get_namespace(), True])
        
        self.control_mode = "-SPEED-"
        self.value_list = [1.0, 1.0, 0.10, 1.0, 1.0, 0.10, 0.20, 0.20, 0.50]
        self.localization_opened = False

        self.pose_frame_id = 'earth'
        self.twist_frame_id = 'earth'

        self.kb_interface = KeyboardInterface(
            theme="DarkBlack1", font= ("Terminus Font", 14), menu_font = ("Ubuntu Mono", 18, 'bold'))

        if thread:
            self.t = threading.Thread(target=self.tick_main_window, daemon=True)
            self.t.start()
        else:
            self.window = self.kb_interface.make_main_window(self.drone_id_list)

    def tick_main_window(self):
        self.window = self.kb_interface.make_main_window(self.drone_id_list)
        while self.execute_main_window(self.window):
            pass

    def execute_main_window(self, window: sg.Window):

        event, values = window.read(timeout=50)  # type: ignore

        if event == sg.WIN_CLOSED:
            if (self.localization_opened):
                self.localization_window.close()
            return False

        self.manage_main_window_event(window, event, values)

        return True

    def manage_main_window_event(self, window: sg.Window, event, value):
        selection_values = list(value.values())
        #self.uav_list[0].get_logger().info(str(selection_values))
        if event == "Localization":
            if (not self.localization_opened):
                self.localization_window = self.kb_interface.make_localization_window(location=window.current_location(), uav_list=self.uav_list)

            self.localization_opened = True

        elif event == "Settings":
            settings_window = self.kb_interface.make_settings_window(location=window.current_location(), value_list=self.value_list)

            while (True):
                settings_event, settings_value = settings_window.read()  # type: ignore

                if settings_event == sg.WIN_CLOSED or settings_event == "Exit":
                    settings_window.close()
                    break

                self.manage_settings_event(window, settings_window, settings_event, settings_value)

        elif event == "All":
            
            if (list(value.values())[-1]):
                for index, value in enumerate(selection_values[:-1]):
                    window[self.drone_id_list[index][0]].update(True)
            else:
                for index, value in enumerate(selection_values[:-1]):
                    window[self.drone_id_list[index][0]].update(False)

            for drone_id in self.drone_id_list:
                 drone_id[1] = True

        elif event in [x for l in self.drone_id_list for x in l]:

            if all(selection_values[:-1]):
                window["All"].update(True)
            else:
                window["All"].update(False)

            for index, value in enumerate(selection_values[:-1]):
                self.drone_id_list[index][1] = bool(value)

        elif event in ["-SPEED-", "-POSE-", "-ATTITUDE-"]:
            self.update_main_window_mode(window, event)

        else:
            input = event.split(":")
            self.manage_common_behaviors(window, input)

            if (self.control_mode == "-SPEED-"):
                self.manage_speed_behaviors(window, input)

            elif (self.control_mode == "-POSE-"):
                self.manage_pose_behaviors(window, input)

            if (self.localization_opened):
                self.execute_localization_window(self.localization_window)

    def update_main_window_mode(self, window: sg.Window, event):

        if event == "-SPEED-":
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


    def manage_settings_event(self, window: sg.Window, settings_window: sg.Window, settings_event, settings_value):
        numeric_values = list(settings_value.values())[0:len(self.value_list)]
        selection_values = list(settings_value.values())[len(self.value_list):]
        for idx, value in enumerate(list(numeric_values)):
            try:
                self.value_list[idx] = float(value)
            except ValueError:
                print("Invalid Input, setting to 0.0")
                self.value_list[idx] = 0.0
                settings_window["-VALUE" +
                                str(idx) + "-"].update(value="{:0.2f}".format(0.00))

        if settings_event == "Save":
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

                settings_window["-VALUE" +
                                str(idx) + "-"].update(value="{:0.2f}".format(value))

    def execute_localization_window(self, localization_window: sg.Window):
        localization_event, _ = localization_window.read(timeout=1)  # type: ignore

        localization_window["-LOCALIZATION_X-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].position[0], 2)))
        localization_window["-LOCALIZATION_Y-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].position[1], 2))) 
        localization_window["-LOCALIZATION_Z-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].position[2], 2)))

        localization_window["-LOCALIZATION_R-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].orientation[0], 2)))
        localization_window["-LOCALIZATION_P-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].orientation[1], 2)))
        localization_window["-LOCALIZATION_YW-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].orientation[2], 2)))

        # localization_window.refresh()
        if localization_event == sg.WIN_CLOSED or localization_event == "Exit":
            self.localization_opened = False
            localization_window.close()

    # FUNCTIONS THAT MANAGE THE CALLS TO DRONE INTERFACES FUNCTIONS GIVEN A KB INPUT

    def manage_common_behaviors(self, window: sg.Window, input):

        if (input[0] == "t"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                #self.uav_list[index].get_logger().info(str(drone_id[index][1]))
                if drone_id[1] == True:
                    
                    try:
                        threading.Thread(target=self.take_off, args=(
                                self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "l"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    try:
                        threading.Thread(target=self.land, args=(
                                self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "space"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    try:
                        threading.Thread(target=self.hover, args=(
                                self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif input[0] == "Delete":
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    try:
                        threading.Thread(target=self.emergency_stop, args=(
                            self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

    def manage_speed_behaviors(self, window: sg.Window, input):

        if (input[0] == "Up"):
            window["-key_pressed-"].update(value="↑")
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [self.value_list[0], 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "Down"):
            window["-key_pressed-"].update(value="↓")
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [-self.value_list[0], 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "Left"):
            window["-key_pressed-"].update(value="←")
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, self.value_list[0], 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "Right"):
            window["-key_pressed-"].update(value="→")
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, -self.value_list[0], 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "w"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, self.value_list[1]]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "s"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, -self.value_list[1]]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "a"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, self.value_list[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "d"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, -self.value_list[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

    def manage_pose_behaviors(self, window: sg.Window, input):

        if (input[0] == "Up"):
            window["-key_pressed-"].update(value="↑")
            for index, drone_id in enumerate(self.drone_id_list):
                
                if drone_id[1] == True:

                    position = [self.uav_list[index].position[0] + self.value_list[3],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
                    
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "Down"):
            window["-key_pressed-"].update(value="↓")
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0] - self.value_list[3],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
            #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "Left"):
            window["-key_pressed-"].update(value="←")
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1] +
                                self.value_list[3], self.uav_list[index].position[2]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "Right"):
            window["-key_pressed-"].update(value="→")
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1] -
                                self.value_list[3], self.uav_list[index].position[2]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        if (input[0] == "w"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1],
                                self.uav_list[index].position[2] + self.value_list[4]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "s"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1],
                                self.uav_list[index].position[2] - self.value_list[4]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "a"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
                    euler = self.uav_list[index].orientation
                    #orientation = [self.uav_list[index].orientation.x, self.uav_list[index].orientation.y, self.uav_list[index].orientation.z, self.uav_list[index].orientation.w]
                    yaw = euler[2] + self.value_list[5]
                    #orientation = quaternion_from_euler(euler[0], euler[1], euler[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, yaw,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input[0] == "d"):
            window["-key_pressed-"].update(value=input[0])
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
                    euler = self.uav_list[index].orientation
                    yaw = euler[2] - self.value_list[5]
                    #orientation = [self.uav.orientation.x, self.uav.orientation.y, self.uav.orientation.z, self.uav.orientation.w]

                    #orientation = quaternion_from_euler(euler[0], euler[1], euler[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, yaw,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

    # FUNCTIONS TO CALL THE DRONE INTERFACES FUNCTIONS

    def shutdown(self):
        self.t.join()

    def take_off(self, uav: DroneInterface):
        uav.arm()
        uav.offboard()
        uav.takeoff(1.0, 1.0)
        
    def land(self, uav: DroneInterface):
        uav.land(0.5)

    def hover(self, uav: DroneInterface):
        uav.send_hover()

    def move_at_speed(self, uav: DroneInterface, lineal, yaw_speed):
        uav.speed_motion_handler.send_speed_command_with_yaw_speed(
            lineal, self.twist_frame_id, yaw_speed)

    def go_to_pose(self, uav: DroneInterface, position, orientation):
        uav.position_motion_handler.send_position_command_with_yaw_angle(
            position, None, self.pose_frame_id, self.twist_frame_id, orientation)
        
    def emergency_stop(self, uav: DroneInterface):
        uav.send_emergency_killswitch_to_aircraft()


if __name__ == '__main__':
    main()
