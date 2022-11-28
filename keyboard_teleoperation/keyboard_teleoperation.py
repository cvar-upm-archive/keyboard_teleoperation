"""Keyboard Teleoperation"""

import sys
import rclpy
import threading

import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface
import motion_reference_handlers.utils as mh_utils
from keyboard_interface import KeyboardInterface
from drone_manager import DroneManager


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
        self.value_list = [1.0, 1.0, 0.10, 1.0, 1.0, 0.10, 0.20, 0.20, 0.50] #TODO: MOVE AND TAG VALUES INTO DRONE_MANAGER AS CLASS PROPERTIES
        self.localization_opened = False

        self.kb_interface = KeyboardInterface(
            theme="DarkBlack1", font= ("Terminus Font", 14), menu_font = ("Ubuntu Mono", 18, 'bold'))
        
        self.drone_manager = DroneManager(uav_list=self.uav_list, drone_id_list=self.drone_id_list, pose_frame_id='earth', twist_frame_id='earth')

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
            if input[0] in {"t","l","space","Delete","w","s","a","d"}:
                window["-key_pressed-"].update(value=input[0])
            elif (input[0] == "Up"):
                window["-key_pressed-"].update(value="↑")
            elif (input[0] == "Down"):
                window["-key_pressed-"].update(value="↓")
            elif (input[0] == "Left"):
                window["-key_pressed-"].update(value="←")
            elif (input[0] == "Right"):
                window["-key_pressed-"].update(value="→")
            
            self.drone_manager.manage_common_behaviors(input[0])

            if (self.control_mode == "-SPEED-"):
                self.drone_manager.manage_speed_behaviors(input[0], self.value_list)

            elif (self.control_mode == "-POSE-"):
                self.drone_manager.manage_pose_behaviors(input[0], self.value_list)

            if (self.localization_opened):
                self.execute_localization_window(self.localization_window)

    def update_main_window_mode(self, window: sg.Window, event):

        if event == "-SPEED-":
            self.control_mode = event
            self.update_window_to_speed(window)

        elif event == "-POSE-":
            self.control_mode = event
            self.update_window_to_pose(window)

        elif event == "-ATTITUDE-":
            self.control_mode = event
            self.update_window_to_attitude(window)

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

    def update_window_to_pose(self, window: sg.Window):

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


    def update_window_to_speed(self, window: sg.Window):

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


    def update_window_to_attitude(self, window: sg.Window):

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

if __name__ == '__main__':
    main()
