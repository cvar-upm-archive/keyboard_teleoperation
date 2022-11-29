"""Keyboard Teleoperation"""

import sys
import rclpy
import threading
from enum import Enum

import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface
import motion_reference_handlers.utils as mh_utils
from main_window import MainWindow
from localization_window import LocalizationWindow
from settings_window import SettingsWindow
from drone_manager import DroneManager
from config_values import ControlValues
from config_values import ControlModes

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

    kt = KeyboardTeleoperation(uav_list, False)
    while kt.execute_main_window(kt.main_window):
        pass

    rclpy.shutdown()


class KeyboardTeleoperation:
    """Keyborad Teleoperation"""

    def __init__(self, list_drone_interface: list[DroneInterface], thread=False):
        self.uav_list = list_drone_interface
        drone_id_list = list()
        
        for uav in self.uav_list: drone_id_list.append([uav.get_namespace(), True])
        
        value_list = [ControlValues.SPEED_VALUE.value, ControlValues.VERTICAL_VALUE.value, ControlValues.TURN_SPEED_VALUE.value, ControlValues.POSITION_VALUE.value, ControlValues.ALTITUDE_VALUE.value,
         ControlValues.TURN_ANGLE_VALUE.value, ControlValues.PITCH_ANGLE_VALUE.value, ControlValues.ROLL_ANGLE_VALUE.value, ControlValues.ATTITUDE_DURATION.value] 

        self.localization_opened = False
        
        sg.theme("DarkBlack1")

        self.drone_manager = DroneManager(uav_list=self.uav_list, drone_id_list=drone_id_list, pose_frame_id='earth', twist_frame_id='earth')

        self.settings_window = SettingsWindow(font=("Terminus Font", 14), menu_font=("Ubuntu Mono", 18, 'bold'), value_list=value_list, title="Settings", enable_close_attempted_event=True)

        self.localization_window = LocalizationWindow(font=("Terminus Font", 14), menu_font=("Ubuntu Mono", 18, 'bold'), uav_list=self.uav_list, title="Localization",
         size=(330, 200), use_default_focus=False, enable_close_attempted_event=True)

        self.main_window = MainWindow(settings_window = self.settings_window, localization_window = self.localization_window, font=("Terminus Font", 14), menu_font=("Ubuntu Mono", 18, 'bold'),
         drone_id_list=drone_id_list, value_list=value_list, title="Keyboard Teleoperation", finalize=True, return_keyboard_events=True)

        if thread:
            self.t = threading.Thread(target=self.tick_main_window, daemon=True)
            self.t.start()
        else:
            self.main_window.make_main_window()
            

    def tick_main_window(self):
        self.window = self.main_window.make_main_window()
        while self.execute_main_window(self.window):
            pass

    def execute_main_window(self, window: MainWindow):
        event, values = window.read(timeout=50)  # type: ignore
        control_mode, key, value_list, opened = self.main_window.event_handler(event, values)

        if (control_mode is not None):

            self.drone_manager.manage_common_behaviors(key)

            if (control_mode == ControlModes.SPEED_CONTROL.value):
                self.drone_manager.manage_speed_behaviors(key, value_list)

            elif (control_mode == ControlModes.ATTITUDE_CONTROL.value):
                self.drone_manager.manage_pose_behaviors(key, value_list)

        return opened

if __name__ == '__main__':
    main()
