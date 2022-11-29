"""Keyboard Teleoperation"""

import sys
import rclpy
import threading

import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface
import motion_reference_handlers.utils as mh_utils
from keyboard_interface import KeyboardInterface
from drone_manager import DroneManager
from main_window import MainWindow

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
        
        value_list = [1.0, 1.0, 0.10, 1.0, 1.0, 0.10, 0.20, 0.20, 0.50] #TODO: MOVE AND TAG VALUES INTO DRONE_MANAGER AS CLASS PROPERTIES
        self.localization_opened = False
        
        sg.theme("DarkBlack1")
        drone_manager = DroneManager(uav_list=self.uav_list, drone_id_list=drone_id_list, pose_frame_id='earth', twist_frame_id='earth')

        self.main_window = MainWindow(drone_manager=drone_manager, font=("Terminus Font", 14), menu_font=("Ubuntu Mono", 18, 'bold'),
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
        self.uav_list[0].get_logger().info(event)
        return self.main_window.event_handler(event, values)

if __name__ == '__main__':
    main()
