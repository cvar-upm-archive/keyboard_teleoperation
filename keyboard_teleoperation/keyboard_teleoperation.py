#from msilib.schema import Icon
from tkinter import font
import PySimpleGUI as sg
import rclpy
import threading
from time import sleep
import rclpy.signals
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from python_interface.drone_interface import DroneInterface

def main():
    rclpy.init()
    uav = DroneInterface("drone_0", verbose=True)
    kt = keyboardTeleoperation(uav, "drone_0", False, False)
    while(kt.execute_window(kt.window)):
        kt.tick_window()

class keyboardTeleoperation(Node):
    def __init__(self, drone_interface, drone_id="drone0", verbose=False, thread = False):
        super().__init__(f'{drone_id}_teleoperation')
        self.drone_id = drone_id
        self.uav = drone_interface
        self.window = self.make_window()
        self.control_mode = "-SPEED-"
        self.value_list = [1.0, 1.0, 1.0, 0.10, 0.20, 0.20, 0.50]
        
        #self.event, self.values = self.window.read()
    
        if (thread):     
            self.t = threading.Thread(target=self.tick_window)
            self.t.start()

    def execute_window(self, window):
        
        # End program if user closes window or
        # presses the OK button
        #window.bind('<Button-1>', '+TEXT FOCUS OUT+')
        font = ("Terminus Font", 14)
        font_menu = ("Ubuntu Mono", 18, 'bold')
        window.bind('<Return>', '+TEXT FOCUS OUT+')
        #window['-INPUTTEXT1-'].bind('<Button-1>', '+TEXT FOCUS IN+')       
        event, values = window.read()

        if event == sg.WIN_CLOSED:
            return False

        """for idx, value in enumerate(values):
            self.value_list[idx] = value[]"""

        if event == "Settings":
            settings_window = sg.Window("Settings", use_default_focus=False, layout=[[sg.Text("Speed Control Values", font=font_menu)],
            [sg.Text("Speed value:", font=font), sg.InputText(str(self.value_list[0]), font= font, key="-INPUTTEXT100-", size=(5,3), background_color="white"), sg.Text("m/s", font=font)],
            [sg.Text("", font = font)],
            [sg.Text("Position control values:", font=font_menu)],
            [sg.Text("Position value:", font=font), sg.InputText(str(self.value_list[1]), font= font, key="-INPUTTEXT101-", size=(5,3), background_color="white"), sg.Text("m", font = font)],
            [sg.Text("Altitude value:", font=font), sg.InputText(str(self.value_list[2]), font= font, key="-INPUTTEXT102-", size=(5,3), background_color="white"), sg.Text("m", font = font)],
            [sg.Text("Turn angle value:", font=font), sg.InputText(str(self.value_list[3]), font= font, key="-INPUTTEXT103-", size=(5,3), background_color="white"), sg.Text("rad", font = font)],
            [sg.Text("", font = font)],
            [sg.Text("Attitude control values:", font=font_menu)],
            [sg.Text("Pitch angle value:", font=font), sg.InputText(str(self.value_list[4]), font= font, key="-INPUTTEXT104-", size=(5,3), background_color="white"), sg.Text("rad", font = font)],
            [sg.Text("Roll angle value:", font=font), sg.InputText(str(self.value_list[5]), font= font, key="-INPUTTEXT105-", size=(5,3), background_color="white"), sg.Text("rad", font = font)],
            [sg.Text("Attitude duration:", font=font), sg.InputText(str(self.value_list[6]), font= font, key="-INPUTTEXT106-", size=(5,3), background_color="white"), sg.Text("s", font = font)],
            [sg.Text("", font = font)],
            [sg.Button("Save", font=font), sg.Button("Exit ", font=font, pad=((150,0),(0,0)))]])
            while(True):
                e, v = settings_window.read()

                if e == sg.WIN_CLOSED:
                    break
                print (list(v.values()))
                for idx, value in enumerate(list(v.values())):
                    try:
                        self.value_list[idx] = float(value)
                    except ValueError:
                        print ("Invalid Input, setting to 0.0")
                        self.value_list[idx] = 0.0
                        settings_window["-INPUTTEXT10" + str(idx) + "-"].update(value="{:0.2f}".format(0.00))
                        

                if e == "Save":
                    jdx = 0
                    for idx, value in enumerate(self.value_list):
                        window["-INPUTTEXT" + str(jdx+1) + "-"].update(value="{:0.2f}".format(value))
                        window["-INPUTTEXT" + str(jdx+2) + "-"].update(value="{:0.2f}".format(value))
                        if ((idx != 2) and (idx != 3) and (idx != 4) and (idx != 5)):
                            print(idx)
                            window["-INPUTTEXT" + str(jdx+3) + "-"].update(value="{:0.2f}".format(value))
                            window["-INPUTTEXT" + str(jdx+4) + "-"].update(value="{:0.2f}".format(value))
                            jdx = jdx + 4
                        else:
                            jdx = jdx + 2
                        print("-INPUTTEXT" + str(jdx+1) + "-")
        

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
            window["-P_CONTROL-"].update(visible=True)
            window["-COL5-"].update(visible=False)
            window["-COL4-"].update(visible=True)
            window["-COL7-"].update(visible=False)
            
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
        else:
            input = event.split(":")
            if (input[0]=="t"):
                window["-key_pressed-"].update(value=input[0])
                try:
                    threading.Thread(target=self.take_off, daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')                

            elif (input[0]=="y"):
                window["-key_pressed-"].update(value=input[0])
                try:
                    threading.Thread(target=self.land, daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            elif (input[0]=="Up"):
                window["-key_pressed-"].update(value="↑")
                lineal = [self.value_list[0], 0.0, 0.0]
                print(lineal)
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            elif (input[0]=="Down"):
                window["-key_pressed-"].update(value="↓")
                lineal = [-self.value_list[1], 0.0, 0.0]
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')  

            elif (input[0]=="Left"):
                window["-key_pressed-"].update(value="←")
                lineal = [0.0, self.value_list[2], 0.0]
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')  

            elif (input[0]=="Right"):
                window["-key_pressed-"].update(value="→")
                lineal = [0.0, -self.value_list[3], 0.0]
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            elif (input[0]=="w"):
                window["-key_pressed-"].update(value=input[0])
                lineal = [0.0, -self.value_list[3], 0.0]
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            elif (input[0]=="s"):
                window["-key_pressed-"].update(value=input[0])
                lineal = [0.0, -self.value_list[3], 0.0]
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')

            elif (input[0]=="a"):
                window["-key_pressed-"].update(value=input[0])
                lineal = [0.0, -self.value_list[3], 0.0]
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')
                
            elif (input[0]=="d"):
                window["-key_pressed-"].update(value=input[0])
                lineal = [0.0, -self.value_list[3], 0.0]
                angular = [0.0, 0.0, 0.0]
                try:
                    threading.Thread(target=self.move_at_speed, args=(lineal, angular,), daemon=True).start()
                except Exception as e:
                    print('Error starting work thread.')
                               
        return True

    def make_window(self):
        sg.theme("DarkBlack1")
        font = ("Terminus Font", 14)
        font_menu = ("Ubuntu Mono", 18, 'bold')
        col1_layout = [ 
                        [sg.Text("t", font = font)],
                        [sg.Text("y", font = font)],
                        [sg.Text("h", font = font)],
                        [sg.Text("del", font = font)],
                        [sg.Text("r", font = font)]]
        col2_layout = [
                        [sg.Text("Take off", font = font)],
                        [sg.Text("Land", font = font)],
                        [sg.Text("Hover", font=font)],
                        [sg.Text("Emergency Stop", font = font)],
                        [sg.Text("Reset orientation", font = font)]]

        col3_layout = [
                        [sg.Text("↑", font = font)],
                        [sg.Text("↓", font = font)],
                        [sg.Text("←", font = font)],
                        [sg.Text("→", font = font)],
                        [sg.Text("", font = font)]]
        col4_layout = [
                        [sg.Text("Increase forward speed", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT1-"), sg.Text("m/s", font=font)],
                        [sg.Text("Increase backward speed", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT2-"), sg.Text("m/s", font=font)],
                        [sg.Text("Increase speed to the right", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT3-"), sg.Text("m/s", font=font)],
                        [sg.Text("Increase speed to the left", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT4-"), sg.Text("m/s", font=font)],
                        [sg.Text("", font = font)]
                        ]

        col5_layout = [
                        [sg.Text("Pitch", font = font),sg.Text("0.20", font= font, key="-INPUTTEXT13-"), sg.Text("rad during", font = font),sg.Text("0.50", font= font, key="-INPUTTEXT17-"), sg.Text("s", font = font)],
                        [sg.Text("Pitch -", font = font),sg.Text("0.20", font= font, key="-INPUTTEXT14-"), sg.Text("rad during", font = font),sg.Text("0.50", font= font, key="-INPUTTEXT18-"), sg.Text("s", font = font)],
                        [sg.Text("Roll", font = font),sg.Text("0.20", font= font, key="-INPUTTEXT15-"), sg.Text("rad during", font = font),sg.Text("0.50", font= font, key="-INPUTTEXT19-"), sg.Text("s", font = font)],
                        [sg.Text("Roll -", font = font),sg.Text("0.20", font= font, key="-INPUTTEXT16-"), sg.Text("rad during", font = font),sg.Text("0.50", font= font, key="-INPUTTEXT20-"), sg.Text("s", font = font)],
                        [sg.Text("", font = font)]                       
                        ]

        col6_layout = [
                        [sg.Text("Increase altitude", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT9-"),sg.Text("m", font = font)],
                        [sg.Text("Decrease altitude", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT10-"),sg.Text("m", font = font)],
                        [sg.Text("Turn counter-clockwise", font = font), sg.Text("0.10", font= font, key="-INPUTTEXT11-"),sg.Text("rad", font = font)],
                        [sg.Text("Turn clockwise", font = font), sg.Text("0.10", font= font, key="-INPUTTEXT12-"),sg.Text("rad", font = font)]                       
                        ]
                        
        col7_layout = [
                        [sg.Text("Increase forward position", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT5-"),sg.Text("m", font = font)],
                        [sg.Text("Increase backward position", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT6-"),sg.Text("m", font = font)],
                        [sg.Text("Increase position to the right", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT7-"),sg.Text("m", font = font)],
                        [sg.Text("Increase position to the left", font = font), sg.Text("1.00", font= font, key="-INPUTTEXT8-"),sg.Text("m", font = font)],
                        [sg.Text("", font = font)]
                        ]

        col8_layout = [
                        [sg.Text("w", font = font)],
                        [sg.Text("s", font = font)],
                        [sg.Text("a", font = font)],
                        [sg.Text("d", font = font)]
                        ]

        col_button_layout = [
                        [sg.Button("Speed mode", font = font, key="-SPEED-", focus=True)],
                        [sg.Button("Pose mode", font = font, key="-POSE-")],
                        [sg.Button("Attitude mode", font = font, key="-ATTITUDE-")]
                        ]

        

        self.layout = [[sg.Button("Settings", font=font_menu), sg.Text("Teleoperation mode: Speed mode", justification="left", font = font_menu, key = "-HEADER_SPEED-", visible=True, pad=((300, 0),(0, 0))),
        sg.Text("Teleoperation mode: Pose mode", justification="left", font = font_menu, key = "-HEADER_POSE-", visible=False, pad=((300, 0),(0, 0))),
        sg.Text("Teleoperation mode: Attitude mode", justification="left", font = font_menu, key = "-HEADER_ATTITUDE-", visible=False, pad=((300, 0),(0, 0)))],
         [sg.HSeparator(pad=(0, 10))],
         [sg.Text("BASIC MOTIONS", pad=((10, 280),(10, 0)), font = font_menu), sg.Text("SPEED CONTROL", pad=((0,0),(10, 0)), font = font_menu, key="-SP_CONTROL-"), sg.Text("ATTITUDE CONTROL", pad=((0,0),(10, 0)), font = font_menu, visible=False, key="-AT_CONTROL-"), sg.Text("POSE CONTROL", pad=((0,0),(10, 0)), font = font_menu, visible=False, key="-POS_CONTROL-")],
         [sg.Column(col1_layout, element_justification='left'), sg.Column(col2_layout, element_justification='left', pad=((0,210), (0,0))), 
         sg.Column(col3_layout, element_justification='left', justification="left"), sg.Column(col4_layout, element_justification='left', justification="left", key="-COL4-"), sg.Column(col5_layout, element_justification='left', visible=False, key="-COL5-"), sg.Column(col7_layout, element_justification='left', visible=False, key="-COL7-")],
         [sg.Text("TELEOPERATION MODE SELECTION", pad=((10, 100),(10, 0)), font = font_menu), sg.Text("POSE CONTROL", pad=((0,0),(10, 0)), font = font_menu, key="-P_CONTROL-")],
         [sg.Column(col_button_layout, element_justification='left', pad=((0,270), (0,0))), sg.Column(col8_layout, element_justification='left', key="-COL8-"), sg.Column(col6_layout, element_justification='left', key="-COL6-")],
         [sg.HSeparator(pad=(0, 10))],
         [sg.Text("Last key pressed:", font = font_menu), sg.Text("",font = font_menu, key="-key_pressed-")]]
        
        return sg.Window("Keyboard Teleoperation", self.layout, return_keyboard_events=True, use_default_focus=False, resizable=True, finalize=True)

    def tick_window(self):
        while self.execute_window(self.window):
            pass
        else:
            self.shutdown()
            
    def shutdown(self):
        self.t.join()

    def take_off(self):
        self.uav.arm()
        self.uav.offboard()
        self.uav.takeoff(1.0, 1.0)

    def land(self):
        self.uav.land(0.5)

    def move_at_speed(self, lineal, angular):
        
        self.uav.send_motion_reference_twist(lineal, angular)

    """def go_to_position(self, position, orientation):
        self.uav.send_motion_reference_position()"""

if __name__ == '__main__':
    main()
