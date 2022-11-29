import PySimpleGUI as sg
from settings_window import SettingsWindow
from localization_window import LocalizationWindow
from drone_manager import DroneManager

class MainWindow(sg.Window):
    def __init__(self, drone_manager: DroneManager, font, menu_font, drone_id_list, value_list, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.settings_window = SettingsWindow(font=font, menu_font=menu_font, value_list=value_list, title="Settings", enable_close_attempted_event=True)
        self.localization_window = LocalizationWindow(font=font, menu_font=menu_font, uav_list=drone_manager.uav_list, title="Localization",
         size=(330, 200), use_default_focus=False, enable_close_attempted_event=True)

        self.font = font
        self.menu_font = menu_font
        self.return_keyboard_events=True
        self.use_default_focus=True
        self.resizable=True
        self.drone_id_list = drone_id_list
        self.drone_manager = drone_manager
        self.control_mode = "-SPEED-"
        self.value_list = value_list
        self.localization_opened = False
        
    def make_main_window(self):

        col1_layout = [
            [sg.Text("t", font=self.font)],
            [sg.Text("l", font=self.font)],
            [sg.Text("space", font=self.font)],
            [sg.Text("del", font=self.font)],
            [sg.Text("r", font=self.font)]]
            
        col2_layout = [
            [sg.Text("Take off", font=self.font)],
            [sg.Text("Land", font=self.font)],
            [sg.Text("Hover", font=self.font)],
            [sg.Text("Emergency Stop", font=self.font)],
            [sg.Text("Reset orientation", font=self.font)]]

        col3_layout = [
            [sg.Text("↑", font=self.font)],
            [sg.Text("↓", font=self.font)],
            [sg.Text("←", font=self.font)],
            [sg.Text("→", font=self.font)],
            [sg.Text("", font=self.font)]]

        col4_layout = [
            [sg.Text("Increase forward speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT1-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase backward speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT2-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the right", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT3-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the left", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT4-"), sg.Text("m/s", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col5_layout = [
            [sg.Text("Pitch", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT17-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT21-"), sg.Text("s", font=self.font)],
            [sg.Text("Pitch -", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT18-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT22-"), sg.Text("s", font=self.font)],
            [sg.Text("Roll", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT19-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT23-"), sg.Text("s", font=self.font)],
            [sg.Text("Roll -", font=self.font), sg.Text("0.20", font=self.font, key="-INPUTTEXT20-"), sg.Text(
                "rad during", font=self.font), sg.Text("0.50", font=self.font, key="-INPUTTEXT24-"), sg.Text("s", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col6_layout = [
            [sg.Text("Increase altitude", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT13-"), sg.Text("m", font=self.font)],
            [sg.Text("Decrease altitude", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT14-"), sg.Text("m", font=self.font)],
            [sg.Text("Turn counter-clockwise", font=self.font), sg.Text("0.10",
                                                                    font=self.font, key="-INPUTTEXT15-"), sg.Text("rad", font=self.font)],
            [sg.Text("Turn clockwise", font=self.font), sg.Text(
                "0.10", font=self.font, key="-INPUTTEXT16-"), sg.Text("rad", font=self.font)]
        ]

        col6B_layout = [
            [sg.Text("Increase vertical speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT5-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Decrease vertical speed", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT6-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Turn speed counter-clockwise", font=self.font), sg.Text(
                "0.10", font=self.font, key="-INPUTTEXT7-"), sg.Text("rad/s", font=self.font)],
            [sg.Text("Turn speed clockwise", font=self.font), sg.Text(
                "0.10", font=self.font, key="-INPUTTEXT8-"), sg.Text("rad/s", font=self.font)]
        ]

        col7_layout = [
            [sg.Text("Increase forward position", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT9-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase backward position", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT10-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the right", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT11-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the left", font=self.font), sg.Text(
                "1.00", font=self.font, key="-INPUTTEXT12-"), sg.Text("m", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col8_layout = [
            [sg.Text("w", font=self.font)],
            [sg.Text("s", font=self.font)],
            [sg.Text("a", font=self.font)],
            [sg.Text("d", font=self.font)]
        ]

        col_button_layout = [
            [sg.Button("Speed mode", font=self.font, key="-SPEED-", focus=True)],
            [sg.Button("Pose mode", font=self.font, key="-POSE-")],
            [sg.Button("Attitude mode", font=self.font, key="-ATTITUDE-")]
        ]

        main_buttons_layout = [
            [sg.Text("BASIC MOTIONS", pad=((10, 280), (10, 0)), font=self.menu_font), sg.Text("SPEED CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font, key="-SP_CONTROL-"), sg.Text("ATTITUDE CONTROL",
                pad=((0, 0), (10, 0)), font=self.menu_font, visible=False, key="-AT_CONTROL-"), sg.Text("POSE CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font, visible=False, key="-POS_CONTROL-")],
            [sg.Column(col1_layout, element_justification='left'), sg.Column(col2_layout, element_justification='left', pad=((0, 190), (0, 0))),
            sg.Column(col3_layout, element_justification='left', justification="left"), sg.Column(col4_layout, element_justification='left', justification="left", key="-COL4-"),
            sg.Column(col5_layout, element_justification='left', visible=False, key="-COL5-"), sg.Column(col7_layout, element_justification='left', visible=False, key="-COL7-")],
            [sg.Text("TELEOPERATION MODE SELECTION", pad=((10, 100), (10, 0)), font=self.menu_font), sg.Text(
                "POSE CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font, key="-P_CONTROL-", visible=False)],
            [sg.Column(col_button_layout, element_justification='left', pad=((0, 270), (0, 0))), sg.Column(
                col8_layout, element_justification='left', key="-COL8-"), sg.Column(col6_layout, element_justification='left', key="-COL6-", visible=False), sg.Column(col6B_layout, element_justification='left', key="-COL6B-")]]

        col_selection_layout = list()
        col_active_behavior_layout = ["Behavior Take Off"]
        col_paused_behavior_layout = []

        all_selector = True

        for drone_id in self.drone_id_list: 
            col_selection_layout.append(
                [sg.CB(drone_id[0], key=drone_id[0], enable_events=True, font=self.font, background_color="grey", default=drone_id[1])])
            if not drone_id[1]:
                all_selector = False

        selection_frame = sg.Frame("Drone selection control", layout=[[sg.Column(col_selection_layout, expand_y=True, scrollable=True, vertical_scroll_only=True, background_color="grey")],
            [sg.CB("All", key="All", enable_events=True, font=self.font, background_color="grey", expand_x=True, default=all_selector)]], vertical_alignment="top", size=(200, 300), expand_y=True)

        behavior_frame = sg.Frame("Behavior control",key="-BEHAVIOR CONTROL-",
        layout=[[sg.Text("Active Behaviors", font=self.menu_font, background_color="grey", expand_x=True, justification="center"), sg.Text("Paused Behaviors", font=self.menu_font, background_color="grey", expand_x=True, justification="center")],
        [sg.Listbox(col_active_behavior_layout, background_color="grey", font=self.font,
        select_mode=sg.LISTBOX_SELECT_MODE_MULTIPLE, highlight_text_color="white", text_color="white", highlight_background_color="blue", expand_y=True, size=(17,)),
        sg.Listbox(col_paused_behavior_layout, background_color="grey", font=self.font,
        select_mode=sg.LISTBOX_SELECT_MODE_MULTIPLE, highlight_text_color="white", text_color="white", highlight_background_color="blue", expand_y=True, size=(17,))],
        [sg.Button(" Pause ", font=self.font, key="-PAUSE_BEHAVIORS-", expand_x=True), sg.Button("Resume", font=self.font, key="-RESUME_BEHAVIORS-", expand_x=True)]],
        vertical_alignment="top", size=(470, 300), expand_y=True, visible=False)
        
        self.layout([
            [sg.Button("Settings", font=self.menu_font), sg.Text("|", font=self.menu_font), sg.Button("Localization", font=self.menu_font), sg.Text("|", font=self.menu_font),
            sg.Button("Behavior control", font=self.menu_font, key="-BEHAVIOR-"),
            sg.Text("Teleoperation mode: Speed mode", justification="left", font=self.menu_font, key="-HEADER_SPEED-", visible=True, pad=((78, 0), (0, 0))),
            sg.Text("Teleoperation mode: Pose mode", justification="left",
                    font=self.menu_font, key="-HEADER_POSE-", visible=False, pad=((78, 0), (0, 0))),
            sg.Text("Teleoperation mode: Attitude mode", justification="left", font=self.menu_font, key="-HEADER_ATTITUDE-", visible=False, pad=((78, 0), (0, 0)))],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Column(layout = main_buttons_layout), selection_frame, behavior_frame],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Text("Last key pressed:", font=self.menu_font), sg.Text("", font=self.menu_font, key="-key_pressed-")]])

    def event_handler(self, event, value):

        if event == sg.WIN_CLOSED:
            if (self.localization_opened):
                self.localization_window.close()
            return False

        selection_values = list(value.values())[:len(self.drone_id_list)+1]

        if event == "Localization": #Non-Blocking
            if (not self.localization_opened):
                self.localization_window.make_localization_window(location=self.current_location())

                if self.localization_window._Hidden:
                    self.localization_window.move(self.current_location()[0], self.current_location()[1])
                    self.localization_window.un_hide()

                self.localization_opened = True

        elif event == "Settings": #Blocking 
            self.settings_window.make_settings_window(location=self.current_location())
            
            if self.settings_window._Hidden:
                self.settings_window.move(self.current_location()[0], self.current_location()[1])
                self.settings_window.un_hide()

            while (True):
                settings_event, settings_value = self.settings_window.read()  # type: ignore

                if not self.settings_window.event_handler(self, settings_event, settings_value):
                    break

        elif event == "All":
            
            if (selection_values[-1]):
                for index, value in enumerate(selection_values[:-1]):
                    self[self.drone_id_list[index][0]].update(True)
            else:
                for index, value in enumerate(selection_values[:-1]):
                    self[self.drone_id_list[index][0]].update(False)

            for drone_id in self.drone_id_list:
                 drone_id[1] = True

        elif event in [x for l in self.drone_id_list for x in l]:

            if all(selection_values[:-1]):
                self["All"].update(True)
            else:
                self["All"].update(False)

            for index, value in enumerate(selection_values[:-1]):
                self.drone_id_list[index][1] = bool(value)

        elif event in ["-SPEED-", "-POSE-", "-ATTITUDE-", "-BEHAVIOR-"]:
            self.update_main_window_mode(event)

        else:
            input = event.split(":")
            if input[0] in {"t","l","space","Delete","w","s","a","d"}:
                self["-key_pressed-"].update(value=input[0])
            elif (input[0] == "Up"):
                self["-key_pressed-"].update(value="↑")
            elif (input[0] == "Down"):
                self["-key_pressed-"].update(value="↓")
            elif (input[0] == "Left"):
                self["-key_pressed-"].update(value="←")
            elif (input[0] == "Right"):
                self["-key_pressed-"].update(value="→")
            
            self.drone_manager.manage_common_behaviors(input[0])

            if (self.control_mode == "-SPEED-"):
                self.drone_manager.manage_speed_behaviors(input[0], self.value_list)

            elif (self.control_mode == "-POSE-"):
                self.drone_manager.manage_pose_behaviors(input[0], self.value_list)

            if (self.localization_opened):
                self.localization_opened = self.localization_window.execute_localization_window()
            
        return True

    def update_main_window_mode(self, event):

        if event == "-SPEED-":
            self.control_mode = event
            self.update_window_to_speed()

        elif event == "-POSE-":
            self.control_mode = event
            self.update_window_to_pose()

        elif event == "-ATTITUDE-":
            self.control_mode = event
            self.update_window_to_attitude()

        elif event == "-BEHAVIOR-":
            self["-BEHAVIOR CONTROL-"].update(visible=(not self["-BEHAVIOR CONTROL-"].visible))

    def update_window_to_pose(self):

        self["-POSE-"].set_focus(True)
        self["-HEADER_SPEED-"].update(visible=False)
        self["-HEADER_POSE-"].update(visible=True)
        self["-HEADER_ATTITUDE-"].update(visible=False)
        self["-SP_CONTROL-"].update(visible=False)
        self["-POS_CONTROL-"].update(visible=True)
        self["-AT_CONTROL-"].update(visible=False)
        self["-COL5-"].update(visible=False)
        self["-COL4-"].update(visible=False)
        self["-P_CONTROL-"].update(visible=False)
        self["-COL7-"].update(visible=True)
        self["-COL6B-"].update(visible=False)
        self["-COL6-"].update(visible=True)


    def update_window_to_speed(self):

        self["-SPEED-"].set_focus(True)
        self["-HEADER_SPEED-"].update(visible=True)
        self["-HEADER_POSE-"].update(visible=False)
        self["-HEADER_ATTITUDE-"].update(visible=False)
        self["-SP_CONTROL-"].update(visible=True)
        self["-POS_CONTROL-"].update(visible=False)
        self["-AT_CONTROL-"].update(visible=False)
        self["-P_CONTROL-"].update(visible=False)
        self["-COL5-"].update(visible=False)
        self["-COL4-"].update(visible=True)
        self["-COL7-"].update(visible=False)
        self["-COL6B-"].update(visible=True)
        self["-COL6-"].update(visible=False)


    def update_window_to_attitude(self):

        self["-ATTITUDE-"].set_focus(True)
        self["-HEADER_SPEED-"].update(visible=False)
        self["-HEADER_POSE-"].update(visible=False)
        self["-HEADER_ATTITUDE-"].update(visible=True)
        self["-SP_CONTROL-"].update(visible=False)
        self["-POS_CONTROL-"].update(visible=False)
        self["-AT_CONTROL-"].update(visible=True)
        self["-P_CONTROL-"].update(visible=True)
        self["-COL5-"].update(visible=True)
        self["-COL4-"].update(visible=False)
        self["-COL7-"].update(visible=False)
        self["-COL6B-"].update(visible=False)
        self["-COL6-"].update(visible=True)