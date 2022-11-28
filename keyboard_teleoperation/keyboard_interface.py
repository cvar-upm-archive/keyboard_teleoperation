import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface

class KeyboardInterface:
    def __init__(self, theme, font, menu_font):
        sg.theme(theme)
        self.font = font
        self.menu_font = menu_font

    def make_main_window(self, drone_id_list):
        
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

        for drone_id in drone_id_list: 
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
        

        self.layout = [
            [sg.Button("Settings", font=self.menu_font), sg.Text("|", font=self.menu_font), sg.Button("Localization", font=self.menu_font), sg.Text("|", font=self.menu_font),
            sg.Button("Behavior control", font=self.menu_font, key="-BEHAVIOR-"),
            sg.Text("Teleoperation mode: Speed mode", justification="left", font=self.menu_font, key="-HEADER_SPEED-", visible=True, pad=((78, 0), (0, 0))),
            sg.Text("Teleoperation mode: Pose mode", justification="left",
                    font=self.menu_font, key="-HEADER_POSE-", visible=False, pad=((78, 0), (0, 0))),
            sg.Text("Teleoperation mode: Attitude mode", justification="left", font=self.menu_font, key="-HEADER_ATTITUDE-", visible=False, pad=((78, 0), (0, 0)))],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Column(layout = main_buttons_layout), selection_frame, behavior_frame],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Text("Last key pressed:", font=self.menu_font), sg.Text("", font=self.menu_font, key="-key_pressed-")]]

        return sg.Window("Keyboard Teleoperation", self.layout, return_keyboard_events=True, use_default_focus=True, resizable=True)

    def make_settings_window(self, location, value_list):
        col_value_settings_layout=[[sg.Text("Speed Control Values", font=self.menu_font)],
            [sg.Text("Speed value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[0])), font=self.font, key="-VALUE0-", size=(5, 3), background_color="white"), sg.Text("m/s", font=self.font)],
            [sg.Text("Vertical value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[1])), font=self.font, key="-VALUE1-", size=(5, 3), background_color="white"), sg.Text("m/s", font=self.font)],
            [sg.Text("Turn speed value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[2])), font=self.font, key="-VALUE2-", size=(5, 3), background_color="white"), sg.Text("rad/s", font=self.font)],
            [sg.Text(
                "", font=self.font)],
            [sg.Text(
                "Position control values:", font=self.menu_font)],
            [sg.Text("Position value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[3])), font=self.font, key="-VALUE3-", size=(5, 3), background_color="white"), sg.Text("m", font=self.font)],
            [sg.Text("Altitude value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[4])), font=self.font, key="-VALUE4-", size=(5, 3), background_color="white"), sg.Text("m", font=self.font)],
            [sg.Text("Turn angle value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[5])), font=self.font, key="-VALUE5-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
            [sg.Text(
                "", font=self.font)],
            [sg.Text(
                "Attitude control values:", font=self.menu_font)],
            [sg.Text("Pitch angle value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[6])), font=self.font, key="-VALUE6-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
            [sg.Text("Roll angle value:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[7])), font=self.font, key="-VALUE7-", size=(5, 3), background_color="white"), sg.Text("rad", font=self.font)],
            [sg.Text("Attitude duration:", font=self.font), sg.InputText(str(
                "{:0.2f}".format(value_list[8])), font=self.font, key="-VALUE8-", size=(5, 3), background_color="white"), sg.Text("s", font=self.font)],
            [sg.Text(
                "", font=self.font)],
            [sg.Button("Save", font=self.font), sg.Button("Exit", font=self.font, pad=((150, 0), (0, 0)))]]

        return sg.Window("Settings", layout=[[sg.Column(col_value_settings_layout)]],
                 location=location, use_default_focus=False)

    def make_localization_window(self, location, uav_list: list[DroneInterface]):

        return sg.Window("Localization", location=location, use_default_focus=False, size=(330, 200),
            layout=[[sg.Text("Position", font=self.menu_font)],
                    [sg.Text("x:", font=self.font), sg.Text("{:0.2f}".format(round(uav_list[0].position[0], 2)), font=self.font, key="-LOCALIZATION_X-"), sg.Text(",", font=self.font),
                    sg.Text("y:", font=self.font), sg.Text("{:0.2f}".format(round(
                        uav_list[0].position[1], 2)), font=self.font, key="-LOCALIZATION_Y-"), sg.Text(",", font=self.font),
                    sg.Text("z:", font=self.font), sg.Text("{:0.2f}".format(round(uav_list[0].position[2], 2)), font=self.font, key="-LOCALIZATION_Z-")],
                    [sg.Text(
                        "Orientation", font=self.menu_font)],
                    [sg.Text("r:", font=self.font), sg.Text("{:0.2f}".format(round(uav_list[0].orientation[0], 2)), font=self.font, key="-LOCALIZATION_R-"), sg.Text(",", font=self.font),
                    sg.Text("p:", font=self.font), sg.Text("{:0.2f}".format(round(
                        uav_list[0].orientation[1], 2)), font=self.font, key="-LOCALIZATION_P-"), sg.Text(",", font=self.font),
                    sg.Text("y:", font=self.font), sg.Text("{:0.2f}".format(round(uav_list[0].orientation[2], 2)), font=self.font, key="-LOCALIZATION_YW-")],
                    [sg.Button("Exit", font=self.font, pad=((240, 0), (20, 0)))]])