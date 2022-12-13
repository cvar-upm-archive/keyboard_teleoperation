import PySimpleGUI as sg
from keyboard_teleoperation.settings_window import SettingsWindow
from keyboard_teleoperation.localization_window import LocalizationWindow
from keyboard_teleoperation.config_values import KeyMappings
from keyboard_teleoperation.config_values import ControlValues
from keyboard_teleoperation.config_values import ControlModes


class MainWindow(sg.Window):
    def __init__(self, settings_window: SettingsWindow,
                 localization_window: LocalizationWindow, font, menu_font,
                 drone_id_list, value_list, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.settings_window = settings_window
        self.localization_window = localization_window

        self.font = font
        self.menu_font = menu_font
        self.return_keyboard_events = True
        self.use_default_focus = True
        self.resizable = True
        self.drone_id_list = drone_id_list
        self.control_mode = ControlModes.SPEED_CONTROL.value
        self.value_list = value_list
        self.localization_opened = False

    def make_main_window(self):

        col1_layout = [
            [sg.Text(KeyMappings.TAKE_OFF_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.LAND_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.HOVER_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.EMERGENCY_KEY.value, font=self.font)],
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
                "{:0.2f}".format(ControlValues.SPEED_VALUE.value),
                font=self.font, key="-INPUTTEXT1-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase backward speed", font=self.font), sg.Text(
                "{:0.2f}".format(ControlValues.SPEED_VALUE.value),
                font=self.font, key="-INPUTTEXT2-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the right", font=self.font), sg.Text(
                "{:0.2f}".format(ControlValues.SPEED_VALUE.value),
                font=self.font, key="-INPUTTEXT3-"), sg.Text("m/s", font=self.font)],
            [sg.Text("Increase speed to the left", font=self.font), sg.Text(
                "{:0.2f}".format(ControlValues.SPEED_VALUE.value),
                font=self.font, key="-INPUTTEXT4-"), sg.Text("m/s", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col5_layout = [
            [sg.Text("Pitch", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.PITCH_ANGLE_VALUE.value),
                     font=self.font, key="-INPUTTEXT17-"),
             sg.Text("rad during", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ATTITUDE_DURATION.value),
                     font=self.font, key="-INPUTTEXT21-"),
             sg.Text("s", font=self.font)],
            [sg.Text("Pitch -", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.PITCH_ANGLE_VALUE.value),
                     font=self.font, key="-INPUTTEXT18-"),
             sg.Text("rad during", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ATTITUDE_DURATION.value),
                     font=self.font, key="-INPUTTEXT22-"),
             sg.Text("s", font=self.font)],
            [sg.Text("Roll", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ROLL_ANGLE_VALUE.value),
                     font=self.font, key="-INPUTTEXT19-"),
             sg.Text("rad during", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ATTITUDE_DURATION.value),
                     font=self.font, key="-INPUTTEXT23-"),
             sg.Text("s", font=self.font)],
            [sg.Text("Roll -", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ROLL_ANGLE_VALUE.value),
                     font=self.font, key="-INPUTTEXT20-"),
             sg.Text("rad during", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ATTITUDE_DURATION.value),
                     font=self.font, key="-INPUTTEXT24-"),
             sg.Text("s", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col6_layout = [
            [sg.Text("Increase altitude", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ALTITUDE_VALUE.value),
                     font=self.font, key="-INPUTTEXT13-"),
             sg.Text("m", font=self.font)],
            [sg.Text("Decrease altitude", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.ALTITUDE_VALUE.value),
                     font=self.font, key="-INPUTTEXT14-"),
             sg.Text("m", font=self.font)],
            [sg.Text("Turn counter-clockwise", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.TURN_ANGLE_VALUE.value),
                     font=self.font, key="-INPUTTEXT15-"),
             sg.Text("rad", font=self.font)],
            [sg.Text("Turn clockwise", font=self.font),
             sg.Text("{:0.2f}".format(ControlValues.TURN_ANGLE_VALUE.value),
                     font=self.font, key="-INPUTTEXT16-"),
             sg.Text("rad", font=self.font)]
        ]

        col6B_layout = [
            [sg.Text("Increase vertical speed", font=self.font),
             sg.Text("{:0.2f}".format(
                 ControlValues.VERTICAL_VALUE.value), font=self.font, key="-INPUTTEXT5-"),
             sg.Text("m/s", font=self.font)],
            [sg.Text("Decrease vertical speed", font=self.font),
             sg.Text("{:0.2f}".format(
                 ControlValues.VERTICAL_VALUE.value), font=self.font, key="-INPUTTEXT6-"),
             sg.Text("m/s", font=self.font)],
            [sg.Text("Turn speed counter-clockwise", font=self.font),
             sg.Text("{:0.2f}".format(
                 ControlValues.TURN_SPEED_VALUE.value), font=self.font, key="-INPUTTEXT7-"),
             sg.Text("rad/s", font=self.font)],
            [sg.Text("Turn speed clockwise", font=self.font),
             sg.Text("{:0.2f}".format(
                 ControlValues.TURN_SPEED_VALUE.value), font=self.font, key="-INPUTTEXT8-"),
             sg.Text("rad/s", font=self.font)]
        ]

        col7_layout = [
            [sg.Text("Increase forward position", font=self.font), sg.Text(
                "{:0.2f}".format(ControlValues.POSITION_VALUE.value), font=self.font, key="-INPUTTEXT9-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase backward position", font=self.font), sg.Text(
                "{:0.2f}".format(ControlValues.POSITION_VALUE.value), font=self.font, key="-INPUTTEXT10-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the right", font=self.font), sg.Text(
                "{:0.2f}".format(ControlValues.POSITION_VALUE.value), font=self.font, key="-INPUTTEXT11-"), sg.Text("m", font=self.font)],
            [sg.Text("Increase position to the left", font=self.font), sg.Text(
                "{:0.2f}".format(ControlValues.POSITION_VALUE.value), font=self.font, key="-INPUTTEXT12-"), sg.Text("m", font=self.font)],
            [sg.Text("", font=self.font)]
        ]

        col8_layout = [
            [sg.Text(KeyMappings.UP_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.DOWN_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.ROTATE_LEFT_KEY.value, font=self.font)],
            [sg.Text(KeyMappings.ROTATE_RIGHT_KEY.value, font=self.font)]
        ]

        col_button_layout = [
            [sg.Button("Speed mode", font=self.font,
                       key=ControlModes.SPEED_CONTROL.value, focus=True)],
            [sg.Button("Pose mode", font=self.font,
                       key=ControlModes.POSE_CONTROL.value)],
            [sg.Button("Attitude mode", font=self.font,
                       key=ControlModes.ATTITUDE_CONTROL.value)]
        ]

        main_buttons_layout = [
            [sg.Text("BASIC MOTIONS", pad=((10, 280), (10, 0)), font=self.menu_font), sg.Text("SPEED CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font, key="-SP_CONTROL-"), sg.Text("ATTITUDE CONTROL",
                                                                                                                                                                                        pad=((0, 0), (10, 0)), font=self.menu_font, visible=False, key="-AT_CONTROL-"), sg.Text("POSE CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font, visible=False, key="-POS_CONTROL-")],
            [sg.Column(col1_layout, element_justification='left'), sg.Column(col2_layout, element_justification='left', pad=((0, 190), (0, 0))),
             sg.Column(col3_layout, element_justification='left', justification="left"), sg.Column(
                 col4_layout, element_justification='left', justification="left", key="-COL4-"),
             sg.Column(col5_layout, element_justification='left', visible=False, key="-COL5-"), sg.Column(col7_layout, element_justification='left', visible=False, key="-COL7-")],
            [sg.Text("TELEOPERATION MODE SELECTION", pad=((10, 100), (10, 0)), font=self.menu_font), sg.Text(
                "POSE CONTROL", pad=((0, 0), (10, 0)), font=self.menu_font, key="-P_CONTROL-", visible=False)],
            [sg.Column(col_button_layout, element_justification='left', pad=((0, 270), (0, 0))), sg.Column(
                col8_layout, element_justification='left', key="-COL8-"), sg.Column(col6_layout, element_justification='left', key="-COL6-", visible=False), sg.Column(col6B_layout, element_justification='left', key="-COL6B-")]]

        col_selection_layout = list()
        # Here active behavior list is added
        col_active_behavior_layout = ["Behavior Take Off"]
        col_paused_behavior_layout = []

        all_selector = True

        for drone_id in self.drone_id_list:
            col_selection_layout.append(
                [sg.CB(drone_id[0], key=drone_id[0], enable_events=True, font=self.font, background_color="grey", default=drone_id[1])])
            if not drone_id[1]:
                all_selector = False

        selection_frame = sg.Frame("Drone selection control", layout=[[sg.Column(col_selection_layout, expand_y=True, scrollable=True, vertical_scroll_only=True, background_color="grey", sbar_trough_color="white", sbar_arrow_color="grey")],
                                                                      [sg.CB("All", key="All", enable_events=True, font=self.font, background_color="grey", expand_x=True, default=all_selector)]], vertical_alignment="top", size=(200, 300), expand_y=True)

        behavior_frame = sg.Frame("Behavior control", key="-BEHAVIOR CONTROL-",
                                  layout=[[sg.Text("Active Behaviors", font=self.menu_font, background_color="grey", expand_x=True, justification="center"), sg.Text("Paused Behaviors", font=self.menu_font, background_color="grey", expand_x=True, justification="center")],
                                          [sg.Listbox(col_active_behavior_layout, background_color="grey", font=self.font,
                                                      select_mode=sg.LISTBOX_SELECT_MODE_MULTIPLE, highlight_text_color="white", text_color="white", highlight_background_color="blue", expand_y=True, size=(17,), sbar_trough_color="white", sbar_arrow_color="grey"),
                                           sg.Listbox(col_paused_behavior_layout, background_color="grey", font=self.font,
                                                      select_mode=sg.LISTBOX_SELECT_MODE_MULTIPLE, highlight_text_color="white", text_color="white", highlight_background_color="blue", expand_y=True, size=(17,), sbar_trough_color="white", sbar_arrow_color="grey")],
                                          [sg.Button(" Pause ", font=self.font, key="-PAUSE_BEHAVIORS-", expand_x=True), sg.Button("Resume", font=self.font, key="-RESUME_BEHAVIORS-", expand_x=True)]],
                                  vertical_alignment="top", size=(470, 300), expand_y=True, visible=False)

        self.layout([
            [sg.Button("Settings", font=self.menu_font), sg.Text("|", font=self.menu_font), sg.Button("Localization", font=self.menu_font), sg.Text("|", font=self.menu_font),
             sg.Button("Behavior control",
                       font=self.menu_font, key="-BEHAVIOR-"),
             sg.Text("Teleoperation mode: Speed mode", justification="left",
                     font=self.menu_font, key="-HEADER_SPEED-", visible=True, pad=((78, 0), (0, 0))),
             sg.Text("Teleoperation mode: Pose mode", justification="left",
                    font=self.menu_font, key="-HEADER_POSE-", visible=False, pad=((78, 0), (0, 0))),
             sg.Text("Teleoperation mode: Attitude mode", justification="left", font=self.menu_font, key="-HEADER_ATTITUDE-", visible=False, pad=((78, 0), (0, 0)))],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Column(layout=main_buttons_layout),
             selection_frame, behavior_frame],
            [sg.HSeparator(pad=(0, 10))],
            [sg.Text("Last key pressed:", font=self.menu_font), sg.Text("", font=self.menu_font, key="-key_pressed-")]])

    def event_handler(self, event, value):

        if event == sg.WIN_CLOSED:
            if (self.localization_opened):
                self.localization_window.close()
            return None, None, None, False

        selection_values = list(value.values())[:len(self.drone_id_list)+1]

        if event == "Localization":  # Non-Blocking
            if (not self.localization_opened):
                self.localization_window.make_localization_window(
                    location=self.current_location())

                if self.localization_window._Hidden:
                    self.localization_window.move(self.current_location()[
                                                  0], self.current_location()[1])
                    self.localization_window.un_hide()

                self.localization_opened = True

        elif event == "Settings":  # Blocking
            self.settings_window.make_settings_window(
                location=self.current_location())

            if self.settings_window._Hidden:
                self.settings_window.move(self.current_location()[
                                          0], self.current_location()[1])
                self.settings_window.un_hide()

            while (True):
                settings_event, settings_value = self.settings_window.read()  # type: ignore

                self.value_list, opened = self.settings_window.event_handler(
                    self, settings_event, settings_value)

                if not opened:
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

        elif event in ControlModes.list() or event == "-BEHAVIOR-":
            self.update_main_window_mode(event)

        elif event.split(":")[0] in KeyMappings.list():
            key = event.split(":")
            self["-key_pressed-"].update(value=key[0])
            if (key[0] == KeyMappings.FORWARD_KEY.value):
                self["-key_pressed-"].update(value="↑")
            elif (key[0] == KeyMappings.BACKWARD_KEY.value):
                self["-key_pressed-"].update(value="↓")
            elif (key[0] == KeyMappings.LEFT_KEY.value):
                self["-key_pressed-"].update(value="←")
            elif (key[0] == KeyMappings.RIGHT_KEY.value):
                self["-key_pressed-"].update(value="→")

            return self.control_mode, key[0], self.value_list, True

        if (self.localization_opened):
            self.localization_opened = self.localization_window.execute_localization_window()

        return None, None, None, True

    def update_main_window_mode(self, event):

        if event == ControlModes.SPEED_CONTROL.value:
            self.control_mode = event
            self.update_window_to_speed()

        elif event == ControlModes.POSE_CONTROL.value:
            self.control_mode = event
            self.update_window_to_pose()

        elif event == ControlModes.ATTITUDE_CONTROL.value:
            self.control_mode = event
            self.update_window_to_attitude()

        elif event == "-BEHAVIOR-":
            self["-BEHAVIOR CONTROL-"].update(
                visible=(not self["-BEHAVIOR CONTROL-"].visible))

    def update_window_to_pose(self):

        self[ControlModes.POSE_CONTROL.value].set_focus(True)
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

        self[ControlModes.SPEED_CONTROL.value].set_focus(True)
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

        self[ControlModes.ATTITUDE_CONTROL.value].set_focus(True)
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
