import PySimpleGUI as sg
from python_interface.drone_interface import DroneInterface

class LocalizationWindow(sg.Window):
    def __init__(self, font, menu_font, uav_list: list[DroneInterface], *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.font = font
        self.menu_font = menu_font
        self.uav_list = uav_list

    def make_localization_window(self, location):

        self.Location = location
        self.layout([[sg.Text("Position", font=self.menu_font)],
                [sg.Text("x:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav_list[0].position[0], 2)), font=self.font, key="-LOCALIZATION_X-"), sg.Text(",", font=self.font),
                sg.Text("y:", font=self.font), sg.Text("{:0.2f}".format(round(
                    self.uav_list[0].position[1], 2)), font=self.font, key="-LOCALIZATION_Y-"), sg.Text(",", font=self.font),
                sg.Text("z:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav_list[0].position[2], 2)), font=self.font, key="-LOCALIZATION_Z-")],
                [sg.Text(
                    "Orientation", font=self.menu_font)],
                [sg.Text("r:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav_list[0].orientation[0], 2)), font=self.font, key="-LOCALIZATION_R-"), sg.Text(",", font=self.font),
                sg.Text("p:", font=self.font), sg.Text("{:0.2f}".format(round(
                    self.uav_list[0].orientation[1], 2)), font=self.font, key="-LOCALIZATION_P-"), sg.Text(",", font=self.font),
                sg.Text("y:", font=self.font), sg.Text("{:0.2f}".format(round(self.uav_list[0].orientation[2], 2)), font=self.font, key="-LOCALIZATION_YW-")],
                [sg.Button("Exit", font=self.font, pad=((240, 0), (20, 0)))]])

    def execute_localization_window(self):
        localization_event, _ = self.read(timeout=1)  # type: ignore

        self["-LOCALIZATION_X-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].position[0], 2)))
        self["-LOCALIZATION_Y-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].position[1], 2))) 
        self["-LOCALIZATION_Z-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].position[2], 2)))

        self["-LOCALIZATION_R-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].orientation[0], 2)))
        self["-LOCALIZATION_P-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].orientation[1], 2)))
        self["-LOCALIZATION_YW-"].update(
            value="{:0.2f}".format(round(self.uav_list[0].orientation[2], 2)))

        # self.refresh()
        if localization_event == sg.WIN_CLOSE_ATTEMPTED_EVENT or localization_event == "Exit":
            self.hide()
            return False

        return True