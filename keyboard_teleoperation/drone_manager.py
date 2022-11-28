from python_interface.drone_interface import DroneInterface
import threading

TAKE_OFF_KEY = "t"
LAND_KEY = "l"
HOVER_KEY = "space"



class DroneManager:
    def __init__(self, uav_list: list[DroneInterface], drone_id_list, pose_frame_id, twist_frame_id):
        self.uav_list = uav_list
        self.drone_id_list = drone_id_list
        self.pose_frame_id = pose_frame_id
        self.twist_frame_id = twist_frame_id

    def manage_common_behaviors(self, input):

        if (input == "t"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    
                    try:
                        threading.Thread(target=self.take_off, args=(
                                self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "l"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    try:
                        threading.Thread(target=self.land, args=(
                                self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "space"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    try:
                        threading.Thread(target=self.hover, args=(
                                self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif input == "Delete":

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    try:
                        threading.Thread(target=self.emergency_stop, args=(
                            self.uav_list[index],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

    def manage_speed_behaviors(self, input, value_list):

        if (input == "Up"):
            
            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [value_list[0], 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "Down"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [-value_list[0], 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "Left"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, value_list[0], 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "Right"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, -value_list[0], 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "w"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, value_list[1]]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "s"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, -value_list[1]]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, 0.0,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "a"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, value_list[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "d"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:

                    lineal = [0.0, 0.0, 0.0]
                    try:
                        threading.Thread(target=self.move_at_speed, args=(
                            self.uav_list[index], lineal, -value_list[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

    def manage_pose_behaviors(self, input, value_list):

        if (input == "Up"):

            for index, drone_id in enumerate(self.drone_id_list):
                if drone_id[1] == True:

                    position = [self.uav_list[index].position[0] + value_list[3],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
                    
                    #orientation = quaternion_from_euler(self.uav.orientation[0], self.uav.orientation[1], self.uav.orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "Down"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0] - value_list[3],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
            #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "Left"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1] +
                                value_list[3], self.uav_list[index].position[2]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "Right"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1] -
                                value_list[3], self.uav_list[index].position[2]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "w"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1],
                                self.uav_list[index].position[2] + value_list[4]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "s"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0], self.uav_list[index].position[1],
                                self.uav_list[index].position[2] - value_list[4]]
                    #orientation = quaternion_from_euler(self.uav_list[index].orientation[0], self.uav_list[index].orientation[1], self.uav_list[index].orientation[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, self.uav_list[index].orientation[2],), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "a"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
                    euler = self.uav_list[index].orientation
                    #orientation = [self.uav_list[index].orientation.x, self.uav_list[index].orientation.y, self.uav_list[index].orientation.z, self.uav_list[index].orientation.w]
                    yaw = euler[2] + value_list[5]
                    #orientation = quaternion_from_euler(euler[0], euler[1], euler[2])
                    try:
                        threading.Thread(target=self.go_to_pose, args=(
                            self.uav_list[index], position, yaw,), daemon=True).start()
                    except Exception as e:
                        print('Error starting work thread.')

        elif (input == "d"):

            for index, drone_id in enumerate(self.drone_id_list): 
                if drone_id[1] == True:
                    position = [self.uav_list[index].position[0],
                                self.uav_list[index].position[1], self.uav_list[index].position[2]]
                    euler = self.uav_list[index].orientation
                    yaw = euler[2] - value_list[5]
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