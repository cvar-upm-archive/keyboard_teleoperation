#include "keyboard_teleoperation.hpp"

KeyboardTeleoperation::KeyboardTeleoperation() : as2::Node("keyboard_teleoperation") {}

void KeyboardTeleoperation::run(){
  switch(current_mode){
          case as2_msgs::msg::ControlMode::POSITION:
            move(2, 0);
            attron(COLOR_PAIR(2));printw("                        Teleoperation mode: Pose      "); attroff(COLOR_PAIR(2));
            clrtoeol(); refresh();  
          break;
          case as2_msgs::msg::ControlMode::ATTITUDE:
            move(2, 0);
            attron(COLOR_PAIR(3));printw("                        Teleoperation mode: Attitude      "); attroff(COLOR_PAIR(3));
            clrtoeol(); refresh();  
          break;
          case as2_msgs::msg::ControlMode::SPEED:
            move(2, 0);
            attron(COLOR_PAIR(4));printw("                        Teleoperation mode: Speed         "); attroff(COLOR_PAIR(4));
            clrtoeol(); refresh();  
          break;
          default:
            move(2, 0);
            printw("                        Teleoperation mode: Unknown          ");
            clrtoeol(); refresh();   
          break;      
    }

    move(16,0);
    printw("                        Last key pressed: ");
    //Read command
    command = getch();
    switch (command){
      case 't':  // Take off
        takeOff();
        printw("t       ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Take off           ");clrtoeol();
        //startQuadrotorControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+drone_id_namespace+"/quadrotor_pid_controller_process/start");
        //startQuadrotorControllerClientSrv.call(req);
        switch(current_mode){
          case as2_msgs::msg::ControlMode::POSITION:
              setControlMode(as2_msgs::msg::ControlMode::POSITION);
              //printoutPoseControls(); 
          break;
          case as2_msgs::msg::ControlMode::SPEED:
              setControlMode(as2_msgs::msg::ControlMode::SPEED);
              //printoutGroundSpeedControls();         
          break;
          case as2_msgs::msg::ControlMode::ATTITUDE:
              setControlMode(as2_msgs::msg::ControlMode::ATTITUDE);
              //printoutGroundSpeedControls();       
          break;
        }
        go();
        break;
      case 'y':  // Land
        hover();
        land();
        //startQuadrotorControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+drone_id_namespace+"/quadrotor_pid_controller_process/stop");
        //startQuadrotorControllerClientSrv.call(req);
        printw("y        ");clrtoeol(); 
        move(17, 0); 
        printw("                        Last command:     Land             ");clrtoeol();            
        break;
      case ' ':  // Emergency stop 
        emergencyStop();
        printw("space     ");clrtoeol(); 
        move(17, 0); 
        printw("                        Last command:     Emergency stop            ");clrtoeol();
        break;
      case 'h':  // Hover   
      {
        hover();
        printw("h      ");clrtoeol();
        move(17, 0); printw("                        Last command:     Keep hovering             ");clrtoeol();refresh();

          /*if (setControlMode(as2_msgs::msg::ControlMode::SPEED)){
              //clearSpeedReferences();
              //publishSpeedReference();
              //boost::this_thread::sleep(boost::posix_time::milliseconds(2000)); //2 seconds
              while(abs(self_speed_msg.twist.linear.x) >= 0.01 || abs(self_speed_msg.twist.linear.y) >= 0.01){
                //Waiting
              }   
              if (current_mode == POSE){
                setControlMode(aerostack_msgs::MotionControlMode::POSE);
              }      
              if (current_mode == ATTITUDE){
                setControlMode(aerostack_msgs::MotionControlMode::ATTITUDE);
              }      
              if (current_mode == GROUND_SPEED){
                setControlMode(aerostack_msgs::MotionControlMode::GROUND_SPEED);
              }   
              motion_reference_pose_msg.pose = self_localization_pose_msg.pose;                                             
              pose_reference_publ.publish(motion_reference_pose_msg);       
        }*/ 
        break;
      }
      case 'q':  // Move upwards
        go();
        /*motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z + CTE_ALTITUDE;
        motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
        pose_reference_publ.publish(motion_reference_pose_msg);*/
        printw("q      ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase altitude         ");clrtoeol();
        break;
      case 'a':  //Move downwards
        go();
          /*motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z - CTE_ALTITUDE;
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          pose_reference_publ.publish(motion_reference_pose_msg);*/
        printw("a        ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Decrease altitude         ");clrtoeol(); 
        break;         
      case 'z':  //(yaw) turn counter-clockwise
      {        
        go();       
        double yaw,r,p;
        /*toEulerianAngle(motion_reference_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(r, p, yaw + CTE_YAW);
        current_commands_yaw = yaw + CTE_YAW;
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);*/
        printw("z       ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Turn counter-clockwise        ");clrtoeol();  
        break;    
      }      
      case 'x':  // (yaw) turn clockwise
      {      
        go();       
        /*double yaw,r,p;
        toEulerianAngle(motion_reference_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(r, p, yaw - CTE_YAW);
        current_commands_yaw = yaw - CTE_YAW;
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);
        setControlMode(aerostack_msgs::MotionControlMode::GROUND_SPEED);
        clearSpeedReferences();
        speed_reference_msg.twist.angular.z = current_speed_ref.twist.angular.z - CTE_YAW;
        publishSpeedReference();*/
        
        printw("x      ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Turn clockwise          ");clrtoeol();
      }
        break;                 
      case ASCII_KEY_RIGHT:
        go();
        /*if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y - CTE_SPEED;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y - CTE_POSE;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = CTE_COMMANDS;
          current_commands_pitch = current_commands_pitch;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = RIGHT;
          attitude_publ.publish(msg_int);
        }*/   
        printw("\u2192            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase movement to the right        ");clrtoeol();  
        break;               
      case ASCII_KEY_LEFT:
        go();
        /*if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y + CTE_SPEED;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y + CTE_POSE;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = -CTE_COMMANDS;
          current_commands_pitch = current_commands_pitch;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = LEFT;
          attitude_publ.publish(msg_int);          
        }*/         
        printw("\u2190            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase movement to the left         ");clrtoeol();
        break;       
      case ASCII_KEY_DOWN:
        go();
        /*if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x - CTE_SPEED;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x - CTE_POSE;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = current_commands_roll;
          current_commands_pitch = CTE_COMMANDS;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = DOWN;
          attitude_publ.publish(msg_int);
        }*/       
        printw("\u2193     ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase backward      ");clrtoeol();
        break;        
      case ASCII_KEY_UP:
        go();
        /*if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x + CTE_SPEED;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x + CTE_POSE;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = current_commands_roll;
          current_commands_pitch = -CTE_COMMANDS;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = UP;
          attitude_publ.publish(msg_int);
        }*/    
        printw("\u2191    ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase forward            ");clrtoeol();
        break;         
    case 'r':
    {
      printw("r        ");clrtoeol();
      move(17, 0); 
      printw("                        Last command:     Reset orientation           ");clrtoeol(); 
        /*current_commands_yaw = 0;
        motion_reference_pose_msg.pose.orientation.w = 0;
        motion_reference_pose_msg.pose.orientation.x = 0;
        motion_reference_pose_msg.pose.orientation.y = 0;
        motion_reference_pose_msg.pose.orientation.z = 0;
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);*/   
    }
    break;
    case '1':
    {
      printw("1        ");clrtoeol(); 
      move(17, 0);   
      printw("                        Last command:     Speed mode               ");clrtoeol();    

      /*if (setControlMode(aerostack_msgs::MotionControlMode::GROUND_SPEED)){
          hover();
          clearSpeedReferences();
          publishSpeedReference();        
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
          pose_reference_publ.publish(motion_reference_pose_msg);         
          current_mode = GROUND_SPEED;
          printoutGroundSpeedControls();
        }*/
        current_mode = as2_msgs::msg::ControlMode::SPEED;
    }
    break;    
    case '2':
    {
      printw("2        ");clrtoeol();
      move(17, 0); 
      printw("                        Last command:     Pose mode         ");clrtoeol();        
      /*if (setControlMode(aerostack_msgs::MotionControlMode::POSE)){
        hover();
        clearSpeedReferences();
        publishSpeedReference();
        motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        pose_reference_publ.publish(motion_reference_pose_msg);             
        printoutPoseControls();   
        current_mode = POSE;    
      }*/
      current_mode = as2_msgs::msg::ControlMode::POSITION;
    }
    break;  
    case '3':
    {
      printw("3        ");clrtoeol();
      move(17, 0); 
      printw("                        Last command:     Attitude mode          ");clrtoeol();    
      /*if (setControlMode(aerostack_msgs::MotionControlMode::ATTITUDE)){
        hover();
        clearSpeedReferences();
        publishSpeedReference();
        motion_reference_pose_msg.pose.position = self_localization_pose_msg.pose.position;
        double yaw,r,p;
        toEulerianAngle(self_localization_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(0, 0, yaw);
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();        
        pose_reference_publ.publish(motion_reference_pose_msg);  
        current_mode = ATTITUDE;
        printoutAttitudeControls();
      }*/
      current_mode = as2_msgs::msg::ControlMode::ATTITUDE;
    }
    break;                   
    }
    refresh();
  }

void KeyboardTeleoperation::setupNode(){

}

void KeyboardTeleoperation::takeOff(){

}

void KeyboardTeleoperation::go(){

}

void KeyboardTeleoperation::hover(){

}

bool KeyboardTeleoperation::setControlMode(const int control_mode){
  return true;
}

void KeyboardTeleoperation::land(){

}

void KeyboardTeleoperation::emergencyStop(){

}


void KeyboardTeleoperation::printSpeedControls(){
  move(4,0);clrtoeol();
  printw(" BASIC MOTIONS                            SPEED CONTROL");
  move(5,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   t");attroff(COLOR_PAIR(5)); printw("      Take off              ");  
  attron(COLOR_PAIR(5));printw("    \u2191");attroff(COLOR_PAIR(5));printw("  Increase forward speed %.2f m/s  ",CTE_SPEED);
  
  move(6,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   y");attroff(COLOR_PAIR(5)); printw("      Land                  ");
  attron(COLOR_PAIR(5));printw("    \u2193");attroff(COLOR_PAIR(5));printw("  Increase backward speed %.2f m/s  ",CTE_SPEED);
  
  move(7,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   h");attroff(COLOR_PAIR(5)); printw("      Keep hovering         ");
  attron(COLOR_PAIR(5));printw("    \u2192");attroff(COLOR_PAIR(5));printw("  Increase speed to the right %.2f m/s  ",CTE_SPEED);  

  move(8,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   space ");attroff(COLOR_PAIR(5));printw(" Emergency stop        ");
  attron(COLOR_PAIR(5));printw("    \u2190");attroff(COLOR_PAIR(5));printw("  Increase speed to the left %.2f m/s  ",CTE_SPEED);
  
  move(9,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   r");attroff(COLOR_PAIR(5));printw("      Reset orientation    ");  
  
  move(10,0);clrtoeol();
  printw("                                   POSE CONTROL");
  
  move(11,0);clrtoeol();
  printw(" TELEOPERATION MODE SELECTION      ");
  attron(COLOR_PAIR(5));printw(" q");attroff(COLOR_PAIR(5));printw("  Increase altitude %.2f m           ",CTE_ALTITUDE);
  
  move(12,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   1");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(4));printw("      Ground speed mode");attroff(COLOR_PAIR(4));
  attron(COLOR_PAIR(5));printw("         a");attroff(COLOR_PAIR(5));printw("  Decrease altitude %.2f m            ",CTE_ALTITUDE);
  move(13,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   2");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(2));printw("      Pose mode            ");  attroff(COLOR_PAIR(2));
  attron(COLOR_PAIR(5));printw("     z");attroff(COLOR_PAIR(5));printw("  Turn counter-clockwise %.2f rad      ",CTE_YAW);
  
  move(14,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   3");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(3));printw("      Attitude mode         "); attroff(COLOR_PAIR(3));
  attron(COLOR_PAIR(5));printw("    x");attroff(COLOR_PAIR(5));printw("  Turn clockwise %.2f rad        ",CTE_YAW);
  move(15,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  refresh();
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn KeyboardTeleoperation::on_configure(
    const rclcpp_lifecycle::State& _state) {
  // Set subscriptions, publishers, services, actions, etc. here.
  setupNode();

  setlocale(LC_ALL, "");
  std::setlocale(LC_NUMERIC, "C");
  initscr();
  start_color();
  use_default_colors();  
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_BLUE, -1);
  init_pair(2, COLOR_GREEN, -1);
  init_pair(3, COLOR_CYAN, -1);
  init_pair(4, COLOR_RED, -1);
  init_pair(5, COLOR_YELLOW, -1);

  move(0,0);clrtoeol();
  printw("                           - KEYBOARD TELEOPERATION -                           ");
  move(3,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  printSpeedControls();
  return CallbackReturn::SUCCESS;
};

CallbackReturn KeyboardTeleoperation::on_deactivate(
    const rclcpp_lifecycle::State& _state) {
  // Clean up subscriptions, publishers, services, actions, etc. here.
  return CallbackReturn::SUCCESS;
};

CallbackReturn KeyboardTeleoperation::on_shutdown(
    const rclcpp_lifecycle::State& _state) {
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
};
