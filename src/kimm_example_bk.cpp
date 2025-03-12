#include "kimm_indydcp3/kimm_example.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {

    //Ros setting
    ros::init(argc, argv, "kimm_indydcp3");
    ros::NodeHandle n_node;

    dt_ = 0.01; //100Hz
    time_ = 0.0;
    ros::Rate loop_rate(1.0/dt_);    

    n_node.getParam("/load_gripper", load_gripper_);    

    // husky_ctrl_pub_.init(n_node, "cmd_vel", 4);    
    husky_ctrl_pub_ = n_node.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    joint_states_pub_ = n_node.advertise<sensor_msgs::JointState>("joint_states", 5);
    ft_calibrated_data_pub_ = n_node.advertise<geometry_msgs::Wrench>("ft_calibrated_data", 5);
    ft_transformed_data_pub_ = n_node.advertise<geometry_msgs::Wrench>("ft_transformed_data", 5);
    ft_link0_data_pub_ = n_node.advertise<geometry_msgs::Wrench>("ft_link0_data", 5);
    robotiq_ft_link0_data_pub_ = n_node.advertise<geometry_msgs::Wrench>("robotiq_ft_link0_data", 5);    
    teleop_joy_subs_ = n_node.subscribe("joy_teleop/joy", 1, &teleopjoyCallback);            
    robotq_ft_sensor_subs_ = n_node.subscribe("robotiq_ft_wrench", 1, &robotiqftsensorCallback);   
    robotiq_ft_reset_pub_ = n_node.advertise<std_msgs::Bool>("robotiq_ft_sensor_reset", 5);    

    // Indy7
    IndyDCP3 indy("192.168.10.9");    
    // example_get_robot_data(indy);

    //joint_state    
    joint_states_msg_.name = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};    
    joint_states_msg_.position.resize(6);
    joint_states_msg_.velocity.resize(6);

    // Robotiq 2f-140 gripper  
    isgrasp_ = false;     
    cout << load_gripper_ << endl; 
    if(load_gripper_){
        gripper_robotiq_ = new robotiq_2f_gripper_control::RobotiqActionClient("/command_robotiq_action", true);      
        robotiq_state_subs_ = n_node.subscribe( "/robotiq/joint_states", 1, &robotiqstateCallback);                
    }

    // keyboard event, this code begins from here  
    std::thread mode_change_thread_;
    mode_change_thread_ = std::thread(&modeChangeReaderProc);

    // joystick
    isbutton_pushed_.resize(14); //ps5 joystick
    isbutton_pushed_.setZero();                   

    indy_q_.setZero();
    indy_dq_.setZero();
    is_log_ = false;
    ft_calibration_.setZero();
    ft_calibration_temp_.setZero();
    is_cal_ = false;
    calibration_cnt_ = 0;
    R_link0_tcp_.setZero();
    R_link0_ft_.setZero();
    R_tcp_ft_sensor();
    can_i_do_mobile_teaching_ = false;

    // example_get_device_info(indy);
    // example_get_int_variables(indy);        

    // loop
    while (ros::ok()) {
        // command
        keyboard_event(indy);

        // pub
        call_IndyData(indy);     
        
        if(mobile_teaching_trigger_) {
            if(can_i_do_mobile_teaching_)  mobile_teaching();
            else cout << "please calibrate ft_sensor data" << endl;
        }        

        //robotiq FT sensor update for admittance control, it is not realtime command!
        // example_set_custom_control_gain_high(indy);


        ros::spinOnce();
        loop_rate.sleep();
    }//while

    example_get_stop_state(indy);            
    return 0;
}

void modeChangeReaderProc(){  

  while (!quit_all_proc_)
  {
    char key = getchar();
    key = tolower(key);
    calculation_mutex_.lock();

    int msg = 0;
    switch (key){
        //------------- basic motion -------------------------------------------------//        
        case 'h': //home       
            msg_ = 1;             
            std::cout << " " << std::endl;
            std::cout << "home position" << std::endl;
            std::cout << " " << std::endl;          
            break;         
        case 'u': //zero
            msg_ = 2;             
            std::cout << " " << std::endl;
            std::cout << "zero position" << std::endl;
            std::cout << " " << std::endl;          
            break;         
        case 'r': //grasp ready
            msg_ = 3;             
            std::cout << " " << std::endl;
            std::cout << "ready for grasp" << std::endl;
            std::cout << " " << std::endl;          
            break;  
        case 'i': //move pose
            msg_ = 4;             
            std::cout << " " << std::endl;
            std::cout << "move pose" << std::endl;
            std::cout << " " << std::endl;          
            break;  
        case 't': //forward jog
            msg_ = 50;             
            std::cout << " " << std::endl;
            std::cout << "forward jog" << std::endl;
            std::cout << " " << std::endl;          
            break;  
        case 'y': //backward jog
            msg_ = 51;             
            std::cout << " " << std::endl;
            std::cout << "backward jog" << std::endl;
            std::cout << " " << std::endl;          
            break;  
        case 'f': //left jog
            msg_ = 52;             
            std::cout << " " << std::endl;
            std::cout << "left jog" << std::endl;
            std::cout << " " << std::endl;          
            break;  
        case 'g': //right jog
            msg_ = 53;             
            std::cout << " " << std::endl;
            std::cout << "right jog" << std::endl;
            std::cout << " " << std::endl;          
            break; 
        case 'v': //up jog
            msg_ = 54;             
            std::cout << " " << std::endl;
            std::cout << "up jog" << std::endl;
            std::cout << " " << std::endl;          
            break; 
        case 'b': //down jog
            msg_ = 55;             
            std::cout << " " << std::endl;
            std::cout << "down jog" << std::endl;
            std::cout << " " << std::endl;          
            break; 
        
        case 'q': //actvie sdk
            msg_ = 10;             
            std::cout << " " << std::endl;
            std::cout << "actvie sdk" << std::endl;
            std::cout << " " << std::endl;          
            break;
        case 'a': //get custom control mode
            msg_ = 11;             
            std::cout << " " << std::endl;
            std::cout << "get custom control mode" << std::endl;
            std::cout << " " << std::endl;          
            break;
        case 'w': //set custom control mode
            msg_ = 12;             
            std::cout << " " << std::endl;
            std::cout << "set custom control mode" << std::endl;
            std::cout << " " << std::endl;          
            break;            
        case 's': //set basic control mode
            msg_ = 13;             
            std::cout << " " << std::endl;
            std::cout << "set basic control mode" << std::endl;
            std::cout << " " << std::endl;          
            break;            
        case 'e': //get custom gain
            msg_ = 14;             
            std::cout << " " << std::endl;
            std::cout << "get custom gain" << std::endl;
            std::cout << " " << std::endl;          
            break;
        case 'd': //set custom gain low
            msg_ = 15;             
            std::cout << " " << std::endl;
            std::cout << "set custom gain low" << std::endl;
            std::cout << " " << std::endl;          
            break;
        case 'c': //set custom gain high
            msg_ = 16;             
            std::cout << " " << std::endl;
            std::cout << "set custom gain high" << std::endl;
            std::cout << " " << std::endl;          
            break;

        case 'o': //enable direct teaching
            msg_ = 20;             
            std::cout << " " << std::endl;
            std::cout << "enable direct teaching" << std::endl;
            std::cout << " " << std::endl;          
            break;
        case 'l': //disable direct teaching
            msg_ = 21;             
            std::cout << " " << std::endl;
            std::cout << "disable direct teaching" << std::endl;
            std::cout << " " << std::endl;          
            break;        
        case 'p': //recover
            msg_ = 30;             
            std::cout << " " << std::endl;
            std::cout << "recover" << std::endl;
            std::cout << " " << std::endl;          
            break;        

        case 'm': //get ft sensor data
            msg_ = 40;             
            std::cout << " " << std::endl;
            std::cout << "get ft sensor data" << std::endl;
            std::cout << " " << std::endl;          
            break;     
        case 'n': //ft sensor data calibration
            msg_ = 41;             
            std::cout << " " << std::endl;
            std::cout << "ft sensor data calibration" << std::endl;
            std::cout << " " << std::endl;          
            break;     
        case ',': //get ft sensor config
            msg_ = 42;             
            std::cout << " " << std::endl;
            std::cout << "get ft sensor config" << std::endl;
            std::cout << " " << std::endl;          
            break;     
        case '.': //set ft sensor config
            msg_ = 43;             
            std::cout << " " << std::endl;
            std::cout << "set ft sensor config" << std::endl;
            std::cout << " " << std::endl;          
            break;     

        case '9': //log start
            msg_ = 80;             
            std::cout << " " << std::endl;
            std::cout << "log start" << std::endl;
            std::cout << " " << std::endl;          
            break;         
        case '0': //log end
            msg_ = 81;             
            std::cout << " " << std::endl;
            std::cout << "log end" << std::endl;
            std::cout << " " << std::endl;          
            break; 

        case '1': //stop
            msg_ = 99;             
            std::cout << " " << std::endl;
            std::cout << "stop" << std::endl;
            std::cout << " " << std::endl;          
            break;                 
        case '2': //mobile teaching
            if (mobile_teaching_trigger_) {            
                std::cout << " " << std::endl;
                std::cout << "end mobile teaching" << std::endl;
                std::cout << " " << std::endl;     
                mobile_teaching_trigger_ = false;
            }       
            else {     
                std::cout << " " << std::endl;
                std::cout << "start mobile teaching" << std::endl;
                std::cout << " " << std::endl;     
                mobile_teaching_trigger_ = true;
            }
            break;                 
                
        case 'z': //gripper
            if (isgrasp_){
                cout << "Release hand" << endl;
                isgrasp_ = false;
                gripper_robotiq_->open(false);
            }
            else{
                cout << "Grasp object" << endl;
                isgrasp_ = true; 
                gripper_robotiq_->close(0.5, 220, false);                
            }
            break;
        case '\n':
            break;
        case '\r':
            break;
        default:
            break;
    }        
    calculation_mutex_.unlock();
  }  
}

void keyboard_event(IndyDCP3& indy){    
    if(msg_ == 1){ //h
        //home pos
        std::vector<float> j_pos_2 = {0.0, 0.0, -90.0, 0.0, -90.0, 0.0};
        indy.movej(j_pos_2, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);    
        // indy.wait_progress(100);
        
        msg_ = 0;
    }
    else if(msg_ == 2){ //u
        //zero pos
        std::vector<float> j_pos_1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        indy.movej(j_pos_1, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE);
        // indy.wait_progress(100);

        msg_ = 0;
    }   
    else if(msg_ ==3){ //r
        //grasp ready pos
        std::vector<float> j_pos_3 = {0.0, -45.0, -135.0, 0.0, 90.0, 0.0};
        indy.movej(j_pos_3, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);            
        // indy.wait_progress(100);
    
        msg_ = 0;
    }
    else if(msg_ ==4){ //i
        //moveo pos
        std::vector<float> j_pos_4 = {0.0, 90.0, -90.0, 0.0, -90.0, 0.0};
        indy.movej(j_pos_4, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);            
        // indy.wait_progress(100);
    
        msg_ = 0;
    }                                                
    else if(msg_ ==50){ //t
        //forward jog
        std::array<float, 6> t_pos = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);    
        // indy.wait_progress(100);
    
        msg_ = 0;
    }  
    else if(msg_ ==51){ //y
        //backward jog
        std::array<float, 6> t_pos = {-100.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);    
        // indy.wait_progress(100);
    
        msg_ = 0;
    }  
    else if(msg_ ==52){ //f
        //left jog
        std::array<float, 6> t_pos = {0.0, 100.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);    
        // indy.wait_progress(100);
    
        msg_ = 0;
    }  
    else if(msg_ ==53){ //g
        //right jog
        std::array<float, 6> t_pos = {0.0, -100.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);    
        // indy.wait_progress(100);
    
        msg_ = 0;
    }  
    else if(msg_ ==54){ //v
        //up jog
        std::array<float, 6> t_pos = {0.0, 0.0, 100.0, 0.0, 0.0, 0.0}; //[mm]
        indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);    
        // indy.wait_progress(100);
    
        msg_ = 0;
    }  
    else if(msg_ ==55){ //b
        //down jog
        std::array<float, 6> t_pos = {0.0, 0.0, -100.0, 0.0, 0.0, 0.0}; //[mm]
        indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);    
        // indy.wait_progress(100);
    
        msg_ = 0;
    }  
    
    else if(msg_ ==10){ //q
        example_activate_sdk(indy);
    
        msg_ = 0;
    }
    else if(msg_ ==11){ //a
        example_get_custom_control_mode(indy);        
        
        msg_ = 0;
    }       
    else if(msg_ ==12){ //w
        example_set_custom_control_mode(indy, 1);
        
        msg_ = 0;
    }
    else if(msg_ ==13){ //s
        example_set_custom_control_mode(indy, 0);
        
        msg_ = 0;
    }
    else if(msg_ ==14){ //e
        example_get_custom_control_gain(indy);
        
        msg_ = 0;
    }
    else if(msg_ ==15){ //d
        example_set_custom_control_gain_low(indy);
        
        msg_ = 0;
    }
    else if(msg_ ==16){ //c
        example_set_custom_control_gain_high(indy);
        // example_set_custom_control_gain_high_impedance(indy);
        
        msg_ = 0;
    }
    
    else if(msg_ ==20){ //o
        example_enable_direct_teaching(indy, true);
        
        msg_ = 0;
    }
    else if(msg_ ==21){ //l
        example_enable_direct_teaching(indy, false);
        
        msg_ = 0;
    }
    else if(msg_ ==30){ //p
        example_recover_robot(indy);
        
        msg_ = 0;
    }

    else if(msg_ ==40){ //m
        example_get_ft_sensor_data(indy);
        // tf_print(indy);
        example_get_tool_properties(indy);
        
        msg_ = 0;
    }
    else if(msg_ ==41){ //n
        ft_sensor_calibration(indy);
        
        std_msgs::Bool is_robotiq_ft_reset;
        is_robotiq_ft_reset.data = true;
        robotiq_ft_reset_pub_.publish(is_robotiq_ft_reset);
        
        msg_ = 0;
    }
    else if(msg_ ==42){ //,
        example_get_ft_sensor_config(indy);                
        
        msg_ = 0;
    }
    else if(msg_ ==43){ //,
        example_set_ft_sensor_config(indy);                
        
        msg_ = 0;
    }

    else if(msg_ ==80){ //9
        example_start_log(indy);
        // data_log_start(indy);
        
        msg_ = 0;

        // # Appending values to lists
        // self.record["t"].append(float(line[0]))
        // self.record["cycle_time"].append(float(line[1]))
        // self.record["period"].append(float(line[2]))

        // for i in range(1, 7):  # For each joint (1 through 6)
        //     # Adjusted indexing to correctly align with the CSV structure
        //     self.record[f"q{i}"].append(float(line[2 + i]))  # Starts from index 3 (4th column) for the first joint
        //     self.record[f"qdot{i}"].append(float(line[8 + i]))  # Continues sequentially, adjusting the starting index for each parameter
        //     self.record[f"qddot{i}"].append(float(line[14 + i]))
        //     self.record[f"tau{i}"].append(float(line[20 + i]))
        //     self.record[f"tauact{i}"].append(float(line[26 + i]))
        //     self.record[f"qd{i}"].append(float(line[32 + i]))
        //     self.record[f"qdotd{i}"].append(float(line[38 + i]))
        //     self.record[f"qddotd{i}"].append(float(line[44 + i]))
        //     self.record[f"p{i}"].append(float(line[50 + i]))
        //     self.record[f"pd{i}"].append(float(line[56 + i]))
        //     self.record[f"fact{i}"].append(float(line[62 + i]))
        //     self.record[f"fdes{i}"].append(-float(line[68 + i]))  # Note the negation for fdes
        //     self.record[f"tauidyn{i}"].append(float(line[74 + i]))
        //     self.record[f"tauref{i}"].append(float(line[80 + i]))
        //     self.record[f"taugrav{i}"].append(float(line[86 + i]))
        //     self.record[f"taufric{i}"].append(float(line[92 + i]))
        //     self.record[f"tauext{i}"].append(float(line[98 + i]))
        //     self.record[f"tauJts{i}"].append(float(line[104 + i]))
        //     self.record[f"tauJtsRaw1{i}"].append(float(line[110 + i]))
        //     self.record[f"tauJtsRaw2{i}"].append(float(line[116 + i]))


        //     # Compute Joint and Task Position errors
        //     self.record[f"qe{i}"].append(self.record[f"qd{i}"][-1] - self.record[f"q{i}"][-1])
        //     self.record[f"qdote{i}"].append(self.record[f"qdotd{i}"][-1] - self.record[f"qdot{i}"][-1])
        //     self.record[f"pe{i}"].append(self.record[f"pd{i}"][-1] - self.record[f"p{i}"][-1])
        //     self.record[f"fe{i}"].append(self.record[f"fdes{i}"][-1] - self.record[f"fact{i}"][-1])
    }
    else if(msg_ ==81){ //0
        example_end_log(indy);
        // data_log_end(indy);
        
        msg_ = 0;
    }


    else if(msg_ ==99){ //1
        example_stop_robot_motion(indy);    
        
        msg_ = 0;
    }                                    
}

void tf_print(IndyDCP3& indy){        

    bool is_success;
    Nrmk::IndyFramework::ControlData control_data;
    is_success = indy.get_robot_data(control_data);    

    Eigen::AngleAxisd rollAngle(control_data.p(3)*3.14/180.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(control_data.p(4)*0.0/180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(control_data.p(5)*1.57/180.0, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q_link0_tcp = yawAngle * pitchAngle * rollAngle;    
    Eigen::Matrix3d R_link0_tcp = q_link0_tcp.matrix();            
    cout << R_link0_tcp << endl;

    // cout << R_link0_ft_ << endl;

    // bool is_success;
    // Nrmk::IndyFramework::ControlData control_data;
    // is_success = indy.get_robot_data(control_data);    

    // //rotation link0 to tcp
    // Eigen::AngleAxisd rollAngle(control_data.p(3)*3.14/180.0, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitchAngle(control_data.p(4)*3.14/180.0, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yawAngle(control_data.p(5)*3.14/180.0, Eigen::Vector3d::UnitZ());

    // Eigen::Quaternion<double> q_link0_tcp = yawAngle * pitchAngle * rollAngle;    
    // Eigen::Matrix3d R_link0_tcp = q_link0_tcp.matrix();            
    // cout << R_link0_tcp << endl;
    
    // //rotation from tcp to ft_sensor
    // Eigen::AngleAxisd rollAngle_tcp_ft(0.0, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitchAngle_tcp_ft(0.0, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yawAngle_tcp_ft(-90*3.14/180.0, Eigen::Vector3d::UnitZ());
    
    // Eigen::Quaternion<double> q_tcp_ft = yawAngle_tcp_ft * pitchAngle_tcp_ft * rollAngle_tcp_ft;
    // Eigen::Matrix3d R_tcp_ft = q_tcp_ft.matrix();
    // cout << R_tcp_ft << endl;

    // //rotation from link0 to ft_sensor
    // Eigen::Matrix3d R_link0_ft;
    // R_link0_ft = R_link0_tcp * R_tcp_ft;
    // cout << R_link0_ft << endl;
    // cout << (q_link0_tcp * q_tcp_ft).x() << (q_link0_tcp * q_tcp_ft).y() << (q_link0_tcp * q_tcp_ft).z() << (q_link0_tcp * q_tcp_ft).w() << endl;
    // so, use as ne_ft_sensor_data = R_link0_ft * ft_sensor_data to meet the global direction!!!!!!


    // tf::TransformListener listener_;
    // tf::StampedTransform transform;        
    // try {        
    //     // if (listener_.waitForTransform("/link0", "/link6", ros::Time(0), ros::Duration(4.0))) {
    //     //     listener_.lookupTransform("/link0", "/link6", ros::Time(0), transform);
    //     // } 
    //     // else {
    //     //     ROS_ERROR("Failed to read transform");
    //     // }        
    //     listener_.lookupTransform("/link0", "/link6", ros::Time(0), transform);
    // } 
    // catch (tf::TransformException& ex) {
    //     ROS_ERROR("TransformListener: %s", ex.what());        
    // }
    // tf::transformTFToEigen(transform, T_base_ft_);
    // cout << transform.getOrigin().x() << transform.getOrigin().y() << transform.getOrigin().z() << endl;
    // // cout << transform.getRotation() << endl;

    // cout << T_base_ft_.matrix() << endl;        
}


void data_log_start(IndyDCP3& indy){    
    fout_.open("/home/kimm/kimm_catkin_ws/src/kimm_indydcp3/data/data.txt");
    is_log_ = true;
}

void data_log_end(IndyDCP3& indy){    
    is_log_ = false;
    fout_.close();          
}

void ft_sensor_calibration(IndyDCP3& indy) {    
    is_cal_ = true;  
}    

void robotiqstateCallback(const sensor_msgs::JointStateConstPtr &msg) {
    //
}
void robotiqftsensorCallback(const geometry_msgs::WrenchStamped &msg){    
    robotiq_ft_link0_data_.head(3) = R_link0_ft_ * Eigen::Vector3d(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    robotiq_ft_link0_data_.tail(3) = R_link0_ft_ * Eigen::Vector3d(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);

    robotiq_ft_link0_filtered_data_for_mobile_ = lowpassFilter( dt_,  robotiq_ft_link0_data_,  robotiq_ft_link0_filtered_data_for_mobile_,  0.5); //in Hz, Vector7d}
    
    robotiq_ft_link0_data_for_mobile_msg_.force.x = robotiq_ft_link0_filtered_data_for_mobile_(0);
    robotiq_ft_link0_data_for_mobile_msg_.force.y = robotiq_ft_link0_filtered_data_for_mobile_(1);
    robotiq_ft_link0_data_for_mobile_msg_.force.z = robotiq_ft_link0_filtered_data_for_mobile_(2);
    robotiq_ft_link0_data_for_mobile_msg_.torque.x = robotiq_ft_link0_filtered_data_for_mobile_(3);
    robotiq_ft_link0_data_for_mobile_msg_.torque.y = robotiq_ft_link0_filtered_data_for_mobile_(4);
    robotiq_ft_link0_data_for_mobile_msg_.torque.z = robotiq_ft_link0_filtered_data_for_mobile_(5);

    robotiq_ft_link0_data_pub_.publish(robotiq_ft_link0_data_for_mobile_msg_);
}

void teleopjoyCallback(const sensor_msgs::JoyConstPtr &joy_msg) {      
  if(joy_msg->buttons[5]){ //R1 button pushed 
    
    //home, h
    if (joy_msg->buttons[0])  { //cross      button pushed
      if (!isbutton_pushed_(0)) {
        msg_ = 1;     
        cout << " " << endl;
        cout << "home position" << endl;
        cout << " " << endl;        

        isbutton_pushed_(0) = 1;
      }
    }
    else isbutton_pushed_(0) = 0;
  }
}

void mobile_teaching() {
    double teaching_v;
    double teaching_w;

    //  qvel = D^(-1) * torque_base : robotiq FT300s
    // teaching_v = robotiq_ft_link0_filtered_data_for_mobile_(0) * 0.005; 
    // teaching_w = robotiq_ft_link0_filtered_data_for_mobile_(1) * 0.005;

    //  qvel = D^(-1) * torque_base : aidin AFT200-D80
    teaching_v = ft_link0_filtered_data_for_mobile_(0) * 0.005; 
    // teaching_w = ft_link0_filtered_data_for_mobile_(1) * 0.005;
    teaching_w = ft_link0_filtered_data_for_mobile_(5) * 0.02;

    //for ready pos 241030
    // teaching_v = compensated_value_ * 0.008; 
    // teaching_w = ft_link0_filtered_data_for_mobile_(5) * 0.002 / 0.02;

    double thes_vel_x = 0.4;
    double thes_vel_w = 0.2;
    if (teaching_v > thes_vel_x)
      thes_vel_x = thes_vel_x;
    else if (teaching_v < -thes_vel_x)
      thes_vel_x = -thes_vel_x;
    if (teaching_w > thes_vel_w)
      teaching_w = thes_vel_w;
    else if (teaching_w < -thes_vel_w)
      teaching_w = -thes_vel_w;  
    
    if (abs(teaching_v) < 0.01)
      teaching_v = 0.0;
    if (abs(teaching_w) < 0.01)
      teaching_w = 0.0;

    // if (husky_ctrl_pub_.trylock())
    // {
    // husky_ctrl_pub_.msg_.linear.x = teaching_v;
    // husky_ctrl_pub_.msg_.angular.z = teaching_w;
    // husky_ctrl_pub_.unlockAndPublish();
    // }    
    
    husky_ctrl_pub_msg_.linear.x = teaching_v;
    husky_ctrl_pub_msg_.angular.z = teaching_w;
    husky_ctrl_pub_.publish(husky_ctrl_pub_msg_);
}

void call_IndyData(IndyDCP3& indy){        
    bool is_success;
    Nrmk::IndyFramework::ControlData control_data;

    // ControlData
    is_success = indy.get_robot_data(control_data);            
    if (is_success){        
        joint_states_msg_.header.stamp = ros::Time::now();        
        for (int i=0; i<6; i++){
            joint_states_msg_.position[i] = deg2rad(control_data.q(i));
            joint_states_msg_.velocity[i] = deg2rad(control_data.qdot(i));    

            indy_q_(i) = deg2rad(control_data.q(i));
            indy_dq_(i) = deg2rad(control_data.qdot(i));

            if(is_log_){
                fout_ << indy_q_.transpose() << "\n";
            }
        }                   
        //indy joint state publish
        joint_states_pub_.publish(joint_states_msg_);

        // link0 TransformedFTSensorData    
        Eigen::AngleAxisd rollAngle(control_data.p(3)*3.14/180.0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(control_data.p(4)*3.14/180.0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(control_data.p(5)*3.14/180.0, Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> q_link0_tcp = yawAngle * pitchAngle * rollAngle;    
        R_link0_tcp_ = q_link0_tcp.matrix();                                
        R_link0_ft_ = R_link0_tcp_ * R_tcp_ft_;    
    }
    else{
        std::cout << "Failed to get_robot_data" << std::endl;
    }        

    // FTSensorData
    Nrmk::IndyFramework::FTSensorData ft_sensor_data;
    is_success = indy.get_ft_sensor_data(ft_sensor_data);
    if (is_success) {                                        
        ft_data_ << ft_sensor_data.ft_fx(), ft_sensor_data.ft_fy(), ft_sensor_data.ft_fz(), ft_sensor_data.ft_tx(), ft_sensor_data.ft_ty(), ft_sensor_data.ft_tz();
        ft_calibrated_data_ = ft_data_ - ft_calibration_;

        if(is_cal_){
            if(calibration_cnt_ < 100) {
                ft_calibration_temp_ += ft_data_ * 0.01;
                calibration_cnt_ += 1;
            }
            else{ //calibration end
                is_cal_ = false;
                calibration_cnt_ = 0;
                can_i_do_mobile_teaching_ = true;
                cout << "end start calibration" << endl;   
                ft_calibration_ = ft_calibration_temp_;
                ft_calibration_temp_.setZero();  
                cout << "calibration data" << ft_calibration_.transpose() << endl; 
            }
        }

        ft_calibrated_data_msg_.force.x = ft_calibrated_data_(0);
        ft_calibrated_data_msg_.force.y = ft_calibrated_data_(1);
        ft_calibrated_data_msg_.force.z = ft_calibrated_data_(2);
        ft_calibrated_data_msg_.torque.x = ft_calibrated_data_(3);
        ft_calibrated_data_msg_.torque.y = ft_calibrated_data_(4);
        ft_calibrated_data_msg_.torque.z = ft_calibrated_data_(5);

        ft_calibrated_data_pub_.publish(ft_calibrated_data_msg_);
       
        ft_link0_data_.head(3) = R_link0_ft_ * Eigen::Vector3d(ft_calibrated_data_(0), ft_calibrated_data_(1), ft_calibrated_data_(2));
        ft_link0_data_.tail(3) = R_link0_ft_ * Eigen::Vector3d(ft_calibrated_data_(3), ft_calibrated_data_(4), ft_calibrated_data_(5));

        ft_link0_filtered_data_for_mobile_ = lowpassFilter( dt_,  ft_link0_data_,  ft_link0_filtered_data_for_mobile_,  0.5); //in Hz, Vector7d

        //compensation        
        // if (ft_link0_filtered_data_for_mobile_(5)>=0) compensated_value_ = ft_link0_filtered_data_for_mobile_(0) - ft_link0_filtered_data_for_mobile_(5) / 0.035;
        // else                                          compensated_value_ = ft_link0_filtered_data_for_mobile_(0) + ft_link0_filtered_data_for_mobile_(5) / 0.018;
        
        // ft_link0_data_for_mobile_msg_.force.x = compensated_value_; //for ready pos 241030
        ft_link0_data_for_mobile_msg_.force.x = ft_link0_filtered_data_for_mobile_(0);
        ft_link0_data_for_mobile_msg_.force.y = ft_link0_filtered_data_for_mobile_(1);
        ft_link0_data_for_mobile_msg_.force.z = ft_link0_filtered_data_for_mobile_(2);
        ft_link0_data_for_mobile_msg_.torque.x = ft_link0_filtered_data_for_mobile_(3);
        ft_link0_data_for_mobile_msg_.torque.y = ft_link0_filtered_data_for_mobile_(4);
        ft_link0_data_for_mobile_msg_.torque.z = ft_link0_filtered_data_for_mobile_(5);

        ft_link0_data_pub_.publish(ft_link0_data_for_mobile_msg_);
    } 
    else{
        std::cerr << "GetFTSensorData RPC failed." << std::endl;
    }

    // TransformedFTSensorData
    // Nrmk::IndyFramework::TransformedFTSensorData ft_sensor_transformed_data;
    // is_success = indy.get_transformed_ft_sensor_data(ft_sensor_transformed_data);    
    // if (is_success) {
    //     ft_transformed_data_ << ft_sensor_transformed_data.ft_fx(), ft_sensor_transformed_data.ft_fy(), ft_sensor_transformed_data.ft_fz(), ft_sensor_transformed_data.ft_tx(), ft_sensor_transformed_data.ft_ty(), ft_sensor_transformed_data.ft_tz();

    //     ft_transformed_data_msg_.force.x = ft_transformed_data_(0);
    //     ft_transformed_data_msg_.force.y = ft_transformed_data_(1);
    //     ft_transformed_data_msg_.force.z = ft_transformed_data_(2);
    //     ft_transformed_data_msg_.torque.x = ft_transformed_data_(3);
    //     ft_transformed_data_msg_.torque.y = ft_transformed_data_(4);
    //     ft_transformed_data_msg_.torque.z = ft_transformed_data_(5);

    //     ft_transformed_data_pub_.publish(ft_transformed_data_msg_);
        
    // } else {
    //     std::cerr << "Failed to retrieve Transformed FT Sensor Data." << std::endl;
    // }    
}

void R_tcp_ft_sensor() {
    //rotation from tcp to ft_sensor
    Eigen::AngleAxisd rollAngle_tcp_ft(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle_tcp_ft(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle_tcp_ft(-90*3.14/180.0, Eigen::Vector3d::UnitZ()); //-90degree w.r.t. z-axis
    
    Eigen::Quaternion<double> q_tcp_ft = yawAngle_tcp_ft * pitchAngle_tcp_ft * rollAngle_tcp_ft;
    R_tcp_ft_ = q_tcp_ft.matrix();    
}

double deg2rad(const double degrees) {
    double rad = degrees * M_PI / 180.0;        
    return rad;    
}

void example_get_robot_data(IndyDCP3& indy) {
    bool is_success;
    Nrmk::IndyFramework::ControlData control_data;
    is_success = indy.get_robot_data(control_data);
    if (is_success){
        std::cout << "Control data:" << std::endl;
        for (int i = 0; i < control_data.q_size(); i++)
            std::cout << "q" << i << ": " << control_data.q(i) << std::endl;
        for (int i = 0; i < control_data.qdot_size(); i++)
            std::cout << "qdot" << i << ": " << control_data.qdot(i) << std::endl;

        for (int i = 0; i < control_data.p_size(); i++)
            std::cout << "p" << i << ": " << control_data.p(i) << std::endl;
        for (int i = 0; i < control_data.pdot_size(); i++)
            std::cout << "pdot" << i << ": " << control_data.pdot(i) << std::endl;
    }
    else{
        std::cout << "Failed to get_robot_data" << std::endl;
    }
}

void example_get_robot_control_data(IndyDCP3& indy) {
    bool is_success;
    Nrmk::IndyFramework::ControlData control_data;
    is_success = indy.get_control_data(control_data);
    if (is_success){
        std::cout << "running_hours: " << control_data.running_hours() << std::endl;
        std::cout << "running_mins: " << control_data.running_mins() << std::endl;
        std::cout << "running_secs: " << control_data.running_secs() << std::endl;
        std::cout << "op_state: " << control_data.op_state() << std::endl;
        std::cout << "sim_mode: " << control_data.sim_mode() << std::endl;
    }
    else{
        std::cout << "Failed to get_control_data" << std::endl;
    }
}

void example_get_digital_inputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::DigitalList di_data;
    bool is_success = indy.get_di(di_data);
    if (is_success) {
        std::cout << "GetDI RPC succeeded." << std::endl;
        for (int i = 0; i < di_data.signals_size(); i++) {
            const auto& signal = di_data.signals(i);
            std::cout << "Address: " << signal.address() << ", State: " << signal.state() << std::endl;
        }
    } else {
        std::cerr << "GetDI RPC failed." << std::endl;
    }
}

void example_get_digital_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::DigitalList do_data;
    bool is_success = indy.get_do(do_data);
    if (is_success) {
        std::cout << "GetDO RPC succeeded." << std::endl;
        for (int i = 0; i < do_data.signals_size(); i++) {
            const auto& signal = do_data.signals(i);
            std::cout << "Address: " << signal.address() << ", State: " << signal.state() << std::endl;
        }
    } else {
        std::cerr << "GetDO RPC failed." << std::endl;
    }
}

void example_set_digital_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::DigitalList do_signal_list;
    
    auto *do_signal1 = do_signal_list.add_signals();
    do_signal1->set_address(1);
    do_signal1->set_state(DigitalState::OFF_STATE);
    
    auto *do_signal5 = do_signal_list.add_signals();
    do_signal5->set_address(5);
    do_signal5->set_state(DigitalState::ON_STATE);

    auto *do_signal9 = do_signal_list.add_signals();
    do_signal9->set_address(9);
    do_signal9->set_state(DigitalState::ON_STATE);

    bool is_success = indy.set_do(do_signal_list);
    if (is_success) {
        std::cout << "SetDO RPC succeeded." << std::endl;
    } else {
        std::cerr << "SetDO RPC failed." << std::endl;
    }
}

void example_get_analog_inputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::AnalogList ai_data;
    bool is_success = indy.get_ai(ai_data);

    if (is_success) {
        std::cout << "GetAI RPC succeeded." << std::endl;
        for (int i = 0; i < ai_data.signals_size(); i++) {
            const auto& signal = ai_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetAI RPC failed." << std::endl;
    }
}

void example_get_analog_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::AnalogList ao_data;
    bool is_success = indy.get_ao(ao_data);

    if (is_success) {
        std::cout << "GetAO RPC succeeded." << std::endl;
        for (int i = 0; i < ao_data.signals_size(); ++i) {
            const auto& signal = ao_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetAO RPC failed." << std::endl;
    }
}

void example_set_analog_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::AnalogList ao_signal_list;

    auto* ao_signal1 = ao_signal_list.add_signals();
    ao_signal1->set_address(1);
    ao_signal1->set_voltage(1000);

    auto* ao_signal2 = ao_signal_list.add_signals();
    ao_signal2->set_address(2);
    ao_signal2->set_voltage(2000);

    bool is_success = indy.set_ao(ao_signal_list);

    if (is_success) {
        std::cout << "SetAO RPC succeeded." << std::endl;
    } else {
        std::cerr << "SetAO RPC failed." << std::endl;
    }
}

void example_get_endtool_digital_inputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::EndtoolSignalList endtool_di_data;
    bool is_success = indy.get_endtool_di(endtool_di_data);

    if (is_success) {
        std::cout << "GetEndDI RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_di_data.signals_size(); i++) {
            const auto& signal = endtool_di_data.signals(i);
            std::cout << "Port: " << signal.port() << std::endl;
            for (const auto& state : signal.states()) {
                std::cout << "State: " << state << std::endl;
            }
        }
    } else {
        std::cerr << "GetEndDI RPC failed." << std::endl;
    }
}

void example_get_endtool_digital_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::EndtoolSignalList endtool_do_data;
    bool is_success = indy.get_endtool_do(endtool_do_data);

    if (is_success) {
        std::cout << "GetEndDO RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_do_data.signals_size(); i++) {
            const auto& signal = endtool_do_data.signals(i);
            std::cout << "Port: " << signal.port() << std::endl;
            for (const auto& state : signal.states()) {
                std::cout << "State: " << state << std::endl;
            }
        }
    } else {
        std::cerr << "GetEndDO RPC failed." << std::endl;
    }
}

void example_set_endtool_digital_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::EndtoolSignalList end_do_signal_list;

    auto* end_signal1 = end_do_signal_list.add_signals();
    end_signal1->set_port("C");
    end_signal1->add_states(EndtoolState::UNUSED);

    bool is_success = indy.set_endtool_do(end_do_signal_list);
    if (is_success) {
        std::cout << "SetEndDO RPC succeeded." << std::endl;
    } else {
        std::cerr << "SetEndDO RPC failed." << std::endl;
    }
}

void example_get_endtool_analog_inputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::AnalogList endtool_ai_data;
    bool is_success = indy.get_endtool_ai(endtool_ai_data);

    if (is_success) {
        std::cout << "GetEndAI RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_ai_data.signals_size(); i++) {
            const auto& signal = endtool_ai_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetEndAI RPC failed." << std::endl;
    }
}

void example_get_endtool_analog_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::AnalogList endtool_ao_data;
    bool is_success = indy.get_endtool_ao(endtool_ao_data);

    if (is_success) {
        std::cout << "GetEndAO RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_ao_data.signals_size(); i++) {
            const auto& signal = endtool_ao_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetEndAO RPC failed." << std::endl;
    }
}

void example_set_endtool_analog_outputs(IndyDCP3& indy) {
    Nrmk::IndyFramework::AnalogList end_ao_signal_list;

    // Create and configure the first analog output signal
    auto* end_ao_signal1 = end_ao_signal_list.add_signals();
    end_ao_signal1->set_address(1);
    end_ao_signal1->set_voltage(1000);

    // Set the analog outputs
    bool is_success = indy.set_endtool_ao(end_ao_signal_list);

    if (is_success) {
        std::cout << "SetEndAO RPC succeeded." << std::endl;
    } else {
        std::cerr << "SetEndAO RPC failed." << std::endl;
    }
}

void example_get_device_info(IndyDCP3& indy) {
    Nrmk::IndyFramework::DeviceInfo device_info;
    bool is_success = indy.get_device_info(device_info);

    if (is_success) {
        std::cout << "GetDeviceInfo RPC succeeded." << std::endl;
        std::cout << "Num joints: " << device_info.num_joints() << std::endl;
        std::cout << "Robot serial: " << device_info.robot_serial() << std::endl;
        std::cout << "Payload: " << device_info.payload() << std::endl;
        std::cout << "IO board FW version: " << device_info.io_board_fw_ver() << std::endl;
        std::cout << "Core board FW versions: ";
        for (int i = 0; i < device_info.core_board_fw_vers_size(); ++i) {
            std::cout << device_info.core_board_fw_vers(i) << " ";
        }
        std::cout << std::endl;
        std::cout << "Endtool board FW version: " << device_info.endtool_board_fw_ver() << std::endl;
        std::cout << "Controller version: " << device_info.controller_ver() << std::endl;
        std::cout << "Controller detail: " << device_info.controller_detail() << std::endl;
        std::cout << "Controller date: " << device_info.controller_date() << std::endl;
        std::cout << "Teleop loaded: " << device_info.teleop_loaded() << std::endl;
        std::cout << "Calibrated: " << device_info.calibrated() << std::endl;
    } else {
        std::cerr << "GetDeviceInfo RPC failed." << std::endl;
    }
}

void example_get_ft_sensor_data(IndyDCP3& indy) {
    Nrmk::IndyFramework::FTSensorData ft_sensor_data;
    bool is_success = indy.get_ft_sensor_data(ft_sensor_data);

    if (is_success) {
        std::cout << "GetFTSensorData RPC succeeded." << std::endl;
        std::cout << "FT Fx: " << ft_sensor_data.ft_fx() << std::endl;
        std::cout << "FT Fy: " << ft_sensor_data.ft_fy() << std::endl;
        std::cout << "FT Fz: " << ft_sensor_data.ft_fz() << std::endl;
        std::cout << "FT Tx: " << ft_sensor_data.ft_tx() << std::endl;
        std::cout << "FT Ty: " << ft_sensor_data.ft_ty() << std::endl;
        std::cout << "FT Tz: " << ft_sensor_data.ft_tz() << std::endl;        
    } else {
        std::cerr << "GetFTSensorData RPC failed." << std::endl;
    }
}

void example_stop_robot_motion(IndyDCP3& indy) {
    StopCategory stop_category = StopCategory::SMOOTH_BRAKE;
    bool is_success = indy.stop_motion(stop_category);
    if (is_success) {
        std::cout << "Robot motion stopped successfully with smooth brake." << std::endl;
    } else {
        std::cerr << "Failed to stop robot motion." << std::endl;
    }
}

void example_get_home_position(IndyDCP3& indy) {
    Nrmk::IndyFramework::JointPos home_jpos;
    bool is_success = indy.get_home_pos(home_jpos);

    if (is_success) {
        std::cout << "Get Home Pos RPC succeeded." << std::endl;
        for (int i = 0; i < home_jpos.jpos_size(); i++) {
            std::cout << "Joint " << i << " position: " << home_jpos.jpos(i) << std::endl;
        }
    } else {
        std::cerr << "Get Home Pos RPC failed." << std::endl;
    }
}

void example_joint_move(IndyDCP3& indy, const std::vector<float>& j_pos, int base_type = JointBaseType::RELATIVE_JOINT) {
    bool is_success = indy.movej(j_pos, base_type);
    if (is_success) {
        std::cout << "MoveJ command executed successfully." << std::endl;
    } else {
        std::cerr << "MoveJ command failed." << std::endl;
    }
}

void example_task_move(IndyDCP3& indy, const std::array<float, 6>& t_pos, int base_type = TaskBaseType::RELATIVE_TASK) {
    bool is_success = indy.movel(t_pos, base_type);
    if (is_success) {
        std::cout << "MoveL command executed successfully." << std::endl;
    } else {
        std::cerr << "MoveL command failed." << std::endl;
    }
}

void example_move_along_circular_path(IndyDCP3& indy, const std::array<float, 6>& t_pos1, const std::array<float, 6>& t_pos2, float angle) {
    bool is_success = indy.movec(t_pos1, t_pos2, angle);
    if (is_success) {
        std::cout << "MoveC command executed successfully." << std::endl;
    } else {
        std::cerr << "MoveC command failed." << std::endl;
    }
}

void example_move_robot_to_home(IndyDCP3& indy) {
    bool is_success = indy.move_home();
    if (is_success) {
        std::cout << "MoveHome command executed successfully." << std::endl;
    } else {
        std::cerr << "MoveHome command failed." << std::endl;
    }
}

void example_start_teleoperation(IndyDCP3& indy, TeleMethod method=TeleMethod::TELE_JOINT_RELATIVE) {
    bool is_success = indy.start_teleop(method);
    if (is_success) {
        std::cout << "Teleoperation started successfully in mode: " << method << std::endl;
    } else {
        std::cerr << "Failed to start teleoperation." << std::endl;
    }
}

void example_move_joints_in_teleoperation(IndyDCP3& indy, const std::vector<float>& jpos) {
    bool is_success = indy.movetelej(jpos, 1.0, 1.0, TeleMethod::TELE_JOINT_RELATIVE);
    if (is_success) {
        std::cout << "Joint positions moved successfully in teleoperation mode." << std::endl;
    } else {
        std::cerr << "Failed to move joint positions in teleoperation mode." << std::endl;
    }
}

void example_stop_teleoperation(IndyDCP3& indy) {
    bool is_success = indy.stop_teleop();
    if (is_success) {
        std::cout << "Teleoperation stopped successfully." << std::endl;
    } else {
        std::cerr << "Failed to stop teleoperation." << std::endl;
    }
}

void example_inverse_kinematics(IndyDCP3& indy, const std::array<float, 6>& tpos, const std::vector<float>& init_jpos) {
    std::vector<float> jpos;
    bool is_success = indy.inverse_kin(tpos, init_jpos, jpos);
    
    if (is_success) {
        std::cout << "Inverse Kinematics successful. Joint positions: ";
        for (float jp : jpos) {
            std::cout << jp << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Inverse Kinematics failed." << std::endl;
    }
}

void example_inverse_kinematics(IndyDCP3& indy) {

    std::array<float, 6> tpos = {350.0f, -186.5f, 522.0f, -180.0f, 0.0f, 180.0f};
    std::vector<float> init_jpos = {0.0, 0.0, -90.0, 0.0, -90.0, 0.0};
    Nrmk::IndyFramework::InverseKinematicsReq request;
    Nrmk::IndyFramework::InverseKinematicsRes response;

    for (float value : tpos) {
        request.add_tpos(value);
    }
    for (float value : init_jpos) {
        request.add_init_jpos(value);
    }

    bool is_success = indy.inverse_kin(request, response);
    if (is_success) {
        std::vector<float> jpos;
        for (int i = 0; i < response.jpos_size(); ++i) {
            jpos.push_back(response.jpos(i));
        }

        std::cout << "Inverse Kinematics successful. Joint positions: ";
        for (float jp : jpos) {
            std::cout << jp << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Inverse Kinematics failed." << std::endl;
    }
}

void example_enable_direct_teaching(IndyDCP3& indy, bool enable) {
    bool is_success = indy.set_direct_teaching(enable);
    if (is_success) {
        std::cout << "Direct teaching " << (enable ? "enabled." : "disabled.") << std::endl;
    } else {
        std::cerr << "Failed to " << (enable ? "enable" : "disable") << " direct teaching." << std::endl;
    }
}

void example_set_simulation_mode(IndyDCP3& indy, bool enable) {
    bool is_success = indy.set_simulation_mode(enable);
    if (is_success) {
        std::cout << "Simulation mode " << (enable ? "enabled." : "disabled.") << std::endl;
    } else {
        std::cerr << "Failed to " << (enable ? "enable" : "disable") << " simulation mode." << std::endl;
    }
}

void example_recover_robot(IndyDCP3& indy) {
    bool is_success = indy.recover();
    if (is_success) {
        std::cout << "Recovery successful." << std::endl;
    } else {
        std::cerr << "Recovery failed." << std::endl;
    }
}

void example_enable_manual_recovery(IndyDCP3& indy, bool enable) {
    bool is_success = indy.set_manual_recovery(enable);
    if (is_success) {
        std::cout << "Manual recovery " << (enable ? "enabled." : "disabled.") << std::endl;
    } else {
        std::cerr << "Failed to " << (enable ? "enable" : "disable") << " manual recovery." << std::endl;
    }
}

void example_calculate_and_print_relative_pose(IndyDCP3& indy, 
                                       const std::array<float, 6>& current_pos, 
                                       const std::array<float, 6>& relative_pos) {
    std::array<float, 6> calculated_pose;
    bool is_success = indy.calculate_current_pose_rel(current_pos, relative_pos, TaskBaseType::ABSOLUTE_TASK, calculated_pose);

    if (is_success) {
        std::cout << "Calculated pose: ";
        for (float cp : calculated_pose) {
            std::cout << cp << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to calculate current relative pose." << std::endl;
    }
}

void example_play_program(IndyDCP3& indy, const std::string& program_name, int program_index) {
    bool is_success = indy.play_program(program_name, program_index);
    if (is_success) {
        std::cout << "Program started successfully." << std::endl;
    } else {
        std::cerr << "Failed to start program." << std::endl;
    }
}

void example_pause_program(IndyDCP3& indy) {
    bool is_success = indy.pause_program();
    if (is_success) {
        std::cout << "Program paused successfully." << std::endl;
    } else {
        std::cerr << "Failed to pause program." << std::endl;
    }
}

void example_resume_program(IndyDCP3& indy) {
    bool is_success = indy.resume_program();
    if (is_success) {
        std::cout << "Program resumed successfully." << std::endl;
    } else {
        std::cerr << "Failed to resume program." << std::endl;
    }
}

void example_stop_program(IndyDCP3& indy) {
    bool is_success = indy.stop_program();
    if (is_success) {
        std::cout << "Program stopped successfully." << std::endl;
    } else {
        std::cerr << "Failed to stop program." << std::endl;
    }
}

void example_set_speed_ratio(IndyDCP3& indy, unsigned int speed_ratio) {
    bool is_success = indy.set_speed_ratio(speed_ratio);
    if (is_success) {
        std::cout << "Speed ratio set to " << speed_ratio << "%" << std::endl;
    } else {
        std::cerr << "Failed to set speed ratio." << std::endl;
    }
}

void example_set_bool_var(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::BoolVariable> set_bool_vars;

    Nrmk::IndyFramework::BoolVariable bool_var1;
    bool_var1.set_addr(1);
    bool_var1.set_value(true);
    set_bool_vars.push_back(bool_var1);

    Nrmk::IndyFramework::BoolVariable bool_var2;
    bool_var2.set_addr(2);
    bool_var2.set_value(false);
    set_bool_vars.push_back(bool_var2);

    bool is_success = indy.set_bool_variable(set_bool_vars);
    if (is_success) {
        std::cout << "Bool variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set bool variables." << std::endl;
    }
}

void example_set_int_var(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::IntVariable> set_int_vars;

    Nrmk::IndyFramework::IntVariable int_var1;
    int_var1.set_addr(1);
    int_var1.set_value(100);
    set_int_vars.push_back(int_var1);

    Nrmk::IndyFramework::IntVariable int_var2;
    int_var2.set_addr(2);
    int_var2.set_value(200);
    set_int_vars.push_back(int_var2);

    bool is_success = indy.set_int_variable(set_int_vars);
    if (is_success) {
        std::cout << "Int variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set int variables." << std::endl;
    }
}

void example_set_float_var(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::FloatVariable> set_float_vars;

    Nrmk::IndyFramework::FloatVariable float_var1;
    float_var1.set_addr(1);
    float_var1.set_value(1.23f);
    set_float_vars.push_back(float_var1);

    Nrmk::IndyFramework::FloatVariable float_var2;
    float_var2.set_addr(2);
    float_var2.set_value(4.56f);
    set_float_vars.push_back(float_var2);

    bool is_success = indy.set_float_variable(set_float_vars);
    if (is_success) {
        std::cout << "Float variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set float variables." << std::endl;
    }
}

void example_set_jpos_var(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::JPosVariable> set_jpos_vars;

    Nrmk::IndyFramework::JPosVariable jpos_var1;
    jpos_var1.set_addr(1);
    jpos_var1.add_jpos(0.1f);
    jpos_var1.add_jpos(0.2f);
    jpos_var1.add_jpos(0.3f);
    jpos_var1.add_jpos(0.4f);
    jpos_var1.add_jpos(0.5f);
    jpos_var1.add_jpos(0.6f);
    set_jpos_vars.push_back(jpos_var1);

    Nrmk::IndyFramework::JPosVariable jpos_var2;
    jpos_var2.set_addr(2);
    jpos_var2.add_jpos(1.0f);
    jpos_var2.add_jpos(1.1f);
    jpos_var2.add_jpos(1.2f);
    jpos_var2.add_jpos(1.3f);
    jpos_var2.add_jpos(1.4f);
    jpos_var2.add_jpos(1.5f);
    set_jpos_vars.push_back(jpos_var2);

    bool is_success = indy.set_jpos_variable(set_jpos_vars);
    if (is_success) {
        std::cout << "JPos variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set JPos variables." << std::endl;
    }
}

void example_set_tpos_var(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::TPosVariable> set_tpos_vars;

    Nrmk::IndyFramework::TPosVariable tpos_var1;
    tpos_var1.set_addr(1);
    tpos_var1.add_tpos(10.0f);  // x
    tpos_var1.add_tpos(20.0f);  // y
    tpos_var1.add_tpos(30.0f);  // z
    tpos_var1.add_tpos(40.0f);  // u
    tpos_var1.add_tpos(50.0f);  // v
    tpos_var1.add_tpos(60.0f);  // w
    set_tpos_vars.push_back(tpos_var1);

    Nrmk::IndyFramework::TPosVariable tpos_var2;
    tpos_var2.set_addr(2);
    tpos_var2.add_tpos(11.0f);  // x
    tpos_var2.add_tpos(21.0f);  // y
    tpos_var2.add_tpos(31.0f);  // z
    tpos_var2.add_tpos(41.0f);  // u
    tpos_var2.add_tpos(51.0f);  // v
    tpos_var2.add_tpos(61.0f);  // w
    set_tpos_vars.push_back(tpos_var2);

    bool is_success = indy.set_tpos_variable(set_tpos_vars);
    if (is_success) {
        std::cout << "TPos variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set TPos variables." << std::endl;
    }
}

void example_get_bool_variables(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::BoolVariable> get_bool_vars;
    bool is_success = indy.get_bool_variable(get_bool_vars);
    if (is_success) {
        std::cout << "Bool variables retrieved successfully." << std::endl;
        for (const auto& var : get_bool_vars) {
            std::cout << "Address: " << var.addr() << ", Value: " << var.value() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve bool variables." << std::endl;
    }
}

void example_get_int_variables(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::IntVariable> get_int_vars;
    bool is_success = indy.get_int_variable(get_int_vars);
    if (is_success) {
        std::cout << "Integer variables retrieved successfully." << std::endl;
        for (const auto& var : get_int_vars) {
            std::cout << "Address: " << var.addr() << ", Value: " << var.value() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve integer variables." << std::endl;
    }
}

void example_get_float_variables(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::FloatVariable> get_float_vars;
    bool is_success = indy.get_float_variable(get_float_vars);
    if (is_success) {
        std::cout << "Float variables retrieved successfully." << std::endl;
        for (const auto& var : get_float_vars) {
            std::cout << "Address: " << var.addr() << ", Value: " << var.value() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve float variables." << std::endl;
    }
}

void example_get_jpos_variables(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::JPosVariable> get_jpos_vars;
    bool is_success = indy.get_jpos_variable(get_jpos_vars);
    if (is_success) {
        std::cout << "JPos variables retrieved successfully." << std::endl;
        for (const auto& var : get_jpos_vars) {
            std::cout << "Address: " << var.addr() << ", JPos: ";
            for (const auto& jpos : var.jpos()) {
                std::cout << jpos << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve JPos variables." << std::endl;
    }
}

void example_get_tpos_variables(IndyDCP3& indy) {
    std::vector<Nrmk::IndyFramework::TPosVariable> get_tpos_vars;
    bool is_success = indy.get_tpos_variable(get_tpos_vars);
    if (is_success) {
        std::cout << "TPos variables retrieved successfully." << std::endl;
        for (const auto& var : get_tpos_vars) {
            std::cout << "Address: " << var.addr() << ", TPos: ";
            for (const auto& tpos : var.tpos()) {
                std::cout << tpos << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve TPos variables." << std::endl;
    }
}

void example_set_home_position(IndyDCP3& indy) {
    Nrmk::IndyFramework::JointPos set_home_jpos;
    set_home_jpos.add_jpos(0.0);
    set_home_jpos.add_jpos(0.0);
    set_home_jpos.add_jpos(90.0);
    set_home_jpos.add_jpos(0.0);
    set_home_jpos.add_jpos(90.0);
    set_home_jpos.add_jpos(0.0);

    bool is_success = indy.set_home_pos(set_home_jpos);
    if (is_success) {
        std::cout << "Home position set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set home position." << std::endl;
    }
}

void example_get_reference_frame(IndyDCP3& indy) {
    std::array<float, 6> get_ref_frame;
    bool is_success = indy.get_ref_frame(get_ref_frame);
    if (is_success) {
        std::cout << "Reference frame retrieved successfully." << std::endl;
        for (const auto& value : get_ref_frame) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to retrieve reference frame." << std::endl;
    }
}

void example_set_reference_frame(IndyDCP3& indy, const std::array<float, 6>& ref_frame) {
    bool is_success = indy.set_ref_frame(ref_frame);
    if (is_success) {
        std::cout << "Reference frame set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set reference frame." << std::endl;
    }
}

void example_set_planar_reference_frame(IndyDCP3& indy, std::array<float, 6>& fpos_out, 
                                const std::array<float, 6>& fpos0, const std::array<float, 6>& fpos1, const std::array<float, 6>& fpos2) {
    bool is_success = indy.set_ref_frame_planar(fpos_out, fpos0, fpos1, fpos2);
    if (is_success) {
        std::cout << "Planar reference frame set successfully. Resulting frame:" << std::endl;
        for (const auto& value : fpos_out) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to set planar reference frame." << std::endl;
    }
}

void example_set_tool_frame(IndyDCP3& indy, const std::array<float, 6>& tool_frame) {
    bool is_success = indy.set_tool_frame(tool_frame);
    if (is_success) {
        std::cout << "Tool frame set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set tool frame." << std::endl;
    }
}

void example_set_friction_compensation(IndyDCP3& indy) {
    Nrmk::IndyFramework::FrictionCompSet set_friction_comp;
    set_friction_comp.set_control_comp_enable(false);
    set_friction_comp.set_teaching_comp_enable(false);

    std::vector<int> control_comp_levels = {5, 5, 5, 5, 5, 5};
    std::vector<int> dt_comp_levels = {5, 2, 5, 5, 5, 5};

    // Add control compensation levels
    for (int level : control_comp_levels) {
        set_friction_comp.add_control_comp_levels(level);
    }

    // Add teaching compensation levels
    for (int level : dt_comp_levels) {
        set_friction_comp.add_teaching_comp_levels(level);
    }

    bool is_success = indy.set_friction_comp(set_friction_comp);
    if (is_success) {
        std::cout << "Friction compensation set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set friction compensation." << std::endl;
    }
}

void example_get_friction_compensation(IndyDCP3& indy) {
    Nrmk::IndyFramework::FrictionCompSet friction_comp;
    bool is_success = indy.get_friction_comp(friction_comp);
    if (is_success) {
        std::cout << "Friction compensation data retrieved successfully." << std::endl;

        std::cout << "Control Compensation Enabled: " << (friction_comp.control_comp_enable() ? "Yes" : "No") << std::endl;
        std::cout << "Control Compensation Levels: ";
        for (int i = 0; i < friction_comp.control_comp_levels_size(); ++i) {
            std::cout << friction_comp.control_comp_levels(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Teaching Compensation Enabled: " << (friction_comp.teaching_comp_enable() ? "Yes" : "No") << std::endl;
        std::cout << "Teaching Compensation Levels: ";
        for (int i = 0; i < friction_comp.teaching_comp_levels_size(); ++i) {
            std::cout << friction_comp.teaching_comp_levels(i) << " ";
        }
        std::cout << std::endl;

    } else {
        std::cerr << "Failed to retrieve friction compensation data." << std::endl;
    }
}

void example_set_tool_properties(IndyDCP3& indy) {
    float mass = 0.5f;
    std::array<float, 3> center_of_mass = {0.0f, 0.1f, 0.2f};
    std::array<float, 6> inertia = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};

    Nrmk::IndyFramework::ToolProperties set_tool_properties;
    set_tool_properties.set_mass(mass);

    for (float value : center_of_mass) {
        set_tool_properties.add_center_of_mass(value);
    }

    for (float value : inertia) {
        set_tool_properties.add_inertia(value);
    }

    bool is_success = indy.set_tool_property(set_tool_properties);
    if (is_success) {
        std::cout << "Tool properties set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set tool properties." << std::endl;
    }
}

void example_get_tool_properties(IndyDCP3& indy) {
    Nrmk::IndyFramework::ToolProperties tool_properties;
    bool is_success = indy.get_tool_property(tool_properties);
    if (is_success) {
        std::cout << "Tool properties retrieved successfully." << std::endl;
        std::cout << "Mass: " << tool_properties.mass() << std::endl;

        std::cout << "Center of Mass: ";
        for (int i = 0; i < tool_properties.center_of_mass_size(); ++i) {
            std::cout << tool_properties.center_of_mass(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Inertia: ";
        for (int i = 0; i < tool_properties.inertia_size(); ++i) {
            std::cout << tool_properties.inertia(i) << " ";
        }
        std::cout << std::endl;

    } else {
        std::cerr << "Failed to retrieve tool properties." << std::endl;
    }
}

void example_set_collision_sensitivity_level(IndyDCP3& indy, unsigned int level) {
    bool is_success = indy.set_coll_sens_level(level);
    if (is_success) {
        std::cout << "Collision sensitivity level set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set collision sensitivity level." << std::endl;
    }
}

void example_get_collision_sensitivity_level(IndyDCP3& indy) {
    unsigned int collision_level;
    bool is_success = indy.get_coll_sens_level(collision_level);
    if (is_success) {
        std::cout << "Collision sensitivity level: " << collision_level << std::endl;
    } else {
        std::cerr << "Failed to retrieve collision sensitivity level." << std::endl;
    }
}

void example_get_collision_sensitivity_parameters(IndyDCP3& indy) {
    Nrmk::IndyFramework::CollisionThresholds get_coll_sens_param;
    bool is_success = indy.get_coll_sens_param(get_coll_sens_param);
    if (is_success) {
        std::cout << "Collision sensitivity parameters retrieved successfully." << std::endl;

        std::cout << "j_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.j_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.j_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "j_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.j_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.j_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.t_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.t_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.t_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.t_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "error_bases: ";
        for (int i = 0; i < get_coll_sens_param.error_bases_size(); ++i) {
            std::cout << get_coll_sens_param.error_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "error_tangents: ";
        for (int i = 0; i < get_coll_sens_param.error_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.error_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_constvel_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.t_constvel_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.t_constvel_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_constvel_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.t_constvel_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.t_constvel_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_conveyor_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.t_conveyor_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.t_conveyor_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_conveyor_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.t_conveyor_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.t_conveyor_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

    } else {
        std::cerr << "Failed to retrieve collision sensitivity parameters." << std::endl;
    }
}

void example_set_collision_sensitivity_parameters(IndyDCP3& indy) {
    Nrmk::IndyFramework::CollisionThresholds coll_sens_param;
    
    std::array<float, 6> j_torque_bases = {9042, 9040, 9019, 9014, 9012, 909};
    std::array<float, 6> j_torque_tangents = {1.2f, 1.2f, 0.6f, 0.0f, 0.6f, 0.4f};
    std::array<float, 6> t_torque_bases = {9037, 9016, 9023.4f, 909.6f, 908.4f, 9012};
    std::array<float, 6> t_torque_tangents = {5.4f, 13.2f, 1.8f, 1.2f, 0.8f, 4.0f};
    std::array<float, 6> error_bases = {90.013f, 90.007f, 90.007f, 90.009f, 90.02f, 90.024f};
    std::array<float, 6> error_tangents = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 6> t_constvel_torque_bases = {900.0, 900.0, 900.0, 900.0, 900.0, 900.0};
    std::array<float, 6> t_constvel_torque_tangents = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 6> t_conveyor_torque_bases = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 6> t_conveyor_torque_tangents = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < 6; ++i) {
        coll_sens_param.add_j_torque_bases(j_torque_bases[i]);
        coll_sens_param.add_j_torque_tangents(j_torque_tangents[i]);
        coll_sens_param.add_t_torque_bases(t_torque_bases[i]);
        coll_sens_param.add_t_torque_tangents(t_torque_tangents[i]);
        coll_sens_param.add_error_bases(error_bases[i]);
        coll_sens_param.add_error_tangents(error_tangents[i]);
        coll_sens_param.add_t_constvel_torque_bases(t_constvel_torque_bases[i]);
        coll_sens_param.add_t_constvel_torque_tangents(t_constvel_torque_tangents[i]);
        coll_sens_param.add_t_conveyor_torque_bases(t_conveyor_torque_bases[i]);
        coll_sens_param.add_t_conveyor_torque_tangents(t_conveyor_torque_tangents[i]);
    }

    bool is_success = indy.set_coll_sens_param(coll_sens_param);
    if (is_success) {
        std::cout << "Collision sensitivity parameters set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set collision sensitivity parameters." << std::endl;
    }
}

void example_get_collision_policy(IndyDCP3& indy) {
    Nrmk::IndyFramework::CollisionPolicy get_coll_policy;
    bool is_success = indy.get_coll_policy(get_coll_policy);
    if (is_success) {
        std::cout << "Collision policy retrieved successfully." << std::endl;
        std::cout << "Policy: " << get_coll_policy.policy() << std::endl;
        std::cout << "Sleep Time: " << get_coll_policy.sleep_time() << " seconds" << std::endl;
        std::cout << "Gravity Time: " << get_coll_policy.gravity_time() << " seconds" << std::endl;
    } else {
        std::cerr << "Failed to retrieve collision policy." << std::endl;
    }
}

void example_set_collision_policy(IndyDCP3& indy) {
    Nrmk::IndyFramework::CollisionPolicy coll_policy;
    coll_policy.set_policy(Nrmk::IndyFramework::CollisionPolicyType::COLL_RESUME_AFTER_SLEEP);
    coll_policy.set_sleep_time(3.0f);
    coll_policy.set_gravity_time(0.1f);

    bool is_success = indy.set_coll_policy(coll_policy);
    if (is_success) {
        std::cout << "Collision policy set successfully." << std::endl;
        switch (coll_policy.policy()) {
            case Nrmk::IndyFramework::CollisionPolicyType::COLL_NO_DETECT:
                std::cout << "Policy: No Collision Detection" << std::endl;
                break;
            case Nrmk::IndyFramework::CollisionPolicyType::COLL_PAUSE:
                std::cout << "Policy: Collision Pause" << std::endl;
                break;
            case Nrmk::IndyFramework::CollisionPolicyType::COLL_RESUME_AFTER_SLEEP:
                std::cout << "Policy: Resume After Sleep" << std::endl;
                break;
            case Nrmk::IndyFramework::CollisionPolicyType::COLL_STOP:
                std::cout << "Policy: Collision Stop" << std::endl;
                break;
            default:
                std::cerr << "Unknown policy type." << std::endl;
                break;
        }
        std::cout << "Sleep Time: " << coll_policy.sleep_time() << " seconds" << std::endl;
        std::cout << "Gravity Time: " << coll_policy.gravity_time() << " seconds" << std::endl;
    } else {
        std::cerr << "Failed to set collision policy." << std::endl;
    }
}

void example_get_safety_limits(IndyDCP3& indy) {
    Nrmk::IndyFramework::SafetyLimits safety_limits;
    bool is_success = indy.get_safety_limits(safety_limits);
    if (is_success) {
        std::cout << "Safety limits retrieved successfully." << std::endl;
        std::cout << "Power Limit: " << safety_limits.power_limit() << std::endl;
        std::cout << "Power Limit Ratio: " << safety_limits.power_limit_ratio() << std::endl;
        std::cout << "TCP Force Limit: " << safety_limits.tcp_force_limit() << std::endl;
        std::cout << "TCP Force Limit Ratio: " << safety_limits.tcp_force_limit_ratio() << std::endl;
        std::cout << "TCP Speed Limit: " << safety_limits.tcp_speed_limit() << std::endl;
        std::cout << "TCP Speed Limit Ratio: " << safety_limits.tcp_speed_limit_ratio() << std::endl;

        std::cout << "Joint Limits: ";
        for (int i = 0; i < safety_limits.joint_upper_limits_size(); ++i) {
            std::cout << safety_limits.joint_upper_limits(i) << " ";
        }
        for (int i = 0; i < safety_limits.joint_lower_limits_size(); ++i) {
            std::cout << safety_limits.joint_lower_limits(i) << " ";
        }

        std::cout << std::endl;
    } else {
        std::cerr << "Failed to retrieve safety limits." << std::endl;
    }
}

void example_set_safety_limits(IndyDCP3& indy) {
    Nrmk::IndyFramework::SafetyLimits safety_limits;
    safety_limits.set_power_limit(1500);
    safety_limits.set_power_limit_ratio(100);
    safety_limits.set_tcp_force_limit(800);
    safety_limits.set_tcp_force_limit_ratio(100);
    safety_limits.set_tcp_speed_limit(4);
    safety_limits.set_tcp_speed_limit_ratio(100);
    
    std::vector<float> joint_limits = {175.0f, 175.0f, 175.0f, 175.0f, 175.0f, 175.0f};

    for (const auto& limit : joint_limits) {
        safety_limits.add_joint_upper_limits(limit);
    }

    for (const auto& limit : joint_limits) {
        safety_limits.add_joint_lower_limits(-limit);
    }

    bool is_success = indy.set_safety_limits(safety_limits);
    if (is_success) {
        std::cout << "Safety limits set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set safety limits." << std::endl;
    }
}

void example_activate_sdk(IndyDCP3& indy) {
    Nrmk::IndyFramework::SDKLicenseInfo request;
    request.set_license_key("3EF2337C65E079205F93E57EADF9889CF3E66B38E695F9465B1780311C4EEF03");
    request.set_expire_date("2025-10-21");

    bool is_active = false;

    Nrmk::IndyFramework::SDKLicenseResp response;
    bool is_success;

    while(!is_active) {        
        is_success = indy.activate_sdk(request, response);
        if (is_success) {
            std::cout << "SDK Activated: " << (response.activated() ? "Yes" : "No") << std::endl;
            std::cout << "Response Code: " << response.response().code() << ", Message: " << response.response().msg() << std::endl;
        } else {
            std::cerr << "Failed to activate SDK." << std::endl;
        }
        if(response.activated()) is_active = true;
    }
}

void example_get_custom_control_mode(IndyDCP3& indy) {
    int get_mode;
    bool is_success = indy.get_custom_control_mode(get_mode);
    if (is_success) {
        std::cout << "Custom control mode: " << get_mode << std::endl;
    } else {
        std::cerr << "Failed to get custom control mode." << std::endl;
    }
}

void example_set_custom_control_mode(IndyDCP3& indy, int mode) {
    bool is_success = indy.set_custom_control_mode(mode);
    if (is_success) {
        std::cout << "Custom control mode set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set custom control mode." << std::endl;
    }
}

void example_get_custom_control_gain(IndyDCP3& indy) {
    Nrmk::IndyFramework::CustomGainSet get_custom_gain;
    bool is_success = indy.get_custom_control_gain(get_custom_gain);
    if (is_success) {
        std::cout << "Custom control gains retrieved successfully." << std::endl;

        std::cout << "Gain0: ";
        for (int i = 0; i < get_custom_gain.gain0_size(); ++i) {
            std::cout << get_custom_gain.gain0(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain1: ";
        for (int i = 0; i < get_custom_gain.gain1_size(); ++i) {
            std::cout << get_custom_gain.gain1(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain2: ";
        for (int i = 0; i < get_custom_gain.gain2_size(); ++i) {
            std::cout << get_custom_gain.gain2(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain3: ";
        for (int i = 0; i < get_custom_gain.gain3_size(); ++i) {
            std::cout << get_custom_gain.gain3(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain4: ";
        for (int i = 0; i < get_custom_gain.gain4_size(); ++i) {
            std::cout << get_custom_gain.gain4(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain5: ";
        for (int i = 0; i < get_custom_gain.gain5_size(); ++i) {
            std::cout << get_custom_gain.gain5(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain6: ";
        for (int i = 0; i < get_custom_gain.gain6_size(); ++i) {
            std::cout << get_custom_gain.gain6(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain7: ";
        for (int i = 0; i < get_custom_gain.gain7_size(); ++i) {
            std::cout << get_custom_gain.gain7(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain8: ";
        for (int i = 0; i < get_custom_gain.gain8_size(); ++i) {
            std::cout << get_custom_gain.gain8(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain9: ";
        for (int i = 0; i < get_custom_gain.gain9_size(); ++i) {
            std::cout << get_custom_gain.gain9(i) << " ";
        }
        std::cout << std::endl;

        // Other gains...
    } else {
        std::cerr << "Failed to retrieve custom control gains." << std::endl;
    }
}

void example_set_custom_control_gain_high(IndyDCP3& indy) {
    Nrmk::IndyFramework::CustomGainSet custom_gain_set;    

    // Example gains
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);

    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);

    custom_gain_set.add_gain2(600.0);
    custom_gain_set.add_gain2(600.0);
    custom_gain_set.add_gain2(450.0);
    custom_gain_set.add_gain2(350.0);
    custom_gain_set.add_gain2(350.0);
    custom_gain_set.add_gain2(350.0);

    custom_gain_set.add_gain3(100.0);
    custom_gain_set.add_gain3(100.0);
    custom_gain_set.add_gain3(100.0);
    custom_gain_set.add_gain3(100.0);
    custom_gain_set.add_gain3(100.0);
    custom_gain_set.add_gain3(100.0);

    custom_gain_set.add_gain4(20.0);
    custom_gain_set.add_gain4(20.0);
    custom_gain_set.add_gain4(20.0);
    custom_gain_set.add_gain4(20.0);
    custom_gain_set.add_gain4(20.0);
    custom_gain_set.add_gain4(20.0);

    custom_gain_set.add_gain5(350.0);
    custom_gain_set.add_gain5(350.0);
    custom_gain_set.add_gain5(200.0);
    custom_gain_set.add_gain5(150.0);
    custom_gain_set.add_gain5(150.0);
    custom_gain_set.add_gain5(100.0);

    custom_gain_set.add_gain6(1.0);
    custom_gain_set.add_gain6(0.06);
    custom_gain_set.add_gain6(500.0);
    custom_gain_set.add_gain6(80.0);
    custom_gain_set.add_gain6(0.0);
    custom_gain_set.add_gain6(0.0);

    // custom_gain_set.add_gain3(100.0);
    // custom_gain_set.add_gain3(100.0);
    // custom_gain_set.add_gain3(100.0);
    // custom_gain_set.add_gain3(100.0);
    // custom_gain_set.add_gain3(100.0);
    // custom_gain_set.add_gain3(100.0);

    // custom_gain_set.add_gain4(20.0);
    // custom_gain_set.add_gain4(20.0);
    // custom_gain_set.add_gain4(20.0);
    // custom_gain_set.add_gain4(20.0);
    // custom_gain_set.add_gain4(20.0);
    // custom_gain_set.add_gain4(20.0);

    // custom_gain_set.add_gain5(600.0);
    // custom_gain_set.add_gain5(600.0);
    // custom_gain_set.add_gain5(450.0);
    // custom_gain_set.add_gain5(350.0);
    // custom_gain_set.add_gain5(350.0);
    // custom_gain_set.add_gain5(350.0);

    // custom_gain_set.add_gain6(20.0);
    // custom_gain_set.add_gain6(20.0);
    // custom_gain_set.add_gain6(30.0);
    // custom_gain_set.add_gain6(30.0);
    // custom_gain_set.add_gain6(3.0);
    // custom_gain_set.add_gain6(3.0);

    custom_gain_set.add_gain7(ft_calibration_(0));
    custom_gain_set.add_gain7(ft_calibration_(1));
    custom_gain_set.add_gain7(ft_calibration_(2));
    custom_gain_set.add_gain7(ft_calibration_(3));
    custom_gain_set.add_gain7(ft_calibration_(4));
    custom_gain_set.add_gain7(ft_calibration_(5));

    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);    

    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);


    bool is_success = indy.set_custom_control_gain(custom_gain_set);
    if (is_success) {
        std::cout << "Custom control gains set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set custom control gains." << std::endl;
    }
}

void example_set_custom_control_gain_high_impedance(IndyDCP3& indy) {
    Nrmk::IndyFramework::CustomGainSet custom_gain_set;    

    // Example gains
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);
    custom_gain_set.add_gain0(100.0);

    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);
    custom_gain_set.add_gain1(20.0);

    custom_gain_set.add_gain2(600.0);
    custom_gain_set.add_gain2(600.0);
    custom_gain_set.add_gain2(450.0);
    custom_gain_set.add_gain2(350.0);
    custom_gain_set.add_gain2(350.0);
    custom_gain_set.add_gain2(350.0);

    custom_gain_set.add_gain3(3.0);
    custom_gain_set.add_gain3(2.0);
    custom_gain_set.add_gain3(50.0);
    custom_gain_set.add_gain3(3.0);
    custom_gain_set.add_gain3(3.0);
    custom_gain_set.add_gain3(3.0);

    custom_gain_set.add_gain4(6.0);
    custom_gain_set.add_gain4(2.0);
    custom_gain_set.add_gain4(50.0);
    custom_gain_set.add_gain4(3.0);
    custom_gain_set.add_gain4(3.0);
    custom_gain_set.add_gain4(3.0);

    custom_gain_set.add_gain5(6.0);
    custom_gain_set.add_gain5(2.0);
    custom_gain_set.add_gain5(50.0);
    custom_gain_set.add_gain5(3.0);
    custom_gain_set.add_gain5(3.0);
    custom_gain_set.add_gain5(3.0);

    custom_gain_set.add_gain6(3.0);
    custom_gain_set.add_gain6(2.0);
    custom_gain_set.add_gain6(50.0);
    custom_gain_set.add_gain6(3.0);
    custom_gain_set.add_gain6(3.0);
    custom_gain_set.add_gain6(3.0);

    custom_gain_set.add_gain7(6.0);
    custom_gain_set.add_gain7(2.0);
    custom_gain_set.add_gain7(50.0);
    custom_gain_set.add_gain7(3.0);
    custom_gain_set.add_gain7(3.0);
    custom_gain_set.add_gain7(3.0);

    custom_gain_set.add_gain8(6.0);
    custom_gain_set.add_gain8(2.0);
    custom_gain_set.add_gain8(50.0);
    custom_gain_set.add_gain8(3.0);
    custom_gain_set.add_gain8(3.0);
    custom_gain_set.add_gain8(3.0);

    
    custom_gain_set.add_gain9(0.5);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);


    bool is_success = indy.set_custom_control_gain(custom_gain_set);
    if (is_success) {
        std::cout << "Custom control gains set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set custom control gains." << std::endl;
    }
}

void example_set_custom_control_gain_low(IndyDCP3& indy) {
    Nrmk::IndyFramework::CustomGainSet custom_gain_set;
    
    // Example gains
    custom_gain_set.add_gain0(2.0);
    custom_gain_set.add_gain0(2.0);
    custom_gain_set.add_gain0(2.0);
    custom_gain_set.add_gain0(0.0);
    custom_gain_set.add_gain0(0.0);
    custom_gain_set.add_gain0(0.0);

    custom_gain_set.add_gain1(0.0);
    custom_gain_set.add_gain1(0.0);
    custom_gain_set.add_gain1(0.0);
    custom_gain_set.add_gain1(0.0);
    custom_gain_set.add_gain1(0.0);
    custom_gain_set.add_gain1(0.0);

    custom_gain_set.add_gain2(0.0);
    custom_gain_set.add_gain2(0.0);
    custom_gain_set.add_gain2(0.0);
    custom_gain_set.add_gain2(0.0);
    custom_gain_set.add_gain2(0.0);
    custom_gain_set.add_gain2(0.0);

    custom_gain_set.add_gain3(0.0);
    custom_gain_set.add_gain3(0.0);
    custom_gain_set.add_gain3(0.0);
    custom_gain_set.add_gain3(0.0);
    custom_gain_set.add_gain3(0.0);
    custom_gain_set.add_gain3(0.0);

    custom_gain_set.add_gain4(0.0);
    custom_gain_set.add_gain4(0.0);
    custom_gain_set.add_gain4(0.0);
    custom_gain_set.add_gain4(0.0);
    custom_gain_set.add_gain4(0.0);
    custom_gain_set.add_gain4(0.0);

    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);

    custom_gain_set.add_gain6(0.0);
    custom_gain_set.add_gain6(0.0);
    custom_gain_set.add_gain6(0.0);
    custom_gain_set.add_gain6(0.0);
    custom_gain_set.add_gain6(0.0);
    custom_gain_set.add_gain6(0.0);

    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);

    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);
    custom_gain_set.add_gain8(0.0);

    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);


    bool is_success = indy.set_custom_control_gain(custom_gain_set);
    if (is_success) {
        std::cout << "Custom control gains set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set custom control gains." << std::endl;
    }
}

void example_start_log(IndyDCP3& indy) {
    bool is_success = indy.start_log();
    if (is_success) {
        std::cout << "Started realtime data logging." << std::endl;
    } else {
        std::cerr << "Failed to start data logging." << std::endl;
    }
}

void example_end_log(IndyDCP3& indy) {
    bool is_success = indy.end_log();
    if (is_success) {
        std::cout << "Finished and saved realtime data logging." << std::endl;
    } else {
        std::cerr << "Failed to end data logging." << std::endl;
    }
}

void example_wait_cmd(IndyDCP3& indy, int exam=0) {
    std::vector<float> j_pos_1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    indy.movej(j_pos_1, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE);

    if (exam == 0){
        indy.wait_time(0.1f);
    }
    else if (exam == 1){
        indy.wait_progress(100);
    }
    else if (exam == 2){
        indy.wait_traj(TrajCondition::TRAJ_ACC_DONE); // when acceleration done
    }
    else if (exam == 3){
        indy.wait_radius(10);
    }

    std::vector<float> j_pos_2 = {0.0, 0.0, -90.0, 0.0, -90.0, 0.0};
    indy.movej(j_pos_2, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE);
}

void example_wait_for_operation_state(IndyDCP3& indy) {
    bool is_success = indy.wait_for_operation_state(OpState::OP_MOVING);
    if (is_success) {
        std::cout << "The robot has reached the target position." << std::endl;
    } else {
        std::cerr << "Failed to wait for the robot to reach the target position." << std::endl;
    }
}

void example_wait_for_motion_state(IndyDCP3& indy) {
    bool is_success = indy.wait_for_motion_state("is_target_reached");
    if (is_success) {
        std::cout << "The robot has reached the target position." << std::endl;
    } else {
        std::cerr << "Failed to wait for the robot to reach the target position." << std::endl;
    }
}

void example_move_joint_waypoint(IndyDCP3& indy) {
    // Add joint waypoints
    indy.add_joint_waypoint({0, 0, 0, 0, 0, 0});
    indy.add_joint_waypoint({-44, 25, -63, 48, -7, -105});
    indy.add_joint_waypoint({0, 0, 90, 0, 90, 0});
    indy.add_joint_waypoint({-145, 31, -33, 117, -7, -133});
    indy.add_joint_waypoint({-90, -15, -90, 0, -75, 0});

    // Retrieve and print joint waypoints
    std::vector<std::vector<float>> waypoints;
    if (indy.get_joint_waypoint(waypoints)) {
        std::cout << "Successfully retrieved joint waypoints:" << std::endl;
        for (const auto& wp : waypoints) {
            for (float joint : wp) {
                std::cout << joint << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "No joint waypoints available." << std::endl;
    }

    // Move joint waypoints without move time
    indy.move_joint_waypoint();

    // Move joint waypoints with move time
    float move_time = 3.0;
    indy.move_joint_waypoint(move_time);

    // Clear joint waypoints
    indy.clear_joint_waypoint();
}

void example_move_task_waypoint(IndyDCP3& indy) {
    // Add task waypoints
    indy.add_task_waypoint({-186.54f, -454.45f, 415.61f, 179.99f, -0.06f, 89.98f});
    indy.add_task_waypoint({-334.67f, -493.07f, 259.00f, 179.96f, -0.12f, 89.97f});
    indy.add_task_waypoint({224.79f, -490.20f, 508.08f, 179.96f, -0.14f, 89.97f});
    indy.add_task_waypoint({-129.84f, -416.84f, 507.38f, 179.95f, -0.16f, 89.96f});
    indy.add_task_waypoint({-186.54f, -454.45f, 415.61f, 179.99f, -0.06f, 89.98f});

    // Retrieve and print task waypoints
    std::vector<std::array<float, 6>> t_waypoints;
    if (indy.get_task_waypoint(t_waypoints)) {
        std::cout << "Successfully retrieved task waypoints:" << std::endl;
        for (const auto& wp : t_waypoints) {
            for (float task : wp) {
                std::cout << task << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "No task waypoints available." << std::endl;
    }

    // Move task waypoints without move time
    indy.move_task_waypoint();

    // Move task waypoints with move time
    float move_time = 1.0f;
    indy.move_task_waypoint(move_time);

    // Clear task waypoints
    indy.clear_task_waypoint();
}

//------------- fw3.3 -----------------
void example_set_mount_pos(IndyDCP3& indy) {
    Nrmk::IndyFramework::MountingAngles mounting_angles;
    mounting_angles.set_ry(45.0f);
    mounting_angles.set_rz(30.0f);

    bool is_success = indy.set_mount_pos(mounting_angles);
    if (is_success) {
        std::cout << "Mounting angles set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set mounting angles" << std::endl;
    }
}

void example_get_mount_pos(IndyDCP3& indy) {
    Nrmk::IndyFramework::MountingAngles mounting_angles;

    bool is_success = indy.get_mount_pos(mounting_angles);
    if (is_success) {
        std::cout << "Mounting angles retrieved successfully." << std::endl;
        std::cout << "rot_y: " << mounting_angles.ry() << ", rot_z: " << mounting_angles.rz() << std::endl;
    } else {
        std::cerr << "Failed to retrieve mounting angles." << std::endl;
    }
}

void example_get_violation_message_queue(IndyDCP3& indy) {
    Nrmk::IndyFramework::ViolationMessageQueue violation_queue;
    bool is_success = indy.get_violation_message_queue(violation_queue);
    
    if (is_success) {
        std::cout << "Violation messages retrieved successfully." << std::endl;
        for (const auto& violation : violation_queue.violation_queue()) {
            std::cout << "Violation Code: " << violation.violation_code() << std::endl;
            std::cout << "Joint Index: " << violation.j_index() << std::endl;

            std::cout << "Integer Arguments: ";
            for (int32_t i_arg : violation.i_args()) {
                std::cout << i_arg << " ";
            }
            std::cout << std::endl;

            std::cout << "Float Arguments: ";
            for (float f_arg : violation.f_args()) {
                std::cout << f_arg << " ";
            }
            std::cout << std::endl;

            std::cout << "Violation String: " << violation.violation_str() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve violation messages." << std::endl;
    }
}

void example_get_stop_state(IndyDCP3& indy) {
    Nrmk::IndyFramework::StopState stop_state;
    bool is_success = indy.get_stop_state(stop_state);
    
    if (is_success) {
        std::cout << "Stop State retrieved successfully." << std::endl;
        std::cout << "Stop Category: " << stop_state.category() << std::endl;
    } else {
        std::cerr << "Failed to retrieve stop state." << std::endl;
    }
}

void example_set_endtool_rs485_rx(IndyDCP3& indy) {
    Nrmk::IndyFramework::EndtoolRS485Rx rs485_rx;
    rs485_rx.set_word1(1);
    rs485_rx.set_word2(2);

    bool is_success = indy.set_endtool_rs485_rx(rs485_rx);
    if (is_success) {
        std::cout << "Endtool RS485 RX set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set Endtool RS485 RX." << std::endl;
    }
}

void example_get_endtool_rs485_rx(IndyDCP3& indy) {
    Nrmk::IndyFramework::EndtoolRS485Rx rx_data;

    bool is_success = indy.get_endtool_rs485_rx(rx_data);
    if (is_success) {
        std::cout << "Endtool RS485 RX data retrieved successfully." << std::endl;
        std::cout << "Word1: " << rx_data.word1() << ", Word2: " << rx_data.word2() << std::endl;
    } else {
        std::cerr << "Failed to retrieve Endtool RS485 RX data." << std::endl;
    }
}

void example_get_endtool_rs485_tx(IndyDCP3& indy) {
    Nrmk::IndyFramework::EndtoolRS485Tx tx_data;

    bool is_success = indy.get_endtool_rs485_tx(tx_data);
    if (is_success) {
        std::cout << "Endtool RS485 TX data retrieved successfully." << std::endl;
        std::cout << "Word1: " << tx_data.word1() << ", Word2: " << tx_data.word2() << std::endl;
    } else {
        std::cerr << "Failed to retrieve Endtool RS485 TX data." << std::endl;
    }
}

void example_set_end_led_dim(IndyDCP3& indy) {
    Nrmk::IndyFramework::EndLedDim led_dim_request;
    led_dim_request.set_led_dim(10);

    bool is_success = indy.set_end_led_dim(led_dim_request);
    if (is_success) {
        std::cout << "End LED dim set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set End LED dim." << std::endl;
    }
}

void example_get_conveyor(IndyDCP3& indy) {
    Nrmk::IndyFramework::Conveyor conveyor;
    bool is_success = indy.get_conveyor(conveyor);
    if (is_success) {
        std::cout << "Conveyor retrieved successfully." << std::endl;
        std::cout << "Name: " << conveyor.name() << std::endl;

        // Encoder data
        std::cout << "Encoder Type: " << conveyor.encoder().type() << std::endl;
        std::cout << "Encoder Channel1: " << conveyor.encoder().channel1() << std::endl;
        std::cout << "Encoder Channel2: " << conveyor.encoder().channel2() << std::endl;
        std::cout << "Encoder Sample Number: " << conveyor.encoder().sample_num() << std::endl;
        std::cout << "Encoder mm per Tick: " << conveyor.encoder().mm_per_tick() << std::endl;
        std::cout << "Encoder Velocity (mm/s): " << conveyor.encoder().vel_const_mmps() << std::endl;
        std::cout << "Encoder Reversed: " << (conveyor.encoder().reversed() ? "True" : "False") << std::endl;

        // Trigger data
        std::cout << "Trigger Type: " << conveyor.trigger().type() << std::endl;
        std::cout << "Trigger Channel: " << conveyor.trigger().channel() << std::endl;
        std::cout << "Trigger Detect Rise: " << (conveyor.trigger().detect_rise() ? "True" : "False") << std::endl;

        // other Conveyor attributes
        std::cout << "Offset Distance: " << conveyor.offset_dist() << std::endl;
        std::cout << "Working Distance: " << conveyor.working_dist() << std::endl;

        // Direction data
        std::cout << "Direction: ";
        for (int i = 0; i < conveyor.direction().values_size(); ++i) {
            std::cout << conveyor.direction().values(i) << " ";
        }
        std::cout << std::endl;

        // Starting Pose data
        std::cout << "Starting Pose (Position): ";
        for (int i = 0; i < conveyor.starting_pose().p_size(); ++i) {
            std::cout << conveyor.starting_pose().p(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Starting Pose (Orientation): ";
        for (int i = 0; i < conveyor.starting_pose().q_size(); ++i) {
            std::cout << conveyor.starting_pose().q(i) << " ";
        }
        std::cout << std::endl;

        // Terminal Pose data
        std::cout << "Terminal Pose (Position): ";
        for (int i = 0; i < conveyor.terminal_pose().p_size(); ++i) {
            std::cout << conveyor.terminal_pose().p(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Terminal Pose (Orientation): ";
        for (int i = 0; i < conveyor.terminal_pose().q_size(); ++i) {
            std::cout << conveyor.terminal_pose().q(i) << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to retrieve conveyor." << std::endl;
    }
}

void example_set_conveyor_by_name(IndyDCP3& indy) {
    Nrmk::IndyFramework::Name conveyor_name;
    conveyor_name.set_name("Conveyor1");  // Set the conveyor name

    bool is_success = indy.set_conveyor_by_name(conveyor_name);
    if (is_success) {
        std::cout << "Conveyor set by name successfully." << std::endl;
    } else {
        std::cerr << "Failed to set conveyor by name." << std::endl;
    }
}

void example_get_conveyor_state(IndyDCP3& indy) {
    Nrmk::IndyFramework::ConveyorState conveyor_state;

    bool is_success = indy.get_conveyor_state(conveyor_state);
    if (is_success) {
        std::cout << "Conveyor state retrieved successfully." << std::endl;
        std::cout << "Velocity: " << conveyor_state.velocity() << std::endl;
        std::cout << "Triggered: " << conveyor_state.triggered() << std::endl;
    } else {
        std::cerr << "Failed to retrieve conveyor state." << std::endl;
    }
}

void example_set_sander_command(IndyDCP3& indy) {
    Nrmk::IndyFramework::SanderCommand::SanderType sander_type = Nrmk::IndyFramework::SanderCommand::SANDER_ONROBOT;
    std::string ip = "192.168.1.1";
    float speed = 1.5f;
    bool state = true;

    bool is_success = indy.set_sander_command(sander_type, ip, speed, state);
    if (is_success) {
        std::cout << "Sander command set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set sander command." << std::endl;
    }
}

void example_get_sander_command(IndyDCP3& indy) {
    Nrmk::IndyFramework::SanderCommand sander_command;
    bool is_success = indy.get_sander_command(sander_command);
    if (is_success) {
        std::cout << "Sander command retrieved successfully." << std::endl;
        std::cout << "Type: " << sander_command.type() << std::endl;
        std::cout << "IP: " << sander_command.ip() << std::endl;
        std::cout << "Speed: " << sander_command.speed() << std::endl;
        std::cout << "State: " << (sander_command.state() ? "On" : "Off") << std::endl;
    } else {
        std::cerr << "Failed to retrieve sander command." << std::endl;
    }
}

void example_get_load_factors(IndyDCP3& indy) {
    Nrmk::IndyFramework::GetLoadFactorsRes load_factors_res;
    bool is_success = indy.get_load_factors(load_factors_res);
    if (is_success) {
        std::cout << "Load Factors retrieved successfully." << std::endl;
        
        std::cout << "Percents: ";
        for (int i = 0; i < load_factors_res.percents_size(); ++i) {
            std::cout << load_factors_res.percents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Torques (Nm): ";
        for (int i = 0; i < load_factors_res.torques_size(); ++i) {
            std::cout << load_factors_res.torques(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Response Code: " << load_factors_res.response().code() 
                  << ", Message: " << load_factors_res.response().msg() << std::endl;
    } else {
        std::cerr << "Failed to retrieve Load Factors." << std::endl;
    }
}

void example_set_auto_mode(IndyDCP3& indy, bool on) {
    bool is_success = indy.set_auto_mode(on);
    if (is_success) {
        std::cout << "Auto Mode set successfully to " << (on ? "ON" : "OFF") << std::endl;
    } else {
        std::cerr << "Failed to set Auto Mode." << std::endl;
    }
}

void example_check_auto_mode(IndyDCP3& indy) {
    Nrmk::IndyFramework::CheckAutoModeRes check_auto_mode_res;
    bool is_success = indy.check_auto_mode(check_auto_mode_res);
    if (is_success) {
        std::cout << "Auto Mode status: " << (check_auto_mode_res.on() ? "ON" : "OFF") << std::endl;
        std::cout << "Message: " << check_auto_mode_res.msg() << std::endl;
    } else {
        std::cerr << "Failed to check Auto Mode." << std::endl;
    }
}

void example_check_reduced_mode(IndyDCP3& indy) {
    Nrmk::IndyFramework::CheckReducedModeRes reduced_mode_res;
    bool is_success = indy.check_reduced_mode(reduced_mode_res);
    if (is_success) {
        std::cout << "Reduced Mode status: " << (reduced_mode_res.on() ? "ON" : "OFF") << std::endl;
        std::cout << "Message: " << reduced_mode_res.msg() << std::endl;
    } else {
        std::cerr << "Failed to check Reduced Mode." << std::endl;
    }
}

void example_get_safety_function_state(IndyDCP3& indy) {
    Nrmk::IndyFramework::SafetyFunctionState safety_function_state;
    bool is_success = indy.get_safety_function_state(safety_function_state);
    if (is_success) {
        std::cout << "Safety Function State retrieved successfully." << std::endl;
        std::cout << "ID: " << safety_function_state.id() << ", State: " << safety_function_state.state() << std::endl;
        std::cout << "Response Code: " << safety_function_state.response().code()
                  << ", Message: " << safety_function_state.response().msg() << std::endl;
    } else {
        std::cerr << "Failed to retrieve Safety Function State." << std::endl;
    }
}

void example_request_safety_function(IndyDCP3& indy) {
    Nrmk::IndyFramework::SafetyFunctionState safety_function_state;
    safety_function_state.set_id(1);  // Example ID
    safety_function_state.set_state(2);  // Example State

    bool is_success = indy.request_safety_function(safety_function_state);
    if (is_success) {
        std::cout << "Safety Function requested successfully." << std::endl;
    } else {
        std::cerr << "Failed to request Safety Function." << std::endl;
    }
}

void example_get_safety_control_data(IndyDCP3& indy) {
    Nrmk::IndyFramework::SafetyControlData safety_control_data;
    bool is_success = indy.get_safety_control_data(safety_control_data);
    if (is_success) {
        std::cout << "Safety Control Data retrieved successfully." << std::endl;
        std::cout << "Auto Mode: " << (safety_control_data.auto_mode() ? "ON" : "OFF") << std::endl;
        std::cout << "Reduced Mode: " << (safety_control_data.reduced_mode() ? "ON" : "OFF") << std::endl;
        std::cout << "Enabler Pressed: " << (safety_control_data.enabler_pressed() ? "YES" : "NO") << std::endl;
        std::cout << "Safety State ID: " << safety_control_data.safety_state().id()
                  << ", State: " << safety_control_data.safety_state().state() << std::endl;
    } else {
        std::cerr << "Failed to retrieve Safety Control Data." << std::endl;
    }
}

void example_get_gripper_data(IndyDCP3& indy) {
    Nrmk::IndyFramework::GripperData gripper_data;
    bool is_success = indy.get_gripper_data(gripper_data);
    if (is_success) {
        std::cout << "Gripper Data retrieved successfully." << std::endl;
        std::cout << "Gripper Type: " << gripper_data.gripper_type() << std::endl;
        std::cout << "Gripper Position: " << gripper_data.gripper_position() << std::endl;
        std::cout << "Gripper State: " << gripper_data.gripper_state() << std::endl;
    } else {
        std::cerr << "Failed to retrieve Gripper Data." << std::endl;
    }
}

void example_set_gripper_command(IndyDCP3& indy) {
    Nrmk::IndyFramework::GripperCommand gripper_command;
    gripper_command.set_gripper_command(Nrmk::IndyFramework::GripperCommand::ACTIVATE); // Example command
    gripper_command.set_gripper_type(Nrmk::IndyFramework::GripperType::ROBOTIQ_GRIPPER); // Example gripper type

    // TODO: Check pvt data length
    std::vector<int32_t> pvt_data = {1, 2, 3};
    for (const auto& data : pvt_data) {
        gripper_command.add_gripper_pvt_data(data);
    }

    bool is_success = indy.set_gripper_command(gripper_command);
    if (is_success) {
        std::cout << "Gripper Command set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set Gripper Command." << std::endl;
    }
}

void example_activate_cri(IndyDCP3& indy) {
    bool is_success = indy.activate_cri(true);
    if (is_success) {
        std::cout << "CRI activated successfully." << std::endl;
    } else {
        std::cerr << "Failed to activate CRI." << std::endl;
    }
}

void example_is_cri_active(IndyDCP3& indy) {
    bool is_active;
    bool is_success = indy.is_cri_active(is_active);
    if (is_success) {
        std::cout << "CRI is " << (is_active ? "active." : "not active.") << std::endl;
    } else {
        std::cerr << "Failed to check CRI activation status." << std::endl;
    }
}

void example_login_cri_server(IndyDCP3& indy) {
    Nrmk::IndyFramework::Account account;
    account.set_email("user@example.com");
    account.set_token("example_token");

    bool is_success = indy.login_cri_server(account);
    if (is_success) {
        std::cout << "Logged in to CRI server successfully." << std::endl;
    } else {
        std::cerr << "Failed to log in to CRI server." << std::endl;
    }
}

void example_is_cri_login(IndyDCP3& indy) {
    bool is_logged_in;
    bool is_success = indy.is_cri_login(is_logged_in);
    if (is_success) {
        std::cout << "CRI is " << (is_logged_in ? "logged in." : "not logged in.") << std::endl;
    } else {
        std::cerr << "Failed to check CRI login status." << std::endl;
    }
}

void example_set_cri_target(IndyDCP3& indy) {
    Nrmk::IndyFramework::CriTarget target;
    target.set_pn("ProjectName");
    target.set_fn("FunctionName");
    target.set_rn("ResourceName");

    bool is_success = indy.set_cri_target(target);
    if (is_success) {
        std::cout << "CRI Target set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set CRI Target." << std::endl;
    }
}

void example_set_cri_option(IndyDCP3& indy) {
    Nrmk::IndyFramework::State option;
    option.set_enable(true);

    bool is_success = indy.set_cri_option(option);
    if (is_success) {
        std::cout << "CRI Option set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set CRI Option." << std::endl;
    }
}

void example_get_cri_proj_list(IndyDCP3& indy) {
    Nrmk::IndyFramework::ProjectList project_list;
    bool is_success = indy.get_cri_proj_list(project_list);
    if (is_success) {
        std::cout << "CRI Project List retrieved successfully." << std::endl;
        std::cout << "Project List: " << project_list.list() << std::endl;
    } else {
        std::cerr << "Failed to retrieve CRI Project List." << std::endl;
    }
}

void example_get_cri(IndyDCP3& indy) {
    Nrmk::IndyFramework::CriData cri_data;
    bool is_success = indy.get_cri(cri_data);
    if (is_success) {
        std::cout << "CRI data retrieved successfully." << std::endl;
        std::cout << "Time: " << cri_data.time() << ", CRI: " << cri_data.cri() << std::endl;
    } else {
        std::cerr << "Failed to retrieve CRI data." << std::endl;
    }
}

void example_basic_movelf(IndyDCP3& indy) {
    std::array<float, 6> ttarget = {250.0, -150.0, 400.0, 0.0, 180.0, 0.0};
    std::vector<bool> enabledaxis = {true, true, true, false, false, false};
    std::vector<float> desforce = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    int base_type = TaskBaseType::ABSOLUTE_TASK;

    bool is_success = indy.movelf(ttarget, enabledaxis, desforce, base_type);

    if (is_success) {
        std::cout << "Basic MoveLF command executed successfully." << std::endl;
    } else {
        std::cerr << "Basic MoveLF command failed." << std::endl;
    }
}

void example_get_transformed_ft_sensor_data(IndyDCP3& indy) {
    Nrmk::IndyFramework::TransformedFTSensorData ft_sensor_data;
    bool is_success = indy.get_transformed_ft_sensor_data(ft_sensor_data);
    
    if (is_success) {
        std::cout << "Transformed FT Sensor Data retrieved successfully." << std::endl;
        std::cout << "Fx: " << ft_sensor_data.ft_fx() << " N" << std::endl;
        std::cout << "Fy: " << ft_sensor_data.ft_fy() << " N" << std::endl;
        std::cout << "Fz: " << ft_sensor_data.ft_fz() << " N" << std::endl;
        std::cout << "Tx: " << ft_sensor_data.ft_tx() << " N*m" << std::endl;
        std::cout << "Ty: " << ft_sensor_data.ft_ty() << " N*m" << std::endl;
        std::cout << "Tz: " << ft_sensor_data.ft_tz() << " N*m" << std::endl;
    } else {
        std::cerr << "Failed to retrieve Transformed FT Sensor Data." << std::endl;
    }
}

void example_move_joint_traj(IndyDCP3& indy) {
    std::vector<std::vector<float>> q_list = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::vector<std::vector<float>> qdot_list = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::vector<std::vector<float>> qddot_list = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    bool is_success = indy.move_joint_traj(q_list, qdot_list, qddot_list);
    if (is_success) {
        std::cout << "MoveJointTraj command executed successfully." << std::endl;
    } else {
        std::cerr << "MoveJointTraj command failed." << std::endl;
    }
}

void example_move_task_traj(IndyDCP3& indy) {
    std::vector<std::vector<float>> p_list = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::vector<std::vector<float>> pdot_list = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::vector<std::vector<float>> pddot_list = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    bool is_success = indy.move_task_traj(p_list, pdot_list, pddot_list);
    if (is_success) {
        std::cout << "MoveTaskTraj command executed successfully." << std::endl;
    } else {
        std::cerr << "MoveTaskTraj command failed." << std::endl;
    }
}

void example_move_conveyor(IndyDCP3& indy) {

    DCPDICond di_condition;
    di_condition.di.insert({1, true});

    //empty variable condition
    DCPVarCond var_condition;

    bool teaching_mode = false;
    bool bypass_singular = false;
    float acc_ratio = 100.0f;
    bool const_cond = true;
    int cond_type = MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND;
    int react_type = MotionCondition_ReactionType::MotionCondition_ReactionType_STOP_COND;

    bool is_success = indy.move_conveyor(teaching_mode, bypass_singular, acc_ratio, const_cond, cond_type, react_type, di_condition, var_condition);
    
    if (is_success) {
        std::cout << "Conveyor move operation succeeded." << std::endl;
    } else {
        std::cerr << "Conveyor move operation failed." << std::endl;
    }
}

void example_move_axis(IndyDCP3& indy) {
    std::array<float, 3> start_mm = {0.0f, 0.0f, 0.0f}; // Starting position in mm
    std::array<float, 3> target_mm = {100.0f, 50.0f, 0.0f}; // Target position in mm

    bool is_success = indy.move_axis(start_mm, target_mm);
    
    if (is_success) {
        std::cout << "Move axis operation succeeded." << std::endl;
    } else {
        std::cerr << "Move axis operation failed." << std::endl;
    }
}

void example_forward_kin(IndyDCP3& indy) {
    Nrmk::IndyFramework::ForwardKinematicsReq request;
    Nrmk::IndyFramework::ForwardKinematicsRes response;

    //joint positions
    std::vector<float> jpos = {0.0f, 0.0f, -90.0f, 0.0f, -90.0f, 0.0f}; //
    for (const auto& pos : jpos) {
        request.add_jpos(pos);
    }

    bool is_success = indy.forward_kin(request, response);
    
    if (is_success) {
        std::cout << "Forward kinematics calculation succeeded." << std::endl;
        std::cout << "Calculated task positions: ";
        for (int i = 0; i < response.tpos_size(); ++i) {
            std::cout << response.tpos(i) << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Forward kinematics calculation failed." << std::endl;
    }
}

void example_set_tact_time(IndyDCP3& indy) {
    Nrmk::IndyFramework::TactTime tact_time;
    tact_time.set_type("test"); // Example type
    tact_time.set_tact_time(15.5f); // Example tact time in seconds

    bool is_success = indy.set_tact_time(tact_time);
    if (is_success) {
        std::cout << "Tact time set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set tact time." << std::endl;
    }
}

void example_get_tact_time(IndyDCP3& indy) {
    Nrmk::IndyFramework::TactTime tact_time;

    bool is_success = indy.get_tact_time(tact_time);
    if (is_success) {
        std::cout << "Tact time retrieved successfully." << std::endl;
        std::cout << "Type: " << tact_time.type() << std::endl;
        std::cout << "Tact Time: " << tact_time.tact_time() << " seconds" << std::endl;
    } else {
        std::cerr << "Failed to retrieve tact time." << std::endl;
    }
}

void example_set_ft_sensor_config(IndyDCP3& indy) {
    Nrmk::IndyFramework::FTSensorDevice sensor_config;

    // Set sensor configuration
    sensor_config.set_dev_type(Nrmk::IndyFramework::FTSensorDevice::AFT200_D80);
    sensor_config.set_com_type(Nrmk::IndyFramework::FTSensorDevice::ENDTOOLCAN);
    sensor_config.set_ip_address("");
    sensor_config.set_ft_frame_translation_offset_x(00.0f);
    sensor_config.set_ft_frame_translation_offset_y(0.0f);
    sensor_config.set_ft_frame_translation_offset_z(200.0f);
    sensor_config.set_ft_frame_rotation_offset_r(0.0f);
    sensor_config.set_ft_frame_rotation_offset_p(0.0f);
    sensor_config.set_ft_frame_rotation_offset_y(0.0f);

    // sensor_config.set_ft_frame_translation_offset_x(10.0f);
    // sensor_config.set_ft_frame_translation_offset_y(0.0f);
    // sensor_config.set_ft_frame_translation_offset_z(5.0f);
    // sensor_config.set_ft_frame_rotation_offset_r(0.0f);
    // sensor_config.set_ft_frame_rotation_offset_p(0.0f);
    // sensor_config.set_ft_frame_rotation_offset_y(0.0f);

    bool is_success = indy.set_ft_sensor_config(sensor_config);
    if (is_success) {
        std::cout << "FT Sensor configuration set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set FT Sensor configuration." << std::endl;
    }
}

void example_get_ft_sensor_config(IndyDCP3& indy) {
    Nrmk::IndyFramework::FTSensorDevice sensor_config;

    bool is_success = indy.get_ft_sensor_config(sensor_config);
    if (is_success) {
        std::cout << "FT Sensor configuration retrieved successfully." << std::endl;
        std::cout << "Device Type: " << sensor_config.dev_type() << std::endl;
        std::cout << "Communication Type: " << sensor_config.com_type() << std::endl;
        std::cout << "IP Address: " << sensor_config.ip_address() << std::endl;
        std::cout << "Frame Translation Offsets: [" 
                  << sensor_config.ft_frame_translation_offset_x() << ", "
                  << sensor_config.ft_frame_translation_offset_y() << ", "
                  << sensor_config.ft_frame_translation_offset_z() << "]" << std::endl;
        std::cout << "Frame Rotation Offsets: ["
                  << sensor_config.ft_frame_rotation_offset_r() << ", "
                  << sensor_config.ft_frame_rotation_offset_p() << ", "
                  << sensor_config.ft_frame_rotation_offset_y() << "]" << std::endl;
    } else {
        std::cerr << "Failed to retrieve FT Sensor configuration." << std::endl;
    }
}

void example_set_do_config_list(IndyDCP3& indy) {
    Nrmk::IndyFramework::DOConfigList do_config_list;

    Nrmk::IndyFramework::DOConfig* do_config = do_config_list.add_do_configs();
    do_config->set_state_code(2);
    do_config->set_state_name("ExampleState");

    Nrmk::IndyFramework::DigitalSignal* on_signal1 = do_config->add_onsignals();
    on_signal1->set_address(1);
    on_signal1->set_state(Nrmk::IndyFramework::ON_STATE);

    Nrmk::IndyFramework::DigitalSignal* on_signal2 = do_config->add_onsignals();
    on_signal2->set_address(2);
    on_signal2->set_state(Nrmk::IndyFramework::OFF_STATE);

    Nrmk::IndyFramework::DigitalSignal* off_signal1 = do_config->add_offsignals();
    off_signal1->set_address(1);
    off_signal1->set_state(Nrmk::IndyFramework::ON_STATE);

    Nrmk::IndyFramework::DigitalSignal* off_signal2 = do_config->add_offsignals();
    off_signal2->set_address(2);
    off_signal2->set_state(Nrmk::IndyFramework::OFF_STATE);

    bool is_success = indy.set_do_config_list(do_config_list);
    if (is_success) {
        std::cout << "DO configuration list set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set DO configuration list." << std::endl;
    }
}

void example_get_do_config_list(IndyDCP3& indy) {
    Nrmk::IndyFramework::DOConfigList do_config_list;

    bool is_success = indy.get_do_config_list(do_config_list);
    if (is_success) {
        std::cout << "DO configuration list retrieved successfully." << std::endl;

        for (const auto& do_config : do_config_list.do_configs()) {
            std::cout << "State Code: " << do_config.state_code() << std::endl;
            std::cout << "State Name: " << do_config.state_name() << std::endl;

            std::cout << "On Signals:" << std::endl;
            for (const auto& on_signal : do_config.onsignals()) {
                std::cout << "  Address: " << on_signal.address() 
                          << ", State: " << on_signal.state() << std::endl;
            }

            std::cout << "Off Signals:" << std::endl;
            for (const auto& off_signal : do_config.offsignals()) {
                std::cout << "  Address: " << off_signal.address() 
                          << ", State: " << off_signal.state() << std::endl;
            }
        }
    } else {
        std::cerr << "Failed to retrieve DO configuration list." << std::endl;
    }
}

void example_move_recover_joint(IndyDCP3& indy) {
    std::vector<float> jtarget = {0.0f, -45.0f, 90.0f, -90.0f, 45.0f, 0.0f};
    int base_type = JointBaseType::ABSOLUTE_JOINT;

    bool is_success = indy.move_recover_joint(jtarget, base_type);
    if (is_success) {
        std::cout << "Move recover joint operation succeeded." << std::endl;
    } else {
        std::cerr << "Move recover joint operation failed." << std::endl;
    }
}

void example_get_control_info(IndyDCP3& indy) {
    Nrmk::IndyFramework::ControlInfo control_info;

    bool is_success = indy.get_control_info(control_info);
    if (is_success) {
        std::cout << "Control info retrieved successfully." << std::endl;
        std::cout << "Control Version: " << control_info.control_version() << std::endl;
        std::cout << "Robot Model: " << control_info.robot_model() << std::endl;
    } else {
        std::cerr << "Failed to retrieve control info." << std::endl;
    }
}

void example_check_aproach_retract_valid(IndyDCP3& indy) {
    std::array<float, 6> tpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> init_jpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 6> pre_tpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 6> post_tpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    Nrmk::IndyFramework::CheckAproachRetractValidRes response;
    bool is_success = indy.check_aproach_retract_valid(tpos, init_jpos, pre_tpos, post_tpos, response);
    
    if (is_success) {
        std::cout << "Aproach and Retract Valid." << std::endl;
    } else {
        std::cerr << "Aproach and Retract check failed." << std::endl;
    }
}

void example_get_pallet_point_list(IndyDCP3& indy) {
    std::array<float, 6> tpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> jpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 6> pre_tpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 6> post_tpos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int pallet_pattern = 1;
    int width = 5;
    int height = 5;

    Nrmk::IndyFramework::GetPalletPointListRes response;
    bool is_success = indy.get_pallet_point_list(tpos, jpos, pre_tpos, post_tpos, pallet_pattern, width, height, response);
    
    if (is_success) {
        std::cout << "Pallet Point List retrieved successfully." << std::endl;
        for (const auto& point : response.pallet_points()) {
            std::cout << "Target Position: ";
            for (const auto& pos : point.tar_pos()) {
                std::cout << pos << " ";
            }
            std::cout << std::endl;
            // TODO: other attributes
        }
    } else {
        std::cerr << "Failed to retrieve Pallet Point List." << std::endl;
    }
}

void example_play_tuning_program(IndyDCP3& indy) {
    Nrmk::IndyFramework::CollisionThresholds response;
    bool is_success = indy.play_tuning_program("example_program", 1, 
                                               Nrmk::IndyFramework::TuningSpace::TUNE_ALL, 
                                               Nrmk::IndyFramework::TuningPrecision::HIGH_PRECISION, 
                                               9, response);
    
    if (is_success) {
        std::cout << "Tuning program played successfully." << std::endl;
        // TODO: Output CollisionThresholds data
    } else {
        std::cerr << "Failed to play tuning program." << std::endl;
    }
}

void example_set_di_config_list(IndyDCP3& indy) {
    Nrmk::IndyFramework::DIConfigList di_config_list;

    auto* di_config = di_config_list.add_di_configs();
    di_config->set_function_code(2);
    di_config->set_function_name("Example Function");
    
    auto* trigger_signal = di_config->add_triggersignals();
    trigger_signal->set_address(1);
    trigger_signal->set_state(Nrmk::IndyFramework::DigitalState::ON_STATE);

    bool is_success = indy.set_di_config_list(di_config_list);
    
    if (is_success) {
        std::cout << "DI config list set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set DI config list." << std::endl;
    }
}

void example_get_di_config_list(IndyDCP3& indy) {
    Nrmk::IndyFramework::DIConfigList di_config_list;
    bool is_success = indy.get_di_config_list(di_config_list);
    
    if (is_success) {
        std::cout << "DI config list retrieved successfully." << std::endl;
        for (const auto& config : di_config_list.di_configs()) {
            std::cout << "Function Code: " << config.function_code() 
                      << ", Function Name: " << config.function_name() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve DI config list." << std::endl;
    }
}

void example_set_auto_servo_off(IndyDCP3& indy) {
    Nrmk::IndyFramework::AutoServoOffConfig config;
    config.set_enable(true);
    config.set_time(60.0f); // 60 seconds

    bool is_success = indy.set_auto_servo_off(config);
    
    if (is_success) {
        std::cout << "Auto Servo Off configuration set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set Auto Servo Off configuration." << std::endl;
    }
}

void example_get_auto_servo_off(IndyDCP3& indy) {
    Nrmk::IndyFramework::AutoServoOffConfig config;
    bool is_success = indy.get_auto_servo_off(config);
    
    if (is_success) {
        std::cout << "Auto Servo Off configuration retrieved successfully." << std::endl;
        std::cout << "Enable: " << (config.enable() ? "True" : "False") << std::endl;
        std::cout << "Time: " << config.time() << " seconds" << std::endl;
    } else {
        std::cerr << "Failed to retrieve Auto Servo Off configuration." << std::endl;
    }
}

void example_set_safety_stop_config(IndyDCP3& indy) {
    Nrmk::IndyFramework::SafetyStopConfig config;
    config.set_joint_position_limit_stop_cat(Nrmk::IndyFramework::StopCategory::IMMEDIATE_BRAKE);
    config.set_joint_speed_limit_stop_cat(Nrmk::IndyFramework::StopCategory::SMOOTH_BRAKE);
    config.set_joint_torque_limit_stop_cat(Nrmk::IndyFramework::StopCategory::SMOOTH_ONLY);
    config.set_tcp_speed_limit_stop_cat(Nrmk::IndyFramework::StopCategory::IMMEDIATE_BRAKE);
    config.set_tcp_force_limit_stop_cat(Nrmk::IndyFramework::StopCategory::SMOOTH_BRAKE);
    config.set_power_limit_stop_cat(Nrmk::IndyFramework::StopCategory::SMOOTH_ONLY);

    bool is_success = indy.set_safety_stop_config(config);
    
    if (is_success) {
        std::cout << "Safety stop configuration set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set safety stop configuration." << std::endl;
    }
}

void example_get_safety_stop_config(IndyDCP3& indy) {
    Nrmk::IndyFramework::SafetyStopConfig config;
    bool is_success = indy.get_safety_stop_config(config);
    
    if (is_success) {
        std::cout << "Safety stop configuration retrieved successfully." << std::endl;
        std::cout << "Joint Position Limit Stop Category: " << config.joint_position_limit_stop_cat() << std::endl;
        std::cout << "Joint Speed Limit Stop Category: " << config.joint_speed_limit_stop_cat() << std::endl;
        std::cout << "Joint Torque Limit Stop Category: " << config.joint_torque_limit_stop_cat() << std::endl;
        std::cout << "TCP Speed Limit Stop Category: " << config.tcp_speed_limit_stop_cat() << std::endl;
        std::cout << "TCP Force Limit Stop Category: " << config.tcp_force_limit_stop_cat() << std::endl;
        std::cout << "Power Limit Stop Category: " << config.power_limit_stop_cat() << std::endl;
    } else {
        std::cerr << "Failed to retrieve safety stop configuration." << std::endl;
    }
}

void example_get_reduced_ratio(IndyDCP3& indy) {
    float ratio;
    bool is_success = indy.get_reduced_ratio(ratio);
    
    if (is_success) {
        std::cout << "Reduced ratio retrieved successfully: " << ratio << std::endl;
    } else {
        std::cerr << "Failed to retrieve reduced ratio." << std::endl;
    }
}

void example_get_reduced_speed(IndyDCP3& indy) {
    float speed;
    bool is_success = indy.get_reduced_speed(speed);
    
    if (is_success) {
        std::cout << "Reduced speed retrieved successfully: " << speed << " mm/s" << std::endl;
    } else {
        std::cerr << "Failed to retrieve reduced speed." << std::endl;
    }
}

void example_set_reduced_speed(IndyDCP3& indy) {
    float speed = 50.0f; // Example speed in mm/s
    bool is_success = indy.set_reduced_speed(speed);
    
    if (is_success) {
        std::cout << "Reduced speed set successfully to " << speed << " mm/s" << std::endl;
    } else {
        std::cerr << "Failed to set reduced speed." << std::endl;
    }
}

void example_set_teleop_params(IndyDCP3& indy) {
    Nrmk::IndyFramework::TeleOpParams request;
    request.set_smooth_factor(0.5f);
    request.set_cutoff_freq(10.0f);
    request.set_error_gain(1.0f);
    
    bool is_success = indy.set_teleop_params(request);
    
    if (is_success) {
        std::cout << "TeleOp parameters set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set TeleOp parameters." << std::endl;
    }
}

void example_get_teleop_params(IndyDCP3& indy) {
    Nrmk::IndyFramework::TeleOpParams response;
    bool is_success = indy.get_teleop_params(response);
    
    if (is_success) {
        std::cout << "TeleOp parameters retrieved successfully." << std::endl;
        std::cout << "Smooth Factor: " << response.smooth_factor() << std::endl;
        std::cout << "Cutoff Frequency: " << response.cutoff_freq() << " Hz" << std::endl;
        std::cout << "Error Gain: " << response.error_gain() << std::endl;
    } else {
        std::cerr << "Failed to retrieve TeleOp parameters." << std::endl;
    }
}

void example_get_kinematics_params(IndyDCP3& indy) {
    Nrmk::IndyFramework::KinematicsParams response;
    bool is_success = indy.get_kinematics_params(response);

    if (is_success) {
        std::cout << "Kinematics parameters retrieved successfully." << std::endl;
        for (const auto& mdh : response.mdh()) {
            std::cout << "MDH Parameters - a: " << mdh.a() << ", alpha: " << mdh.alpha()
                      << ", d0: " << mdh.d0() << ", theta0: " << mdh.theta0()
                      << ", type: " << (mdh.type() == Nrmk::IndyFramework::KinematicsParams_JointType_REVOLUTE ? "Revolute" : "Prismatic")
                      << ", index: " << mdh.index() << ", parent: " << mdh.parent() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve kinematics parameters." << std::endl;
    }
}

void example_get_io_data(IndyDCP3& indy) {
    Nrmk::IndyFramework::IOData response;
    bool is_success = indy.get_io_data(response);

    if (is_success) {
        std::cout << "IO Data retrieved successfully." << std::endl;

        std::cout << "Digital Inputs:" << std::endl;
        for (const auto& di : response.di()) {
            std::cout << "Address: " << di.address() << ", State: " << di.state() << std::endl;
        }

        std::cout << "Digital Outputs:" << std::endl;
        for (const auto& dout : response.do_()) {
            std::cout << "Address: " << dout.address() << ", State: " << dout.state() << std::endl;
        }

        std::cout << "Analog Inputs:" << std::endl;
        for (const auto& ai : response.ai()) {
            std::cout << "Address: " << ai.address() << ", Voltage: " << ai.voltage() << std::endl;
        }

        std::cout << "Analog Outputs:" << std::endl;
        for (const auto& ao : response.ao()) {
            std::cout << "Address: " << ao.address() << ", Voltage: " << ao.voltage() << std::endl;
        }

        std::cout << "Endtool Digital Inputs:" << std::endl;
        for (const auto& end_di : response.end_di()) {
            std::cout << "Port: " << end_di.port() << std::endl;
            for (const auto& state : end_di.states()) {
                std::cout << "State: " << state << std::endl;
            }
        }

        std::cout << "Endtool Digital Outputs:" << std::endl;
        for (const auto& end_do : response.end_do()) {
            std::cout << "Port: " << end_do.port() << std::endl;
            for (const auto& state : end_do.states()) {
                std::cout << "State: " << state << std::endl;
            }
        }
    } else {
        std::cerr << "Failed to retrieve IO data." << std::endl;
    }
}

