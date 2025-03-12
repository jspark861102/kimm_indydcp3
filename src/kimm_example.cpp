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
    n_node.getParam("/load_robotiq_ft_sensor", load_robotiq_ft_sensor_);    

    // husky_ctrl_pub_.init(n_node, "cmd_vel", 4);    
    husky_ctrl_pub_ = n_node.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    joint_states_pub_ = n_node.advertise<sensor_msgs::JointState>("joint_states", 5);
    ft_calibrated_data_pub_ = n_node.advertise<geometry_msgs::Wrench>("ft_calibrated_data", 5);
    ft_link0_data_pub_ = n_node.advertise<geometry_msgs::Wrench>("ft_link0_data", 5);    
    teleop_joy_subs_ = n_node.subscribe("joy_teleop/joy", 1, &teleopjoyCallback);                
    mode_pub_ = n_node.advertise<std_msgs::Int16>("kimm_mode", 5);
    whisper_subs_ = n_node.subscribe("recognized_word", 1, &whisperCallback);                
    whisper_pub_ = n_node.advertise<std_msgs::Int16>("whisper_feedback", 5);

    // Indy7
    IndyDCP3 indy("192.168.10.9");        

    //joint_state    
    joint_states_msg_.name = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};    
    joint_states_msg_.position.resize(6);
    joint_states_msg_.velocity.resize(6);

    // Robotiq gripper  
    isgrasp_ = false;     
    cout << "load_gripper_:" << load_gripper_ << endl; 
    if(load_gripper_){    
        gripper_robotiq_ = new robotiq_2f_gripper_control::RobotiqActionClient("/command_robotiq_action", true);      
        robotiq_state_subs_ = n_node.subscribe( "/robotiq/joint_states", 1, &robotiqstateCallback);                
    }
    
    cout << "load_robotiq_ft_sensor_:" << load_robotiq_ft_sensor_ << endl; 
    if(load_robotiq_ft_sensor_){
        robotiq_ft_link0_data_pub_ = n_node.advertise<geometry_msgs::Wrench>("robotiq_ft_link0_data", 5);    
        robotiq_ft_sensor_subs_ = n_node.subscribe("robotiq_ft_wrench", 1, &robotiqftsensorCallback);   
        robotiq_ft_reset_pub_ = n_node.advertise<std_msgs::Bool>("robotiq_ft_sensor_reset", 5);    
    }

    // keyboard event, this code begins from here  
    std::thread mode_change_thread_;
    mode_change_thread_ = std::thread(&keyboardReaderProc);

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
    mode_ = 0;
    mode_sum = 0;
    mobile_teaching_trigger_ = false;
    direct_teaching_trigger_ = false;
    is_published_whisper_ = false;
    mobile_mode_by_whisper_ = 1;

    // example_get_device_info(indy);
    // example_get_int_variables(indy);        

    // loop
    while (ros::ok()) {
        // command
        modeChange_event(indy);

        // pub
        call_IndyData(indy);     
        
        //ranger mobile teaching
        if(mobile_teaching_trigger_) {
            if(can_i_do_mobile_teaching_){  
                if (!is_published_whisper_) {
                    std_msgs::Int16 feedback;
                    feedback.data = 2; //"mobile teaching start"
                    whisper_pub_.publish(feedback);
                    mobile_mode_by_whisper_ = 1; //basic mode is parallel
                    is_published_whisper_ = true;
                }
                
                mobile_teaching();                
            }
            else {
                cout << "please calibrate ft_sensor data" << endl;
                if (!is_published_whisper_) {
                    std_msgs::Int16 feedback;
                    feedback.data = 1; //"please calibrate ft_sensor data"
                    whisper_pub_.publish(feedback);
                    mobile_mode_by_whisper_ = 1; //basic mode is parallel
                    is_published_whisper_ = true;
                }
            }
        }        

        ros::spinOnce();
        loop_rate.sleep();
    }//while
    
    return 0;
}

void keyboardReaderProc(){  

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
            break;                         
        case 'g': //move pose
            msg_ = 2;                         
            break;  
        case 'i': //forward jog
            msg_ = 10;                         
            break;  
        case 'k': //backward jog
            msg_ = 11;                         
            break;  
        case 'j': //left jog
            msg_ = 12;                         
            break;  
        case 'l': //right jog
            msg_ = 13;                         
            break; 
        case 'u': //up jog
            msg_ = 14;                         
            break; 
        case 'o': //down jog
            msg_ = 15;                         
            break;     
        case '[': //enable direct teaching
            msg_ = 16;                         
            break;
        case ']': //disable direct teaching
            msg_ = 17;                         
            break;                

        case 'q': //actvie sdk
            msg_ = 20;                         
            break;
        case 'a': //get custom control mode
            msg_ = 21;                         
            break;
        case 'w': //set custom control mode
            msg_ = 22;                         
            break;            
        case 's': //set basic control mode
            msg_ = 23;                         
            break;            
        case 'e': //get custom gain
            msg_ = 24;                         
            break;
        case 'd': //set custom compliance gain 
            msg_ = 25;                         
            break;
        case 'c': //set basic impedance gain
            msg_ = 26;                         
            break;
            
        case 'p': //recover
            msg_ = 30;                         
            break;     
        case '1': //stop
            msg_ = 31;                         
            break;                 
        case '2': //mobile teaching
            if (mobile_teaching_trigger_) {            
                std::cout << " " << std::endl;
                std::cout << "end mobile teaching" << std::endl;
                std::cout << " " << std::endl;     
                mobile_teaching_trigger_ = false;     
                can_i_do_mobile_teaching_ = false;                                           
            }       
            else {     
                std::cout << " " << std::endl;
                std::cout << "start mobile teaching" << std::endl;
                std::cout << " " << std::endl;     
                mobile_teaching_trigger_ = true;
            }
            break;       
        case '3': //indy teaching            
            msg_ = 32;                         
            break;    

        case 'm': //get ft sensor data
            msg_ = 40;                         
            break;     
        case 'n': //ft sensor data calibration
            msg_ = 41;                              
            break;             
        case '9': //log start
            msg_ = 42;                          
            break;         
        case '0': //log end
            msg_ = 43;                           
            break;                      
                
        case 'z': //gripper
            if (isgrasp_){
                cout << "Release hand" << endl;                
                isgrasp_ = false;
                // gripper_robotiq_->open(false);
                gripper_robotiq_->goToPosition(0.05,0.1,220,false);
            }
            else{
                cout << "Grasp object" << endl;                                
                isgrasp_ = true; 
                gripper_robotiq_->close(0.4, 220, false);                
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

void modeChange_event(IndyDCP3& indy){        
    if(msg_ == 1){ //h
        //home
        std::cout << " " << std::endl;
        std::cout << "home position" << std::endl;
        std::cout << " " << std::endl;          

        std::vector<float> j_pos_3 = {0.0, -45.0, -135.0, 0.0, 90.0, 0.0};
        // std::vector<float> j_pos_3 = {0.0, -45.0, -135.0, 0.0, 90.0, 180.0};
        indy.movej(j_pos_3, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);                    
    
        msg_ = 0;
    }
    else if(msg_ == 2){ //g
        //moveo pos
        std::cout << " " << std::endl;
        std::cout << "move pose" << std::endl;
        std::cout << " " << std::endl;          

        std::vector<float> j_pos_4 = {0.0, 90.0, -90.0, 0.0, -90.0, 0.0};
        indy.movej(j_pos_4, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);                    
    
        msg_ = 0;
    }
    else if(msg_ ==10){ //i
        //forward jog
        std::cout << " " << std::endl;
        std::cout << "forward jog" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance(indy);

        // std::array<float, 6> t_pos = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        // indy.movel(t_pos, TaskBaseType::RELATIVE_TASK); 

        std::vector<float> jpos = set_jog_jpos(indy, 0, 100.0);        
        indy.movej(jpos, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);                               
    
        msg_ = 0;
    }  
    else if(msg_ ==11){ //k
        //backward jog
        std::cout << " " << std::endl;
        std::cout << "backward jog" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance(indy);

        // std::array<float, 6> t_pos = {-100.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        // indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);  

        std::vector<float> jpos = set_jog_jpos(indy, 0, -100.0);        
        indy.movej(jpos, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);                              

        msg_ = 0;
    }  
    else if(msg_ ==12){ //j
        //left jog
        std::cout << " " << std::endl;
        std::cout << "left jog" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance(indy);

        // std::array<float, 6> t_pos = {0.0, 100.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        // indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);

        std::vector<float> jpos = set_jog_jpos(indy, 1, 100.0);        
        indy.movej(jpos, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);                        
    
        msg_ = 0;
    }  
    else if(msg_ ==13){ //l
        //right jog
        std::cout << " " << std::endl;
        std::cout << "right jog" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance(indy);

        // std::array<float, 6> t_pos = {0.0, -100.0, 0.0, 0.0, 0.0, 0.0}; //[mm]
        // indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);    

        std::vector<float> jpos = set_jog_jpos(indy, 1, -100.0);        
        indy.movej(jpos, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);                    
    
        msg_ = 0;
    }  
    else if(msg_ ==14){ //u
        //up jog
        std::cout << " " << std::endl;
        std::cout << "up jog" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance(indy);

        // std::array<float, 6> t_pos = {0.0, 0.0, 100.0, 0.0, 0.0, 0.0}; //[mm]
        // indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);            

        std::vector<float> jpos = set_jog_jpos(indy, 2, 100.0);        
        indy.movej(jpos, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);                    
    
        msg_ = 0;
    }  
    else if(msg_ ==15){ //o
        //down jog
        std::cout << " " << std::endl;
        std::cout << "down jog" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance(indy);

        // std::array<float, 6> t_pos = {0.0, 0.0, -100.0, 0.0, 0.0, 0.0}; //[mm]
        // indy.movel(t_pos, TaskBaseType::RELATIVE_TASK);            

        // std::vector<float> jpos = set_jog_jpos(indy, 2, -100.0);        
        // std::vector<float> jpos = set_jog_jpos(indy, 2, -200.0);      

        //over 400 makes inverse kinematics problem so, down 225 twice
        std::vector<float> jpos = set_jog_jpos(indy, 2, -225.0);          
        indy.movej(jpos, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0);     

        // jpos = set_jog_jpos(indy, 2, -225.0);          
        // indy.movej(jpos, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_OVERRIDE, 0.0, 15.0); 
        
        msg_ = 0;
    }  
    else if(msg_ ==16){ //[
        //enable direct teaching
        std::cout << " " << std::endl;
        std::cout << "enable direct teaching" << std::endl;
        std::cout << " " << std::endl;          

        example_enable_direct_teaching(indy, true);
        
        msg_ = 0;
    }
    else if(msg_ ==17){ //]
        //disable direct teaching
        std::cout << " " << std::endl;
        std::cout << "disable direct teaching" << std::endl;
        std::cout << " " << std::endl;          
        
        example_enable_direct_teaching(indy, false);
        
        msg_ = 0;
    }    
    else if(msg_ ==20){ //q
        //actvie sdk
        std::cout << " " << std::endl;
        std::cout << "actvie sdk" << std::endl;
        std::cout << " " << std::endl;          

        example_activate_sdk(indy);
    
        msg_ = 0;
    }
    else if(msg_ ==21){ //a
        //get custom control mode
        std::cout << " " << std::endl;
        std::cout << "get custom control mode" << std::endl;
        std::cout << " " << std::endl;          

        example_get_custom_control_mode(indy);        
        
        msg_ = 0;
    }       
    else if(msg_ ==22){ //w
        //set custom control mode
        std::cout << " " << std::endl;
        std::cout << "set custom control mode" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_mode(indy, 1);
        
        msg_ = 0;
    }
    else if(msg_ ==23){ //s
        //set basic control mode
        std::cout << " " << std::endl;
        std::cout << "set basic control mode" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_mode(indy, 0);
        
        msg_ = 0;
    }
    else if(msg_ ==24){ //e
        //get custom gain
        std::cout << " " << std::endl;
        std::cout << "get custom gain" << std::endl;
        std::cout << " " << std::endl;          

        example_get_custom_control_gain(indy);
        
        msg_ = 0;
    }
    else if(msg_ ==25){ //d  
        //set custom compliance gain      
        std::cout << " " << std::endl;
        std::cout << "set custom compliance gain " << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance_compliance(indy);
        
        msg_ = 0;
    }
    else if(msg_ ==26){ //c
        //set basic impedance gain
        std::cout << " " << std::endl;
        std::cout << "set basic impedance gain" << std::endl;
        std::cout << " " << std::endl;          

        example_set_custom_control_gain_impedance(indy);
        // example_set_custom_control_gain_admittance(indy);
        
        msg_ = 0;
    }    
    
    else if(msg_ ==30){ //p
        //recover
        std::cout << " " << std::endl;
        std::cout << "recover" << std::endl;
        std::cout << " " << std::endl;          

        example_recover_robot(indy);
        
        msg_ = 0;
    }
    else if(msg_ ==31){ //1
        //stop
        std::cout << " " << std::endl;
        std::cout << "stop" << std::endl;
        std::cout << " " << std::endl;          

        example_stop_robot_motion(indy);    
        
        msg_ = 0;
    }  
    else if(msg_ ==32){ //3
        //indy teaching
        std::cout << " " << std::endl;
        std::cout << "indy teaching" << std::endl;
        std::cout << " " << std::endl;          

        std::cout << " " << std::endl;
        std::cout << "indy teaching is unavailable now for safety" << std::endl;
        std::cout << " " << std::endl;          

        /*
        example_set_custom_control_gain_impedance_compliance(indy);

        std::array<float, 6> t_pos = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //[mm]        
        bool is_success = indy.movel_time(t_pos, TaskBaseType::RELATIVE_TASK, BlendingType_Type::BlendingType_Type_NONE, 0.0, 
                                                10.0); //sec       
        */
        msg_ = 0;
    }                                     

    else if(msg_ ==40){ //m
        //get ft sensor data
        std::cout << " " << std::endl;
        std::cout << "get ft sensor data" << std::endl;
        std::cout << " " << std::endl;          

        example_get_ft_sensor_data(indy);
        // tf_print(indy);
        // example_get_tool_properties(indy);
        
        msg_ = 0;
    }
    else if(msg_ ==41){ //n
        //ft sensor data calibration
        std::cout << " " << std::endl;
        std::cout << "ft sensor data calibration" << std::endl;
        std::cout << " " << std::endl;     

        ft_sensor_calibration(indy);

        if(load_robotiq_ft_sensor_){        
            std_msgs::Bool is_robotiq_ft_reset;
            is_robotiq_ft_reset.data = true;
            robotiq_ft_reset_pub_.publish(is_robotiq_ft_reset);
        }
        
        msg_ = 0;
    }    
    else if(msg_ ==42){ //9
        //log start
        std::cout << " " << std::endl;
        std::cout << "log start" << std::endl;
        std::cout << " " << std::endl;         

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
    else if(msg_ ==43){ //0
        //log end
        std::cout << " " << std::endl;
        std::cout << "log end" << std::endl;
        std::cout << " " << std::endl;        

        example_end_log(indy);
        // data_log_end(indy);
        
        msg_ = 0;
    }    
    //msg_ = 99 : gripper
}


std::vector<float> set_jog_jpos(IndyDCP3& indy, int index, double jog) {
    std::vector<float> jpos;    

    Nrmk::IndyFramework::ControlData control_data;    
    bool is_success = indy.get_robot_data(control_data);            
    if (is_success){        
        std::vector<float> init_jpos = {control_data.q(0), control_data.q(1), control_data.q(2), control_data.q(3), control_data.q(4), control_data.q(5)};        
        std::array<float, 6> tpos = {control_data.p(0), control_data.p(1), control_data.p(2), control_data.p(3), control_data.p(4), control_data.p(5)}; //[mm]    
        // std::cout << "here1" << std::endl;                
        tpos[index] += jog; //[mm]    
        bool is_ik_success = indy.inverse_kin(tpos, init_jpos, jpos);    
        // std::cout << "here2" << is_ik_success << std::endl;                
        if (is_ik_success) {
            std::cout << init_jpos[0] << " " << init_jpos[1] << " " << init_jpos[2] << " " << init_jpos[3] << " " << init_jpos[4] << " " << init_jpos[5] << " " << std::endl;
            std::cout << jpos[0] << " " << jpos[1] << " " << jpos[2] << " " << jpos[3] << " " << jpos[4] << " " << jpos[5] << " " << std::endl;        
        }        
        else {
            jpos = init_jpos;
            cout << "inverse kinematics is not successed." << endl;
            // std::cout << "here3" << std::endl;                
        }
    }             
    else {        
        std::cout << "Failed to get_control_data" << std::endl;
        jpos = {0.0, 0.0, 90.0, 0.0, 90.0, 0.0}; //home
    } 
    // std::cout << "here4" << std::endl;                
    return jpos;
}

void tf_print(IndyDCP3& indy){        

    bool is_success;
    Nrmk::IndyFramework::ControlData control_data;
    is_success = indy.get_robot_data(control_data);    

    Eigen::AngleAxisd rollAngle(3.14, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(3.14, Eigen::Vector3d::UnitZ());

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

void whisperCallback(const std_msgs::Int16 &msg) {      
    msg_ = msg.data;

    if(msg_ == 97){ //direct teaching
        if (direct_teaching_trigger_) {            
            std::cout << " " << std::endl;
            std::cout << "end direct teaching" << std::endl;
            std::cout << " " << std::endl;     
            direct_teaching_trigger_ = false;
            
            std_msgs::Int16 feedback;
            feedback.data = 7; //"direct teach end"
            whisper_pub_.publish(feedback);                                

            msg_ = 17;
        }       
        else {     
            std::cout << " " << std::endl;
            std::cout << "start direct teaching" << std::endl;
            std::cout << " " << std::endl;     
            direct_teaching_trigger_ = true;

            std_msgs::Int16 feedback;
            feedback.data = 6; //"direct teach start"
            whisper_pub_.publish(feedback);                                

            msg_ = 16;
        }
    }
    if(msg_ == 98){ //mobile teaching
        if (mobile_teaching_trigger_) {            
            std::cout << " " << std::endl;
            std::cout << "end mobile teaching" << std::endl;
            std::cout << " " << std::endl;     
            mobile_teaching_trigger_ = false;
            can_i_do_mobile_teaching_ = false;
            is_published_whisper_ = false;
            
            std_msgs::Int16 feedback;
            feedback.data = 3; //"mobile teaching end"
            whisper_pub_.publish(feedback);                                
        }       
        else {     
            std::cout << " " << std::endl;
            std::cout << "start mobile teaching" << std::endl;
            std::cout << " " << std::endl;     
            mobile_teaching_trigger_ = true;
        }
    }
    if(msg_ == 61) {
        mobile_mode_by_whisper_ = 1;

        std_msgs::Int16 feedback;
        feedback.data = 11; //"parallel"
        whisper_pub_.publish(feedback);    
    }
    else if(msg_ == 62) {
        mobile_mode_by_whisper_ = 2;

        std_msgs::Int16 feedback;
        feedback.data = 12; //"spin"
        whisper_pub_.publish(feedback);    
    }
    
    if(msg_ == 99){ //gripper
        if (isgrasp_){
            cout << "Release hand" << endl;

            std_msgs::Int16 feedback;
            feedback.data = 5; //"gripper released"
            whisper_pub_.publish(feedback);    

            isgrasp_ = false;
            // gripper_robotiq_->open(false);
            gripper_robotiq_->goToPosition(0.05,0.1,220,false);
        }
        else{
            cout << "Grasp object" << endl;
            
            std_msgs::Int16 feedback;
            feedback.data = 4; //"gripper closed"
            whisper_pub_.publish(feedback);    

            isgrasp_ = true; 
            gripper_robotiq_->close(0.4, 220, false);                
        }
    }        
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
    double teaching_vx, teaching_vy, teaching_w;    
    double teaching_vx_thr, teaching_vy_thr, teaching_w_thr;
    
    //-------------------------------------------------------------------------------------------------------------------------------//
    // without diagonal mode, perforamcne is guaranteed -----------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------------------------------------------------------//
    //  qvel = D^(-1) * torque_base : aidin AFT200-D80
    teaching_vx = ft_link0_filtered_data_for_mobile_(0) * 0.01; //force_x                
    teaching_vy = -ft_link0_filtered_data_for_mobile_(3) * 0.4;  //torque_x       
    teaching_w = ft_link0_filtered_data_for_mobile_(5) * 0.4;    //torque_z   

    int mode_dummy;
    mode_dummy = 0;
    
    teaching_vx_thr = 0.06;
    teaching_vy_thr = 0.5;
    teaching_w_thr = 0.15; //*With the KONA cover, teaching_w is hard to apply enough, so lowering the threshold*//

    //*With the KONA cover, undesired large teaching_vx is generated during both parallel mode and spin mode.*// 
    //*Therefore, teaching_vx is not used when detecting spin mode, and diagonal mode is not implemented in parallel mode*//

    // if(fabs(teaching_w) > teaching_w_thr && fabs(teaching_w) > fabs(teaching_vy) && fabs(teaching_w) > fabs(teaching_vx)) {
    if(fabs(teaching_w) > teaching_w_thr && fabs(teaching_w) > fabs(teaching_vy)) { //*teaching_vx is not used*//       
        mode_sum += 1;        
        //*To maintain spin mode even if teaching_vy is inconsistent, mode_sum has been adjusted from 100 to 200*//
        if(mode_sum >= 200) mode_sum = 200; 
    }    
    else if(( fabs(teaching_vy) > teaching_vy_thr || fabs(teaching_vx) > teaching_vx_thr ) && ( fabs(teaching_vy) > fabs(teaching_w) || fabs(teaching_vx) > fabs(teaching_w) )) {
        mode_sum -= 1;
        if(mode_sum <= 0) mode_sum = 0;
    }
    else{  //*to make default mode as parallel mode*//
        mode_sum -= 1;
        if(mode_sum <= 0) mode_sum = 0; 
    }

    // if(mode_sum >= 50) mode_ = 2; //spining
    // else mode_ = 1; //parallel

    if(mobile_mode_by_whisper_ == 1) {
        mode_ = 1; //parallel
    }  
    else if(mobile_mode_by_whisper_ == 2) {
        mode_ = 2; //spining
    }  

    if(mode_ == 1){ //parallel
        if(fabs(teaching_vy) > teaching_vy_thr){        
            //*Due to undesired teaching_vx with the KONA cover, diagonal mode is not implemented*//
            // if(fabs(teaching_vx) < teaching_vx_thr){
            //     teaching_vx = 0.0;    
            //     mode_dummy = 1;
            // }
            // else { //fabs(teaching_vx) > teaching_vx_thr, to prevent steped command
            //     if(teaching_vx >0) teaching_vx = teaching_vx - teaching_vx_thr;
            //     else               teaching_vx = teaching_vx + teaching_vx_thr;
            //     mode_dummy = 2;
            // }       
            teaching_vx = 0.0;    
            mode_dummy = 1;     
        }
        else {
            teaching_vy = 0.0;
            if(fabs(teaching_vx) < teaching_vx_thr){
                teaching_vx = 0.0;    
                mode_dummy = 3;
            }
            else { //fabs(teaching_vx) > teaching_vx_thr, to prevent steped command
                if(teaching_vx >0) teaching_vx = teaching_vx - teaching_vx_thr;
                else               teaching_vx = teaching_vx + teaching_vx_thr;
                mode_dummy = 4;
            }                
        }
        teaching_w = 0.0;    
    }
    else{ //spinning, mode_=2
        teaching_vy = 0.0;    
        teaching_vx = 0.0;    
        mode_dummy = 5;
    }
    //-------------------------------------------------------------------------------------------------------------------------------//


    //-------------------------------------------------------------------------------------------------------------------------------//
    // without diagonal mode, perforamcne is guaranteed -----------------------------------------------------------------------------//
    // CANNOT OVERCOME THE THRESHOLD OF TEACHING_VX BY HUMAN!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ---> CANNOT BE USED -----------------------//
    //-------------------------------------------------------------------------------------------------------------------------------//    
    // //  qvel = D^(-1) * torque_base : aidin AFT200-D80
    // teaching_vx = ft_link0_filtered_data_for_mobile_(0) * 0.01; //force_x                
    // teaching_vy = -ft_link0_filtered_data_for_mobile_(3) * 0.4;  //torque_x       
    // teaching_w = ft_link0_filtered_data_for_mobile_(5) * 0.8;    //torque_z, to exceed teacihng_vx value in spin detection mode
    
    // int mode_dummy;
    // mode_dummy = 0;
    
    // teaching_vx_thr = 0.5; //to neglect undesired teaching_vx (aroung +/35N)
    // teaching_vy_thr = 0.5;
    // teaching_w_thr = 0.3; //*doubled due to 0.8 weight*//

    // if(fabs(teaching_w) > teaching_w_thr && fabs(teaching_w) > fabs(teaching_vy) && fabs(teaching_w) > fabs(teaching_vx)) {        
    //     mode_sum += 1;        
    //     //*To maintain spin mode even if teaching_vy is inconsistent, mode_sum has been adjusted from 100 to 200*//
    //     if(mode_sum >= 200) mode_sum = 200; 
    // }    
    // else if(( fabs(teaching_vy) > teaching_vy_thr || fabs(teaching_vx) > teaching_vx_thr ) && ( fabs(teaching_vy) > fabs(teaching_w) || fabs(teaching_vx) > fabs(teaching_w) )) {
    //     mode_sum -= 1;
    //     if(mode_sum <= 0) mode_sum = 0;
    // }
    // else{  //*to make default mode as parallel mode*//
    //     mode_sum -= 1;
    //     if(mode_sum <= 0) mode_sum = 0; 
    // }

    // if(mode_sum >= 50) mode_ = 2; //spining
    // else mode_ = 1; //parallel

    // if(mode_ == 1){ //parallel
    //     if(fabs(teaching_vy) > teaching_vy_thr){                        
    //         if(fabs(teaching_vx) < teaching_vx_thr){
    //             teaching_vx = 0.0;    
    //             mode_dummy = 1;
    //         }
    //         else { //fabs(teaching_vx) > teaching_vx_thr, to prevent steped command
    //             if(teaching_vx >0) teaching_vx = teaching_vx - teaching_vx_thr;
    //             else               teaching_vx = teaching_vx + teaching_vx_thr;
    //             mode_dummy = 2;
    //         }       
    //         teaching_vx = 0.0;    
    //         mode_dummy = 1;     
    //     }
    //     else {
    //         teaching_vy = 0.0;
    //         if(fabs(teaching_vx) < teaching_vx_thr/5){ //*teaching_vz_thr is divided by 5 to easily operate straight mode*//
    //             teaching_vx = 0.0;    
    //             mode_dummy = 3;
    //         }
    //         else { //fabs(teaching_vx) > teaching_vx_thr, to prevent steped command
    //             if(teaching_vx >0) teaching_vx = teaching_vx - teaching_vx_thr/5;//*teaching_vz_thr is divided by 5 to easily operate straight mode*//
    //             else               teaching_vx = teaching_vx + teaching_vx_thr/5;//*teaching_vz_thr is divided by 5 to easily operate straight mode*//
    //             mode_dummy = 4;
    //         }                
    //     }
    //     teaching_w = 0.0;    
    // }
    // else{ //spinning, mode_=2
    //     teaching_vy = 0.0;    
    //     teaching_vx = 0.0;    
    //     mode_dummy = 5;
    // }
    //-------------------------------------------------------------------------------------------------------------------------------//

    std_msgs::Int16 mode_pub_data;
    mode_pub_data.data = mode_dummy;
    mode_pub_.publish(mode_pub_data);


    double thes_vel_x = 0.2;
    double thes_vel_w = 0.2;
    if (teaching_vx > thes_vel_x)
      teaching_vx = thes_vel_x;
    else if (teaching_vx < -thes_vel_x)
      teaching_vx = -thes_vel_x;

    if (teaching_vy > thes_vel_x)
      teaching_vy = thes_vel_x;
    else if (teaching_vy < -thes_vel_x)
      teaching_vy = -thes_vel_x;

    if (teaching_w > thes_vel_w)
      teaching_w = thes_vel_w;
    else if (teaching_w < -thes_vel_w)
      teaching_w = -thes_vel_w;  
    
    if (fabs(teaching_vx) < 0.01)
      teaching_vx = 0.0;
    if (fabs(teaching_vy) < 0.01)
      teaching_vy = 0.0;
    if (fabs(teaching_w) < 0.01)
      teaching_w = 0.0;    
    
    husky_ctrl_pub_msg_.linear.x = teaching_vx;
    husky_ctrl_pub_msg_.linear.y = teaching_vy;
    husky_ctrl_pub_msg_.angular.z = teaching_w;
    husky_ctrl_pub_.publish(husky_ctrl_pub_msg_);
}

MatrixXd skew_matrix(const VectorXd& vec){
        double v1 = vec(0);
        double v2 = vec(1);
        double v3 = vec(2);

        Eigen::Matrix3d CM;
        CM <<     0, -1*v3,    v2,
                 v3,     0, -1*v1,
              -1*v2,    v1,     0;

        return CM;
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
                is_published_whisper_ = false;
                cout << "end start calibration" << endl;   
                ft_calibration_ = ft_calibration_temp_;
                ft_calibration_temp_.setZero();  
                cout << "calibration data" << ft_calibration_.transpose() << endl; 

                std_msgs::Int16 feedback;
                feedback.data = 8; //"calibration finished"
                whisper_pub_.publish(feedback);                                
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

void example_set_custom_control_gain_admittance(IndyDCP3& indy) {
    Nrmk::IndyFramework::CustomGainSet custom_gain_set;    

    //  CustomJointController = HinfJointController
    //  gain0 = [100, 100, 100, 100, 100, 100] #kp
    //  gain1 = [20, 20, 20, 20, 20, 20] #kv
    //  gain2 = [800, 800, 600, 400, 400, 400] #ki  

    // gain0 = [100, 100, 100, 100, 100, 100]
    // gain1 = [20, 20, 20, 20, 20, 20]
    // gain2 = [600, 600, 400, 350, 350, 350]
    // gain3 = [100, 100, 100, 100, 100, 100] #kp
    // gain4 = [20, 20, 20, 20, 20, 20] #kv
    // gain5 = [350, 350, 200, 150, 150, 100] #ki
    // gain6 = [1, 0.06, 500, 80] #_mass_xyz, _mass_uvw, _stiffness_xyz, _stiffness_uvw

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

    custom_gain_set.add_gain2(800.0);
    custom_gain_set.add_gain2(800.0);
    custom_gain_set.add_gain2(600.0);
    custom_gain_set.add_gain2(400.0);
    custom_gain_set.add_gain2(400.0);
    custom_gain_set.add_gain2(400.0);

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

    custom_gain_set.add_gain5(350.0); // gain5 = [350, 350, 200, 150, 150, 100] #ki
    custom_gain_set.add_gain5(350.0);
    custom_gain_set.add_gain5(200.0);
    custom_gain_set.add_gain5(150.0);
    custom_gain_set.add_gain5(150.0);
    custom_gain_set.add_gain5(100.0);

    custom_gain_set.add_gain6(1.0); // gain6 = [1, 0.06, 500, 80] #_mass_xyz, _mass_uvw, _stiffness_xyz, _stiffness_uvw
    custom_gain_set.add_gain6(0.06);
    custom_gain_set.add_gain6(500.0);
    custom_gain_set.add_gain6(80.0);
    custom_gain_set.add_gain6(0.0);
    custom_gain_set.add_gain6(0.0);

    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);
    custom_gain_set.add_gain7(0.0);

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

    // custom_gain_set.add_gain7(ft_calibration_(0));
    // custom_gain_set.add_gain7(ft_calibration_(1));
    // custom_gain_set.add_gain7(ft_calibration_(2));
    // custom_gain_set.add_gain7(ft_calibration_(3));
    // custom_gain_set.add_gain7(ft_calibration_(4));
    // custom_gain_set.add_gain7(ft_calibration_(5));

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

void example_set_custom_control_gain_impedance(IndyDCP3& indy) {
    Nrmk::IndyFramework::CustomGainSet custom_gain_set;    

//  CustomJointController = HinfJointController
//  gain0 = [100, 100, 100, 100, 100, 100] #kp
//  gain1 = [20, 20, 20, 20, 20, 20] #kv
//  gain2 = [800, 800, 600, 400, 400, 400] #ki  

//  ImpedanceTaskController
//  gain3 = [150, 150, 80, 50, 50, 50] #kp
//  gain4 = [25, 25, 18, 15, 15, 15] #kv
//  gain5 = [0, 0, 0, 0, 0, 0] #ki  
//  gain6 = [5, 5, 5, 5, 5, 1] #K
//  gain7 = [80, 80, 40, 25, 25, 25] #KC
//  gain8 = [55, 55, 30, 15, 15, 15] #KD
//  gain9 = [4, 0, 0, 0, 0, 0] #rate

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

    custom_gain_set.add_gain2(800.0);
    custom_gain_set.add_gain2(800.0);
    custom_gain_set.add_gain2(600.0);
    custom_gain_set.add_gain2(400.0);
    custom_gain_set.add_gain2(400.0);
    custom_gain_set.add_gain2(400.0);

    custom_gain_set.add_gain3(500.0);   //u
    custom_gain_set.add_gain3(500.0);   //v
    custom_gain_set.add_gain3(500.0);   //w
    custom_gain_set.add_gain3(50.0);    //x
    custom_gain_set.add_gain3(50.0);    //y
    custom_gain_set.add_gain3(50.0);    //z        
    
    custom_gain_set.add_gain4(25.0);   //1
    custom_gain_set.add_gain4(25.0);   //2
    custom_gain_set.add_gain4(18.0);   //3
    custom_gain_set.add_gain4(15.0);   //4
    custom_gain_set.add_gain4(15.0);   //5
    custom_gain_set.add_gain4(15.0);   //6

    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);

    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(1.0);        

    custom_gain_set.add_gain7(80.0);
    custom_gain_set.add_gain7(80.0);
    custom_gain_set.add_gain7(40.0);
    custom_gain_set.add_gain7(25.0);
    custom_gain_set.add_gain7(25.0);
    custom_gain_set.add_gain7(25.0);        

    custom_gain_set.add_gain8(55.0);
    custom_gain_set.add_gain8(55.0);
    custom_gain_set.add_gain8(30.0);
    custom_gain_set.add_gain8(15.0);
    custom_gain_set.add_gain8(15.0);
    custom_gain_set.add_gain8(15.0);
    
    custom_gain_set.add_gain9(4.0);
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

void example_set_custom_control_gain_impedance_compliance(IndyDCP3& indy) {
    Nrmk::IndyFramework::CustomGainSet custom_gain_set;    

//  CustomJointController = HinfJointController
//  gain0 = [100, 100, 100, 100, 100, 100] #kp
//  gain1 = [20, 20, 20, 20, 20, 20] #kv
//  gain2 = [800, 800, 600, 400, 400, 400] #ki  

//  ImpedanceTaskController
//  gain3 = [150, 150, 80, 50, 50, 50] #kp
//  gain4 = [25, 25, 18, 15, 15, 15] #kv
//  gain5 = [0, 0, 0, 0, 0, 0] #ki  
//  gain6 = [5, 5, 5, 5, 5, 1] #K
//  gain7 = [80, 80, 40, 25, 25, 25] #KC
//  gain8 = [55, 55, 30, 15, 15, 15] #KD
//  gain9 = [4, 0, 0, 0, 0, 0] #rate

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

    custom_gain_set.add_gain2(800.0);
    custom_gain_set.add_gain2(800.0);
    custom_gain_set.add_gain2(600.0);
    custom_gain_set.add_gain2(400.0);
    custom_gain_set.add_gain2(400.0);
    custom_gain_set.add_gain2(400.0);

    custom_gain_set.add_gain3(500.0);   //u
    custom_gain_set.add_gain3(500.0);   //v
    custom_gain_set.add_gain3(500.0);   //w
    custom_gain_set.add_gain3(0.0);    //x
    custom_gain_set.add_gain3(0.0);    //y
    custom_gain_set.add_gain3(0.0);    //z        
    
    custom_gain_set.add_gain4(25.0);   //1
    custom_gain_set.add_gain4(25.0);   //2
    custom_gain_set.add_gain4(18.0);   //3
    custom_gain_set.add_gain4(15.0);   //4
    custom_gain_set.add_gain4(15.0);   //5
    custom_gain_set.add_gain4(15.0);   //6

    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);
    custom_gain_set.add_gain5(0.0);

    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(5.0);
    custom_gain_set.add_gain6(1.0);        

    custom_gain_set.add_gain7(80.0);
    custom_gain_set.add_gain7(80.0);
    custom_gain_set.add_gain7(40.0);
    custom_gain_set.add_gain7(25.0);
    custom_gain_set.add_gain7(25.0);
    custom_gain_set.add_gain7(25.0);        

    custom_gain_set.add_gain8(55.0);
    custom_gain_set.add_gain8(55.0);
    custom_gain_set.add_gain8(30.0);
    custom_gain_set.add_gain8(15.0);
    custom_gain_set.add_gain8(15.0);
    custom_gain_set.add_gain8(15.0);
    
    custom_gain_set.add_gain9(4.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);
    custom_gain_set.add_gain9(0.0);    
    custom_gain_set.add_gain9(2.0); //1.0:constant, 2.0:ry-axis (basic method), 3.0:z-axis
    custom_gain_set.add_gain9(1.0); //_constant_fz [N]


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