#pragma once

// System
#include <iostream>
#include <fstream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <ros/package.h>
#include <realtime_tools/realtime_publisher.h>
#include "visualization_msgs/Marker.h"
#include "dynamic_reconfigure/server.h"

// tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


//robotiq gripper
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

// neuromeka
#include "../neuromeka_cpp/indydcp3.h"
#include "../include/kimm_indydcp3/kimm_example.h"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

// ROS
ros::Publisher joint_states_pub_, ft_calibrated_data_pub_, ft_transformed_data_pub_, ft_link0_data_pub_, robotiq_ft_link0_data_pub_;
// realtime_tools::RealtimePublisher<geometry_msgs::Twist> husky_ctrl_pub_;
ros::Publisher husky_ctrl_pub_, robotiq_ft_reset_pub_;
ros::Subscriber robotiq_state_subs_, teleop_joy_subs_, robotq_ft_sensor_subs_;
sensor_msgs::JointState joint_states_msg_;        
geometry_msgs::Wrench ft_calibrated_data_msg_, ft_transformed_data_msg_, ft_link0_data_for_mobile_msg_, robotiq_ft_link0_data_for_mobile_msg_;  
geometry_msgs::Twist husky_ctrl_pub_msg_;

// thread
std::mutex calculation_mutex_;
std::thread async_calculation_thread_, mode_change_thread_;
bool is_calculated_{false}, quit_all_proc_{false};

double time_, dt_;
bool isgrasp_ = false;      
bool load_gripper_, can_i_do_mobile_teaching_, mobile_teaching_trigger_;
sensor_msgs::JointState robotiq_state_msg_;
robotiq_2f_gripper_control::RobotiqActionClient * gripper_robotiq_;        
Eigen::VectorXi isbutton_pushed_;
ofstream fout_;  
Vector6d indy_q_, indy_dq_;
bool is_log_, is_cal_;
Vector6d ft_data_, ft_calibration_, ft_calibration_temp_, ft_calibrated_data_, ft_transformed_data_, ft_link0_data_, ft_link0_filtered_data_for_mobile_, robotiq_ft_link0_data_, robotiq_ft_link0_filtered_data_for_mobile_;
int calibration_cnt_;
Eigen::Affine3d T_base_ft_;  
Eigen::Matrix3d R_tcp_ft_, R_link0_tcp_, R_link0_ft_;
double compensated_value_;

int msg_=0;;
void modeChangeReaderProc();
void keyboard_event(IndyDCP3& indy);
void call_IndyData(IndyDCP3& indy);
double deg2rad(const double degrees);
void robotiqstateCallback(const sensor_msgs::JointStateConstPtr &msg);
void teleopjoyCallback(const sensor_msgs::JoyConstPtr &joy_msg);    
void robotiqftsensorCallback(const geometry_msgs::WrenchStamped &msg);
void data_log_start(IndyDCP3& indy);    
void data_log_end(IndyDCP3& indy);    
void ft_sensor_calibration(IndyDCP3& indy);
void tf_print(IndyDCP3& indy);
void R_tcp_ft_sensor();
void mobile_teaching();

Vector6d lowpassFilter(double sample_time, const Vector6d x, const Vector6d y_last, double cutoff_frequency){ //cutoff_frequency is in [Hz]
      double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
      return gain * x + (1 - gain) * y_last;
}

// API
void example_get_robot_data(IndyDCP3& indy);
void example_get_robot_control_data(IndyDCP3& indy);
void example_get_digital_inputs(IndyDCP3& indy);
void example_get_digital_outputs(IndyDCP3& indy);
void example_set_digital_outputs(IndyDCP3& indy);
void example_get_analog_inputs(IndyDCP3& indy);
void example_get_analog_outputs(IndyDCP3& indy);
void example_set_analog_outputs(IndyDCP3& indy);
void example_get_endtool_digital_inputs(IndyDCP3& indy);
void example_get_endtool_digital_outputs(IndyDCP3& indy);
void example_set_endtool_digital_outputs(IndyDCP3& indy);
void example_get_endtool_analog_inputs(IndyDCP3& indy);
void example_get_endtool_analog_outputs(IndyDCP3& indy);
void example_set_endtool_analog_outputs(IndyDCP3& indy);
void example_get_device_info(IndyDCP3& indy);
void example_get_ft_sensor_data(IndyDCP3& indy);
void example_stop_robot_motion(IndyDCP3& indy);
void example_get_home_position(IndyDCP3& indy);
void example_joint_move(IndyDCP3& indy, const std::vector<float>& j_pos, int base_type);
void example_task_move(IndyDCP3& indy, const std::array<float, 6>& t_pos, int base_type);
void example_move_along_circular_path(IndyDCP3& indy, const std::array<float, 6>& t_pos1, const std::array<float, 6>& t_pos2, float angle);
void example_move_robot_to_home(IndyDCP3& indy);
void example_start_teleoperation(IndyDCP3& indy, TeleMethod method);
void example_move_joints_in_teleoperation(IndyDCP3& indy, const std::vector<float>& jpos);
void example_stop_teleoperation(IndyDCP3& indy);
void example_inverse_kinematics(IndyDCP3& indy, const std::array<float, 6>& tpos, const std::vector<float>& init_jpos);
void example_inverse_kinematics(IndyDCP3& indy);
void example_enable_direct_teaching(IndyDCP3& indy, bool enable);
void example_set_simulation_mode(IndyDCP3& indy, bool enable);
void example_recover_robot(IndyDCP3& indy);
void example_enable_manual_recovery(IndyDCP3& indy, bool enable);
void example_calculate_and_print_relative_pose(IndyDCP3& indy, 
                                        const std::array<float, 6>& current_pos, 
                                        const std::array<float, 6>& relative_pos);
void example_play_program(IndyDCP3& indy, const std::string& program_name, int program_index);
void example_pause_program(IndyDCP3& indy);
void example_resume_program(IndyDCP3& indy);
void example_stop_program(IndyDCP3& indy);
void example_set_speed_ratio(IndyDCP3& indy, unsigned int speed_ratio);
void example_set_bool_var(IndyDCP3& indy);
void example_set_int_var(IndyDCP3& indy);
void example_set_float_var(IndyDCP3& indy);
void example_set_jpos_var(IndyDCP3& indy);
void example_set_tpos_var(IndyDCP3& indy);
void example_get_bool_variables(IndyDCP3& indy);
void example_get_int_variables(IndyDCP3& indy);
void example_get_float_variables(IndyDCP3& indy);
void example_get_jpos_variables(IndyDCP3& indy);
void example_get_tpos_variables(IndyDCP3& indy);
void example_set_home_position(IndyDCP3& indy);
void example_get_reference_frame(IndyDCP3& indy);
void example_set_reference_frame(IndyDCP3& indy, const std::array<float, 6>& ref_frame);
void example_set_planar_reference_frame(IndyDCP3& indy, std::array<float, 6>& fpos_out, 
                                const std::array<float, 6>& fpos0, const std::array<float, 6>& fpos1, const std::array<float, 6>& fpos2);
void example_set_tool_frame(IndyDCP3& indy, const std::array<float, 6>& tool_frame);
void example_set_friction_compensation(IndyDCP3& indy);
void example_get_friction_compensation(IndyDCP3& indy);
void example_set_tool_properties(IndyDCP3& indy);
void example_get_tool_properties(IndyDCP3& indy);
void example_set_collision_sensitivity_level(IndyDCP3& indy, unsigned int level);
void example_get_collision_sensitivity_level(IndyDCP3& indy);
void example_get_collision_sensitivity_parameters(IndyDCP3& indy);
void example_set_collision_sensitivity_parameters(IndyDCP3& indy);
void example_get_collision_policy(IndyDCP3& indy);
void example_set_collision_policy(IndyDCP3& indy);
void example_get_safety_limits(IndyDCP3& indy);
void example_activate_sdk(IndyDCP3& indy);
void example_get_custom_control_mode(IndyDCP3& indy);
void example_set_custom_control_mode(IndyDCP3& indy, int mode);
void example_get_custom_control_gain(IndyDCP3& indy);
void example_set_custom_control_gain_low(IndyDCP3& indy);
void example_set_custom_control_gain_high(IndyDCP3& indy);
void example_set_custom_control_gain_high_impedance(IndyDCP3& indy);
void example_start_log(IndyDCP3& indy); 
void example_end_log(IndyDCP3& indy);
void example_wait_cmd(IndyDCP3& indy, int exam);
void example_wait_for_operation_state(IndyDCP3& indy);
void example_wait_for_motion_state(IndyDCP3& indy);
void example_move_joint_waypoint(IndyDCP3& indy);
void example_move_task_waypoint(IndyDCP3& indy);
void example_set_mount_pos(IndyDCP3& indy);
void example_get_mount_pos(IndyDCP3& indy);
void example_get_violation_message_queue(IndyDCP3& indy);
void example_get_stop_state(IndyDCP3& indy);
void example_set_endtool_rs485_rx(IndyDCP3& indy);
void example_get_endtool_rs485_rx(IndyDCP3& indy);
void example_get_endtool_rs485_tx(IndyDCP3& indy);
void example_set_end_led_dim(IndyDCP3& indy);
void example_get_conveyor(IndyDCP3& indy);
void example_set_conveyor_by_name(IndyDCP3& indy);
void example_get_conveyor_state(IndyDCP3& indy);
void example_set_sander_command(IndyDCP3& indy);
void example_get_sander_command(IndyDCP3& indy);
void example_get_load_factors(IndyDCP3& indy);
void example_set_auto_mode(IndyDCP3& indy, bool on);
void example_check_auto_mode(IndyDCP3& indy);
void example_check_reduced_mode(IndyDCP3& indy);
void example_get_safety_function_state(IndyDCP3& indy);
void example_request_safety_function(IndyDCP3& indy);
void example_get_safety_control_data(IndyDCP3& indy);
void example_get_gripper_data(IndyDCP3& indy);
void example_set_gripper_command(IndyDCP3& indy);
void example_activate_cri(IndyDCP3& indy);
void example_is_cri_active(IndyDCP3& indy);
void example_login_cri_server(IndyDCP3& indy);
void example_is_cri_login(IndyDCP3& indy);
void example_set_cri_target(IndyDCP3& indy);
void example_set_cri_option(IndyDCP3& indy);
void example_get_cri_proj_list(IndyDCP3& indy);
void example_get_cri(IndyDCP3& indy);
void example_basic_movelf(IndyDCP3& indy);
void example_get_transformed_ft_sensor_data(IndyDCP3& indy);
void example_move_joint_traj(IndyDCP3& indy);
void example_move_task_traj(IndyDCP3& indy);
void example_move_conveyor(IndyDCP3& indy);
void example_move_axis(IndyDCP3& indy);
void example_forward_kin(IndyDCP3& indy);
void example_set_tact_time(IndyDCP3& indy);
void example_get_tact_time(IndyDCP3& indy);
void example_set_ft_sensor_config(IndyDCP3& indy);
void example_get_ft_sensor_config(IndyDCP3& indy);
void example_set_do_config_list(IndyDCP3& indy);
void example_get_do_config_list(IndyDCP3& indy);
void example_move_recover_joint(IndyDCP3& indy);
void example_get_control_info(IndyDCP3& indy);
void example_check_aproach_retract_valid(IndyDCP3& indy);
void example_get_pallet_point_list(IndyDCP3& indy);
void example_play_tuning_program(IndyDCP3& indy);
void example_set_di_config_list(IndyDCP3& indy);
void example_get_di_config_list(IndyDCP3& indy);
void example_set_auto_servo_off(IndyDCP3& indy);
void example_get_auto_servo_off(IndyDCP3& indy);
void example_set_safety_stop_config(IndyDCP3& indy);
void example_get_safety_stop_config(IndyDCP3& indy);
void example_get_reduced_ratio(IndyDCP3& indy);
void example_get_reduced_speed(IndyDCP3& indy);
void example_set_reduced_speed(IndyDCP3& indy);
void example_set_teleop_params(IndyDCP3& indy);
void example_get_teleop_params(IndyDCP3& indy);
void example_get_kinematics_params(IndyDCP3& indy);
void example_get_io_data(IndyDCP3& indy);



