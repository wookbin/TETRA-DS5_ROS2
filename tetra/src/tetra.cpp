#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// //Custom Service//
#include "interfaces/srv/parameter_read.hpp"
#include "interfaces/srv/parameter_write.hpp"
#include "interfaces/srv/set_move_mode.hpp"
#include "interfaces/srv/linear_position_move.hpp"
#include "interfaces/srv/angular_position_move.hpp"

extern "C"
{
	#include "drive_module.h"
	#include "dssp_rs232_drive_module.h"
}

#include <chrono>
#include <thread>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <atomic>
#include <mutex>
#include <condition_variable>

#define WHEEL_RADIUS 0.1033 //m
#define WHEEL_DISTANCE 0.4330 //m
#define TREAD_WIDTH 0.04 //m

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;
pthread_t p_odom_loop_thread;

//serial
int com_port = 0;
char port[16] = {0,};
//calc
bool first;
double prev_coordinates[3] = {0.0, };
double coordinates[3] = {0.0, }; //x,y,theta
double dt = 0.0;
double velocity[3] = {0.0, };
//cmd_vel
double input_linear=0.0, input_angular = 0.0;
double control_linear=0.0, control_angular = 0.0;
double linear=0.0, angular = 0.0, bt_linear = 0.0, bt_angular = 0.0;
int m_old_accel_data = 50;
//Pose
double m_dX_pos = 0.0;
double m_dY_pos = 0.0;
double m_dTheta = 0.0;
int Reset = 0;
//bumper & emg
int m_bumper_data = 0;
int m_emg_state = 0;
//Error Code
int m_left_error_code = 0;
int m_right_error_code = 0;
//Position move
int m_iPOS_Y = 0;
int m_iPOS_Theta = 0;
bool bPosition_mode_flag = false;
//emg one time check flag
bool m_bCheck_emg = true;
//Joystick value
int joy_linear = 1.0;
int joy_angular = 1.0;
//Parameter data
int m_iParam = 0;
//ekf_localization
bool m_bEKF_option = false;
bool m_bForwardCheck = false;
//Joystick Enable & Disable
bool m_bFlag_joy_enable = false;

// Flags to control callback execution
bool m_bpending_ = true;
// Mutex to protect flag updates
std::mutex mutex_;

std::atomic<bool> stop_requested(false);

void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}

class TETRA: public rclcpp::Node
{
public:
	TETRA() : Node("tetra")
	{
		//tf2_ros
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		//publish list/////////////////////////////////////////////////////////////////////////////////////
		odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 30);
		bumper_publisher = this->create_publisher<std_msgs::msg::Int32>("bumper_data", 10);
		emg_publisher = this->create_publisher<std_msgs::msg::Int32>("emg_state", 1);
		left_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("left_error_code", 1);
		right_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("right_error_code", 1);

		//subscribe list////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&TETRA::joyCallback, this, _1));
		cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", rclcpp::SensorDataQoS(), std::bind(&TETRA::velCallback, this, _1));
		cmd_vel_subscriber2 = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav2", rclcpp::SensorDataQoS(), std::bind(&TETRA::velCallback2, this, _1));
		cmd_vel_manual_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_manual", rclcpp::SensorDataQoS(), std::bind(&TETRA::Manual_velCallback, this, _1));
		acc_subscriber = this->create_subscription<std_msgs::msg::Int32>("accel_vel", 10, std::bind(&TETRA::accelCallback, this, _1));
		pose_reset_subscriber = this->create_subscription<std_msgs::msg::Int32>("pose_reset", 10, std::bind(&TETRA::pose_resetCallback, this, _1));
		servo_on_subscriber = this->create_subscription<std_msgs::msg::Int32>("servo_on", 10, std::bind(&TETRA::servo_onCallback, this, _1));
		power_status_subscriber = this->create_subscription<std_msgs::msg::Int32>("power_status", 10, std::bind(&TETRA::power_statusCallback, this, _1));

		//service list///////////////////////////////////////////////////////////////////////////////////////
		parameter_read_srv = create_service<interfaces::srv::ParameterRead>(
        	"param_read_cmd", 
		std::bind(&TETRA::Parameter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		parameter_write_srv = create_service<interfaces::srv::ParameterWrite>(
        	"param_write_cmd", 
		std::bind(&TETRA::Parameter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));

		set_move_mode_srv = create_service<interfaces::srv::SetMoveMode>(
        	"mode_change_cmd", 
		std::bind(&TETRA::Movemode_Change_Command, this, std::placeholders::_1, std::placeholders::_2));

		linear_position_move_srv = create_service<interfaces::srv::LinearPositionMove>(
        	"linear_move_cmd", 
		std::bind(&TETRA::Linear_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

		angular_position_move_srv = create_service<interfaces::srv::AngularPositionMove>(
        	"angular_move_cmd", 
		std::bind(&TETRA::Angular_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

		
		//PARAM
		this->declare_parameter("m_bEKF_option", rclcpp::PARAMETER_BOOL);
		m_bEKF_option_param = this->get_parameter("m_bEKF_option");
		//Get Param
		m_bEKF_option = m_bEKF_option_param.as_bool();

	}

	////value//////////////////////////////////////////////////////////////////////////////
	rclcpp::Time current_time, last_time;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Parameter m_bEKF_option_param;

	//Publisher
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bumper_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr emg_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_error_code_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_error_code_publisher;

	//Subscription
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber2;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_manual_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr acc_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pose_reset_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_on_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr power_status_subscriber;


	////Serial function//////////////////////////////////////////////////////////////////////////////
	double distance_to_wheelrotation(double wheel_diameter, double distance)
	{
		return (distance / (M_PI * wheel_diameter));
	}

	double direction_to_diwheelrotation_diff(double wheel_distance, double wheel_diameter, double rad)
	{
		return (wheel_distance / (wheel_diameter * M_PI)) * rad;
	}

	void speed_to_diwheel_rpm(double meter_per_sec, double rad_per_sec, double * left_rpm, double * right_rpm)
	{
		double linearRotate = distance_to_wheelrotation(2.0*WHEEL_RADIUS, meter_per_sec);
		double angularRorate = direction_to_diwheelrotation_diff(WHEEL_DISTANCE/2.0, 2.0*WHEEL_RADIUS, rad_per_sec);

		*left_rpm = (linearRotate - angularRorate) * 60.0;
		*right_rpm = (linearRotate + angularRorate) * 60.0;

	}

	double RPM_to_ms(double d_rpm)
	{
		return WHEEL_RADIUS*2.0*M_PI*(d_rpm/60.0);
	}

	void SetMoveCommand(double fLinear_vel, double fAngular_vel)
	{
		double Left_Wheel_vel = 0.0;
		double Right_Wheel_vel = 0.0;
		speed_to_diwheel_rpm(fLinear_vel, fAngular_vel, &Left_Wheel_vel, &Right_Wheel_vel);

		int iData1 = 1000.0 * RPM_to_ms(Left_Wheel_vel);
		int iData2 = 1000.0 * RPM_to_ms(Right_Wheel_vel);
		dssp_rs232_drv_module_set_velocity(iData1, iData2);
	}
	
	//Callback function////////////////////////////////////////////////////////////////////////////
	void velCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if(m_bpending_) return;
        //RCLCPP_INFO(this->get_logger(), "Executing velCallback ");
        linear = vel->linear.x;
        angular = vel->angular.z;

	}
	void velCallback2(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		m_bpending_ = true;
        	//RCLCPP_INFO(this->get_logger(), "Executing velCallback2 !!!!");
        	linear = vel->linear.x;
        	angular = vel->angular.z;

		if(linear == 0.0 && angular == 0.0)
		{
			m_bpending_ = false;
		}

	}
	void Manual_velCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		linear = vel->linear.x;
		angular = vel->angular.z;

	}
	
	void accelCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		int m_iAccel_data = msg->data;
		//Acceleration command
		if(m_old_accel_data != m_iAccel_data)
		{
			dssp_rs232_drv_module_set_parameter(6, m_iAccel_data);
			//update
			m_old_accel_data = m_iAccel_data;
		}

	}

	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
	{
		//Axis stick
		if(joy->buttons[8])
		{
			m_bFlag_joy_enable = false; //back 
		}
		if(joy->buttons[9])
		{
			m_bFlag_joy_enable = true; //start
		}

		if(m_bFlag_joy_enable)
		{
			////Two Hand Joystick
			if(joy->axes[4])
			{
				bt_angular += joy_angular * joy->axes[4];
			}
			if(joy->axes[5])
			{
				bt_linear += joy_linear * joy->axes[5];
			}

			//Buttons
			if(joy->buttons[0])
			{
				joy_angular -= 1.0;
				if(joy_angular < 0) joy_angular = 0;
			}
			if(joy->buttons[1])
			{
				joy_linear -= 1.0;
				if(joy_linear < 0) joy_linear = 0;
			}
			if(joy->buttons[2])
			{
				joy_angular += 1.0;
			}
			if(joy->buttons[3])
			{
				joy_linear += 1.0;
			}
			if(joy->buttons[5])
			{
					bt_linear = 0;
					bt_angular = 0;
					joy_linear = 1.0;
					joy_angular = 1.0;
			}
			
			//Velocity Command//
			linear = (joy_linear * (double)joy->axes[1] + bt_linear) / 3.0;
			////Two Hand Joystick
			angular = ((double)joy_angular * (joy->axes[1] >= 0 ? joy->axes[2] : (joy->axes[2] * -1) ) + bt_angular) / 3.0;

		}
		
	}

	void pose_resetCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		Reset = msg->data;
		if(Reset == 1)
		{
			dssp_rs232_drv_module_reset_odometry();
			Reset = 0;
		}
	}

	void servo_onCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		int m_idata = msg->data;
		if(m_idata == 1)
		{
			bPosition_mode_flag = false;
			//Velocity Mode
			dssp_rs232_drv_module_set_velocitymode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			m_idata = 0;
		}
		else if(m_idata == 2)
		{
			dssp_rs232_drv_module_set_servo(0);
			m_idata = 0;
		}

		else if(m_idata == 3)
		{
			bPosition_mode_flag = true;
			//Velocity Mode
			dssp_rs232_drv_module_set_positionmode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			m_idata = 0;
		}
	}

	void power_statusCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		int m_idata = msg->data;
		if(m_idata == -10)
		{
			dssp_rs232_drv_module_set_servo(0); //Servo Off
			printf("[Error]: Power Board Error !!! \n");
		}

	}


private:
	////value/////////////////////////////////////////////////////////////////////////////////
	rclcpp::Service<interfaces::srv::ParameterRead>::SharedPtr parameter_read_srv;
	rclcpp::Service<interfaces::srv::ParameterWrite>::SharedPtr parameter_write_srv;
	rclcpp::Service<interfaces::srv::SetMoveMode>::SharedPtr set_move_mode_srv;
	rclcpp::Service<interfaces::srv::LinearPositionMove>::SharedPtr linear_position_move_srv;
	rclcpp::Service<interfaces::srv::AngularPositionMove>::SharedPtr angular_position_move_srv;

	//service function command////////////////////////////////////////////////////////////////
	bool Parameter_Read_Command(
		const std::shared_ptr<interfaces::srv::ParameterRead::Request> request, 
		const std::shared_ptr<interfaces::srv::ParameterRead::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_read_parameter(request->num, &m_iParam);
		response->data = m_iParam;
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Parameter_Write_Command(
		const std::shared_ptr<interfaces::srv::ParameterWrite::Request> request, 
		const std::shared_ptr<interfaces::srv::ParameterWrite::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_set_parameter(request->num, request->data);
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Movemode_Change_Command(
		const std::shared_ptr<interfaces::srv::SetMoveMode::Request> request, 
		const std::shared_ptr<interfaces::srv::SetMoveMode::Response> response)
	{
		bool bResult = false;
		if(request->mode == 1) //Position Mode
		{
			dssp_rs232_drv_module_set_servo(0);
			usleep(10000);
			dssp_rs232_drv_module_set_positionmode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			usleep(10000);
		}
		else //Velocity Mode
		{
			dssp_rs232_drv_module_set_servo(0);
			usleep(10000);
			dssp_rs232_drv_module_set_velocitymode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			usleep(10000);
		}
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Linear_Move_Command(
		const std::shared_ptr<interfaces::srv::LinearPositionMove::Request> request, 
		const std::shared_ptr<interfaces::srv::LinearPositionMove::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_set_position(0, 0, request->linear_position);
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Angular_Move_Command(
		const std::shared_ptr<interfaces::srv::AngularPositionMove::Request> request, 
		const std::shared_ptr<interfaces::srv::AngularPositionMove::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_set_position(2, 0, request->angular_degree);
		bResult = true;
		response->command_result = bResult;
		return true;
	}

};


//Main Loop//
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<TETRA>();

	//init
	node->current_time = node->now();
    	node->last_time = node->now();
	first = true;
	for(int i=0;i<3;i++)
	{
		prev_coordinates[i] = 0;
		coordinates[i] = 0;
	}

	rclcpp::WallRate loop_rate(60); //default: 30HZ
	sprintf(port, "/dev/ttyS0");
	//sprintf(port, "/dev/TETRA");
	//RS232 Connect
	if(dssp_rs232_drv_module_create(port, 200) == 0)
	{
		printf("TETRA_DS_rs232 Port Open Success\n");
	}
	else
	{
		printf("TETRA_DS_rs232 Port Open Error!\n");
		return -1;
	}

	// Error Reset//
	dssp_rs232_drv_module_set_drive_err_reset();
	usleep(10000);
	//set acc slope time
	dssp_rs232_drv_module_set_parameter(6, 50); //accelation slop time 50mecs
	usleep(10000);
	//Velocity Mode
	dssp_rs232_drv_module_set_velocitymode();
	usleep(10000);
	//Servo On//
	dssp_rs232_drv_module_set_servo(1);
	usleep(10000);
	//Reset odometry
	dssp_rs232_drv_module_reset_odometry();
	usleep(10000);
	//emg flag//
	bool m_bflag_emg = false;
	//
	printf("□■■■■■■■□■■■■■■□□■■■■■■■□■■■■■■□□□□□□■□□□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□□□■□□□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□□■□■□□□\n");
	printf("□□□□■□□□□■■■■■■□□□□□■□□□□■■■■■■□□□□■□□□■□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□■■■■■□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□■□□□□□■□\n");
	printf("□□□□■□□□□■■■■■■□□□□□■□□□□■□□□□□■□□■□□□□□■□\n");                 

	while (rclcpp::ok() && !stop_requested)
	{
		rclcpp::spin_some(node);
		
		input_linear  = linear;
		input_angular = angular;

		//smoother velocity Loop//////////////////////////////////////////////////
		//linear_velocity
		if(linear > 0)
			m_bForwardCheck = true;
		else
			m_bForwardCheck = false;

		
		if(m_bForwardCheck)
		{
		
			if(input_linear > control_linear)
			{
				control_linear = min(input_linear, control_linear + 0.01);  //10mm++
	
			}
			else if(input_linear < control_linear)
			{
				control_linear = max(input_linear, control_linear - 0.05);  //50mm --

			}	
			else
			{
				control_linear = input_linear;
			}
		}
		else
		{
			if(input_linear < control_linear)
			{
				control_linear = max(input_linear, control_linear - 0.01);
				if(control_linear > 0)
				{
					control_linear = max(input_linear, control_linear - 0.05);  //50mm --
				}
		
			}
			else if(input_linear > control_linear)
			{
				control_linear = min(input_linear, control_linear + 0.05);

			}
			else
			{
				control_linear = input_linear;
			}
		}
		//////////////////////////////////////////////////////////////////////////
		control_angular = input_angular;

		//EMG Check Loop
		std_msgs::msg::Int32 emg_state;
		emg_state.data = m_emg_state;
		node->emg_publisher->publish(emg_state);

		//Bumper Check Loop
		std_msgs::msg::Int32 bumper_data;
		bumper_data.data = m_bumper_data;
		node->bumper_publisher->publish(bumper_data);
		
		//Error Code Check Loop
		std_msgs::msg::Int32 left_error_code;
		std_msgs::msg::Int32 right_error_code;
		left_error_code.data = m_left_error_code;
		right_error_code.data = m_right_error_code;
		node->left_error_code_publisher->publish(left_error_code);
		node->right_error_code_publisher->publish(right_error_code);

		//odometry calback//
		dssp_rs232_drv_module_read_odometry(&m_dX_pos, &m_dY_pos, &m_dTheta);
		coordinates[0] = (m_dX_pos /1000.0);
		coordinates[1] = (m_dY_pos /1000.0);
		coordinates[2] = m_dTheta * (M_PI/1800.0);

		if(!bPosition_mode_flag) //Velocity mode only
		{
			node->SetMoveCommand(control_linear, control_angular);
			//printf("[cmd_vel]: control_linear = %.3f, control_angular = %.3f \n", control_linear, control_angular);
			dssp_rs232_drv_module_read_bumper_emg(&m_bumper_data, &m_emg_state, &m_left_error_code, &m_right_error_code);
		}

		//Error Code Check -> Reset & servo On Loop
		if(m_left_error_code != 48 || m_right_error_code != 48)
		{
			printf("[Motor Driver Error] Left Error Code: %d \n", m_left_error_code);
			printf("[Motor Driver Error] Right Error Code: %d \n", m_right_error_code);
			dssp_rs232_drv_module_set_drive_err_reset();
			usleep(1000);
			dssp_rs232_drv_module_set_servo(1); //Servo On
		}

		if(m_emg_state)
		{
			if(m_bflag_emg)
			{
				dssp_rs232_drv_module_set_servo(0); //Servo Off
				usleep(1000);
				dssp_rs232_drv_module_set_drive_err_reset();
				usleep(1000);
				m_bflag_emg = false;
			}
		}
		else
		{
			if(!m_bflag_emg)
			{
				dssp_rs232_drv_module_set_servo(1); //Servo On
				m_bflag_emg = true;
			}
		}

		//node->read();
		if(first) 
		{
			node->current_time = node->now();
			node->last_time = node->current_time;
			first=false;
		}
		else 
		{
			node->current_time = node->now();

			dt=(node->current_time - node->last_time).seconds(); //dt=(current_time-last_time).toSec();
			for(int i=0;i<3;i++)
			{
				velocity[i]=(coordinates[i]-prev_coordinates[i])/dt;
				prev_coordinates[i]=coordinates[i];
			}

			geometry_msgs::msg::Quaternion odom_quat;
			geometry_msgs::msg::TransformStamped odom_trans;
			tf2::Quaternion q;
			q.setRPY(0.0,0.0,coordinates[2]);

			if(!m_bEKF_option) //add...ekf_localization option _ wbjin
			{
				odom_trans.header.stamp = node->current_time;
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_footprint";
				odom_trans.transform.translation.x = coordinates[0];
				odom_trans.transform.translation.y = coordinates[1];
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation.x = q.x();
				odom_trans.transform.rotation.y = q.y();
				odom_trans.transform.rotation.z = q.z();
				odom_trans.transform.rotation.w = q.w();
				node->tf_broadcaster_->sendTransform(odom_trans);

			}
			
			//nav_msgs::Odometry odom;
			nav_msgs::msg::Odometry odom;
			odom.header.stamp = node->current_time;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";
			//pose
			odom.pose.pose.position.x = coordinates[0];
			odom.pose.pose.position.y = coordinates[1];
			odom.pose.pose.position.z = 0.0;
			//odom.pose.pose.orientation = odom_quat;
			odom.pose.pose.orientation.x = q.x();
			odom.pose.pose.orientation.y = q.y();
			odom.pose.pose.orientation.z = q.z();
			odom.pose.pose.orientation.w = q.w();
			odom.twist.twist.linear.x = velocity[0];
			odom.twist.twist.linear.y = velocity[1];
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.z = velocity[2];
			node->odom_publisher->publish(odom);

			node->last_time = node->current_time;
		}

		loop_rate.sleep();
    }

	
	//Servo Off
	dssp_rs232_drv_module_set_servo(0);
	printf("Servo Off ! \n");
	usleep(10000);
	//RS232 Disconnect
	dssp_rs232_drv_module_destroy();
	printf("RS232 Disconnect ! \n");

	rclcpp::shutdown();
    return 0;
}
