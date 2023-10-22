#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
//#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
//AR_tag Marker message
#include "ar_track_alvar_msgs/msg/alvar_markers.hpp" //AlvarMarkers

// //Custom Service//
#include "interfaces/srv/led_control.hpp"
#include "interfaces/srv/led_toggle_control.hpp"
#include "interfaces/srv/toggle_on.hpp"
#include "interfaces/srv/docking_control.hpp" //docking
#include "interfaces/srv/docking_stop.hpp" //docking
#include "interfaces/srv/imu_reset.hpp" //imu reset
#include "interfaces/srv/save_map.hpp" //Map save service


#include <chrono>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

#define HIGH_BATTERY 95
#define LOW_BATTERY 15
#define MAX_RETRY_CNT 10
#define BUF_LEN 4096

FILE *fp;
int status;
char Textbuffer[BUF_LEN];

int  m_iDocking_CommandMode = 0;
int  m_iNoMarker_cnt = 0;
int  m_iRotation_Mode = 0; //Docking Rotation Mode Select
int  m_iTrun_cnt = 0;
int  m_iBack_cnt = 0;
//Tracked Pose
float m_fTracked_pose_x = 0.0;
float m_fTracked_pose_y = 0.0;
float m_fTracked_pose_z = 0.0;
float m_fTracked_orientation_x = 0.0;
float m_fTracked_orientation_y = 0.0;
float m_fTracked_orientation_z = 0.0;
float m_fTracked_orientation_w = 1.0;

typedef struct HOME_POSE
{
    string HOME_strLOCATION = "HOME";
    double HOME_dPOSITION_X = -0.6;
    double HOME_dPOSITION_Y = 0.0;
    double HOME_dPOSITION_Z = 0.0;
    double HOME_dQUATERNION_X = 0.0;
    double HOME_dQUATERNION_Y = 0.0;
    double HOME_dQUATERNION_Z = 0.0;
    double HOME_dQUATERNION_W = 1.0;

}HOME_POSE;
HOME_POSE _pHomePose;

//AR_TAG Pose
typedef struct AR_TAG_POSE
{
    int m_iSelect_AR_tag_id = 0;
    int m_iAR_tag_id_Index = 0;
    int m_iAR_tag_id = -1;
    float m_fAR_tag_pose_x = 0.0;
    float m_fAR_tag_pose_y = 0.0;
    float m_fAR_tag_pose_z = 0.0;
    float m_fAR_tag_orientation_x = 0.0;
    float m_fAR_tag_orientation_y = 0.0;
    float m_fAR_tag_orientation_z = 0.0;
    float m_fAR_tag_orientation_w = 0.0;
    double m_fAR_tag_roll = 0.0;
    double m_fAR_tag_pitch = 0.0;
    double m_fAR_tag_yaw = 0.0;
    //Transform AR Tag Axis -> Robot Axis
    float m_transform_pose_x = 0.0;
    float m_transform_pose_y = 0.0;
    float m_fPositioning_Angle = 0.0;
    //Calc Odom to AR_Marker TF
    double m_target_yaw = 0.0;

}AR_TAG_POSE;
AR_TAG_POSE _pAR_tag_pose;

typedef struct LANDMARK_POSE
{
    string header_frame_id;
    string ns;
    int mark_id;
    double pose_position_x;
    double pose_position_y;
    double pose_position_z;
    double pose_orientation_x;
    double pose_orientation_y;
    double pose_orientation_z;
    double pose_orientation_w;

}LANDMARK_POSE;

typedef struct LAND_MARK_POSE
{
    float init_position_x = 0.0;
    float init_position_y = 0.0;
    float init_position_z = 0.0;
    float init_orientation_z = 0.0;
    float init_orientation_w = 1.0;

}LAND_MARK_POSE;
LAND_MARK_POSE _pLandMarkPose;

typedef struct ROBOT
{
	int  HOME_id = 0;
	int  m_iDocking_id = 0;
	bool m_bFalg_DockingExit = false;
	int  m_iCallback_Battery = 0;
    int  m_iCallback_ErrorCode = 0;
    int  m_iCallback_EMG = 0;
    int  m_iCallback_Bumper = 0;
    int  m_iCallback_Charging_status = 0;
    //Bumper Collision Behavior//
    int  m_iBumperCollisionBehavor_cnt = 0;
    //auto test
    int  m_iMovebase_Result = 0;
    //Conveyor Info..(Option)
    int  m_iConveyor_Sensor_info = 0;
    int  m_iConveyor_id = 0;
    int  m_iConveyor_movement = 0; // 0: nomal , 1: Loading , 2: Unloading
    
}ROBOT;
ROBOT _pRobot;

typedef struct FALG_VALUE
{
    bool m_bflag_NextStep = false;
    bool m_bflag_ComebackHome = false;
    bool m_bfalg_DockingExit = false;
    bool m_bflag_Conveyor_docking = false;
    bool m_bFlag_Disable_bumper = false;
    //Tracking obstacle Check//
    bool m_bFlag_Obstacle_Right = false;
    bool m_bFlag_Obstacle_Center = false;
    bool m_bFlag_Obstacle_Left = false;
    //PCL obstacle Check//
    bool m_bFlag_Obstacle_PCL1 = false;
    bool m_bFlag_Obstacle_PCL2 = false;
    //Cygbot Check//
    bool m_bFlag_Obstacle_cygbot = false;
    //Error Flag//
    bool m_bumperhit_flag = false;
    bool m_emgpush_flag = false;
    bool BUMPER_BT = false;
    //Dynamic reconfigure flag//
    bool m_bTebMarker_reconfigure_flag = false;
    bool m_bflag_patrol = false;
    bool m_bflag_patrol2 = false;
    bool m_bflag_goto_cancel = false;
    bool m_bflagGo = false;
    bool m_bflagGo2 = false;
    bool m_bCorneringFlag = true;
    //no motion service call flag//
    bool m_bFlag_nomotion = true;
    bool m_bflag_auto_patrol = false;
    bool m_bFlag_Initialpose = false;
    //Setgoal_pub flag//
    bool m_bFlag_pub = false;

}FALG_VALUE;
FALG_VALUE _pFlag_Value;

class TETRA_SERVICE: public rclcpp::Node
{
public:
	TETRA_SERVICE() : Node("tetra_service")
	{
		//tf2_ros
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		//publish list/////////////////////////////////////////////////////////////////////////////////////
		cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		pose_reset_publisher = this->create_publisher<std_msgs::msg::Int32>("pose_reset", 10);
		initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

		//bumper_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("bumper_pointcloud", 10);

		//subscribe list////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		ar_tag_markers_subscriber = this->create_subscription<ar_track_alvar_msgs::msg::AlvarMarkers>(
			"ar_pose_marker", 100, std::bind(&TETRA_SERVICE::AR_tagCallback, this, _1));

		sick_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"scan", 100, std::bind(&TETRA_SERVICE::SickCallback, this, _1));
		
		cygbot_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"scan2", 100, std::bind(&TETRA_SERVICE::CygbotCallback, this, _1));

		bumper_subscriber = this->create_subscription<std_msgs::msg::Int32>(
			"bumper_data", 1, std::bind(&TETRA_SERVICE::BumperCallback, this, _1));

		emg_subscriber = this->create_subscription<std_msgs::msg::Int32>(
			"emg_state", 1, std::bind(&TETRA_SERVICE::EMGCallback, this, _1));

		tetra_battery_subscriber = this->create_subscription<std_msgs::msg::Int32>(
			"tetra_battery", 1, std::bind(&TETRA_SERVICE::BatteryCallback, this, _1));
		
		docking_status_subscriber = this->create_subscription<std_msgs::msg::Int32>(
			"docking_status", 1, std::bind(&TETRA_SERVICE::DockingStatusCallback, this, _1));
		
		conveyor_sensor_subscriber = this->create_subscription<std_msgs::msg::Int32>(
			"conveyor_sensor", 1, std::bind(&TETRA_SERVICE::ConveyorSensorCallback, this, _1));

		conveyor_movement_subscriber = this->create_subscription<std_msgs::msg::Int32>(
			"conveyor_movement", 1, std::bind(&TETRA_SERVICE::ConveyorMovementCallback, this, _1));

		joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
			"joy", 10, std::bind(&TETRA_SERVICE::joyCallback, this, _1));

		
		//Service list///////////////////////////////////////////////////////////////////////////////////////
		docking_cmd_srv = create_service<interfaces::srv::DockingControl>(
        	"docking_cmd", 
		std::bind(&TETRA_SERVICE::Docking_Command, this, std::placeholders::_1, std::placeholders::_2));

		docking_stop_cmd_srv = create_service<interfaces::srv::DockingStop>(
        	"docking_stop_cmd", 
		std::bind(&TETRA_SERVICE::Docking_Stop_Command, this, std::placeholders::_1, std::placeholders::_2));

		save_map_cmd_srv = create_service<interfaces::srv::SaveMap>(
        	"save_map_cmd", 
		std::bind(&TETRA_SERVICE::Save_Map_Command, this, std::placeholders::_1, std::placeholders::_2));


		//Client list///////////////////////////////////////////////////////////////////////////////////////
		led_control_client = this->create_client<interfaces::srv::LedControl>("led_cmd");
		led_toggle_control_client = this->create_client<interfaces::srv::LedToggleControl>("led_toggle_cmd");
		toggle_on_client = this->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");
		imu_reset_client = this->create_client<interfaces::srv::ImuReset>("all_data_reset");
		
		//PARAM

		//Timer
		timer_ = this->create_wall_timer(20ms, std::bind(&TETRA_SERVICE::DockingThread_function, this));

	}

	////value//////////////////////////////////////////////////////////////////////////////
	rclcpp::Time current_time;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	geometry_msgs::msg::Twist cmd;
	std_msgs::msg::Int32 pose_reset_data;
	geometry_msgs::msg::PoseWithCovarianceStamped initPose;
	rclcpp::TimerBase::SharedPtr timer_;
	//sensor_msgs::msg::PointCloud2 pointCloudMsg;

	//Publisher ////////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pose_reset_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;
	//rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bumper_pointcloud_publisher;

	//Subscription ////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Subscription<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr ar_tag_markers_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sick_scan_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr cygbot_scan_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bumper_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr emg_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tetra_battery_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr docking_status_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr conveyor_sensor_subscriber;   //Option
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr conveyor_movement_subscriber; //Option
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

	//Joystick _Logitech F710 Gamepad
	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
	{
		if(joy->buttons[6]) //LT button
		{	
			//Docking Start//
			_pRobot.m_iDocking_id = 0;
			m_iDocking_CommandMode = 1;
			_pFlag_Value.m_bfalg_DockingExit = false;
		}
		if(joy->buttons[7]) //RT button
		{	
			//Docking Stop//
			m_iDocking_CommandMode = 0;
			_pFlag_Value.m_bfalg_DockingExit = true;

		}
	}

	//Subscribe Callback Function
	void AR_tagCallback(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr msg) 
	{
		if(!msg->markers.empty()) 
		{
			//add...find ID array Index Loop...
			for (int i = 0; i < msg->markers.size(); i++)
			{
				if (_pAR_tag_pose.m_iSelect_AR_tag_id == msg->markers[i].id)
				{
					_pAR_tag_pose.m_iAR_tag_id_Index = i;
				}
			}

			//AR_Tag data update...
			_pAR_tag_pose.m_iAR_tag_id = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].id;
			_pAR_tag_pose.m_fAR_tag_pose_x = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.x;
			_pAR_tag_pose.m_fAR_tag_pose_y = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.y;
			_pAR_tag_pose.m_fAR_tag_pose_z = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.z;
			_pAR_tag_pose.m_fAR_tag_orientation_x = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.x;
			_pAR_tag_pose.m_fAR_tag_orientation_y = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.y;
			_pAR_tag_pose.m_fAR_tag_orientation_z = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.z;
			_pAR_tag_pose.m_fAR_tag_orientation_w = msg->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.w;
			
			//< Declaration of quaternion
			tf2::Quaternion q;
			q.setW(_pAR_tag_pose.m_fAR_tag_orientation_w);
			q.setX(_pAR_tag_pose.m_fAR_tag_orientation_x);
			q.setY(_pAR_tag_pose.m_fAR_tag_orientation_y);
			q.setZ(_pAR_tag_pose.m_fAR_tag_orientation_z);
			//< quaternion -> rotation Matrix
			tf2::Matrix3x3 m(q);
			//< rotation Matrix - > quaternion
			m.getRotation(q);
			//< rotation Matrix -> rpy
			m.getRPY(_pAR_tag_pose.m_fAR_tag_roll, _pAR_tag_pose.m_fAR_tag_pitch, _pAR_tag_pose.m_fAR_tag_yaw);
			_pAR_tag_pose.m_fPositioning_Angle = _pAR_tag_pose.m_fAR_tag_pitch * (180.0/M_PI);

			//Transform Axis
			_pAR_tag_pose.m_transform_pose_x = _pAR_tag_pose.m_fAR_tag_pose_z;
			_pAR_tag_pose.m_transform_pose_y = _pAR_tag_pose.m_fAR_tag_pose_x;

		}
		else
		{
			_pAR_tag_pose.m_iAR_tag_id = -1;
		}
	}

	//Sick Tim571 Lidar Callback
	void SickCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		int size = msg->ranges.size();
		//printf("sick_lidar_size: %d \n", size);

		//Right Check//
		int R_minIndex = 0;
		int R_maxIndex = 135;
		int R_closestIndex = -1;
		double R_minVal = 0.4;

		for (int i = R_minIndex; i < R_maxIndex; i++)
		{
			if ((msg->ranges[i] <= R_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
			{
				R_minVal = msg->ranges[i];
				R_closestIndex = i;
			}
		}
		//printf("R_closestIndex: %d || check: %f \n" , R_closestIndex, msg->ranges[R_closestIndex]);
		if(R_closestIndex > 0)
		{
			_pFlag_Value.m_bFlag_Obstacle_Right = true;
			//printf(" Lidar Obstacle_Right !! \n");
		}
		else
			_pFlag_Value.m_bFlag_Obstacle_Right = false;

		/**************************************************************************************************************************/
		//Center Check//
		int C_minIndex = 270;
		int C_maxIndex = 540;
		int C_closestIndex = -1;
		double C_minVal = 0.8; //0.3

		for (int i = C_minIndex; i < C_maxIndex; i++)
		{
			if ((msg->ranges[i] <= C_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
			{
				C_minVal = msg->ranges[i];
				C_closestIndex = i;
			}
		}
		//printf("C_closestIndex: %d || check: %f \n" , C_closestIndex, msg->ranges[C_closestIndex]);
		if(C_closestIndex > 0)
			_pFlag_Value.m_bFlag_Obstacle_Center = true;
		else
			_pFlag_Value.m_bFlag_Obstacle_Center = false;

		/**************************************************************************************************************************/
		//Left Check//
		int L_minIndex = 675;
		int L_maxIndex = 810;
		int L_closestIndex = -1;
		double L_minVal = 0.4;

		for (int i = L_minIndex; i < L_maxIndex; i++)
		{
			if ((msg->ranges[i] <= L_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
			{
				L_minVal = msg->ranges[i];
				L_closestIndex = i;
			}
		}
		//printf("L_closestIndex: %d || check: %f \n" , L_closestIndex, msg->ranges[L_closestIndex]);
		if(L_closestIndex > 0)
		{
			_pFlag_Value.m_bFlag_Obstacle_Left = true;
			//printf(" Lidar Obstacle_Left !! \n");
		}
		else
			_pFlag_Value.m_bFlag_Obstacle_Left = false;

	}

	//Cygbot Lidar Callback
	void CygbotCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		int size = msg->ranges.size();
		//printf("cygbot_size: %d \n",size);
		int cygbot_minIndex = 1;
		int cygbot_maxIndex = 160;
		int cygbot_closestIndex = -1;
		double cygbot_minVal = 0.3;

		for (int i = cygbot_minIndex; i < cygbot_maxIndex; i++)
		{
			if ((msg->ranges[i] <= cygbot_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
			{
				cygbot_minVal = msg->ranges[i];
				cygbot_closestIndex = i;
			}
		}
		//printf("cygbot_closestIndex: %d || check: %f \n" , cygbot_closestIndex, msg->ranges[cygbot_closestIndex]);
		if(cygbot_closestIndex > 0)
		{
			_pFlag_Value.m_bFlag_Obstacle_cygbot = true;
			//printf("Rear Obstacle Check !!!!! \n");
		}
		else
			_pFlag_Value.m_bFlag_Obstacle_cygbot = false;

	}

	void BumperCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		_pRobot.m_iCallback_Bumper = msg->data;

		// // Fill in the header
        // pointCloudMsg.header.stamp = this->get_clock()->now();
        // pointCloudMsg.header.frame_id = "front_bumper";
		// // Set the height and width of the point cloud
        // pointCloudMsg.height = 1;
        // pointCloudMsg.width = 1;
		
		
		if(_pRobot.m_iCallback_Bumper != 0)
		{
			// // Add PointField information (replace these with your own fields)
			// // For example, x, y, z coordinates
			// pointCloudMsg.fields.emplace_back("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
			// pointCloudMsg.fields.emplace_back("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
			// pointCloudMsg.fields.emplace_back("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);

			if(!_pFlag_Value.m_bFlag_Disable_bumper)
			{
				LedToggleControl_Call(1, 10,100,10,1);
    			ToggleOn_Call(18);
				printf("[Bumper] Push Bumper!! _ RED LED On \n");
		
				// if(_pFlag_Value.m_bflagGo)
				// {
				// 	goto_goal_id.id = "";
				// 	ROS_INFO("[Bumper On]Goto Cancel call");
				// 	GotoCancel_pub.publish(goto_goal_id);
				// 	_pFlag_Value.m_bflagGo = false;
				// 	_pFlag_Value.m_bflagGo2 = true;
				// 	if(_pFlag_Value.BUMPER_BT)
				// 		ex_iDocking_CommandMode = 100;
				// }
			}
			_pFlag_Value.m_bumperhit_flag = true;
		}
		else
		{			
			// pointCloudMsg.fields.emplace_back("x", 0, sensor_msgs::msg::PointField::FLOAT32, 0);
			// pointCloudMsg.fields.emplace_back("y", 4, sensor_msgs::msg::PointField::FLOAT32, 0);
			// pointCloudMsg.fields.emplace_back("z", 8, sensor_msgs::msg::PointField::FLOAT32, 0);

			if(_pFlag_Value.m_bumperhit_flag)
			{
				LedToggleControl_Call(1,3,100,3,1);
				ToggleOn_Call(63); //White led
				_pFlag_Value.m_bumperhit_flag = false;
			}

		}
		// // Set the point step and row step
        // pointCloudMsg.point_step = 16;
        // pointCloudMsg.row_step = pointCloudMsg.point_step * pointCloudMsg.width;
        // // Set the is_dense flag
        // pointCloudMsg.is_dense = true;
        // // Resize the data array and fill it with zeros (replace this with your own data)
        // pointCloudMsg.data.resize(pointCloudMsg.row_step * pointCloudMsg.height, 0);
        // // Publish the PointCloud2 message
        // bumper_pointcloud_publisher->publish(std::move(pointCloudMsg));

	}

	void EMGCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		_pRobot.m_iCallback_EMG = msg->data;
		if(_pRobot.m_iCallback_EMG != 0)
		{
			LedToggleControl_Call(1, 10,100,10,1);
			ToggleOn_Call(18);
			//printf("[EMG] Push EMG button!! _ RED LED On \n");
			_pFlag_Value.m_emgpush_flag = false;
		}
		else
		{
			if(!_pFlag_Value.m_emgpush_flag)
			{
				LedToggleControl_Call(1, 3,100,3,1);
				ToggleOn_Call(63);
				_pFlag_Value.m_emgpush_flag = true;
			}
		}
	}

	void BatteryCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		_pRobot.m_iCallback_Battery = msg->data;
	}

	void DockingStatusCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		_pRobot.m_iCallback_Charging_status = msg->data;
	}

	void ConveyorSensorCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		_pRobot.m_iConveyor_Sensor_info = msg->data;
	}

	void ConveyorMovementCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		_pRobot.m_iConveyor_movement = msg->data;
	}


	//Client Callback Function ///////////////////////////////////////////////////////////////
	void LedControl_callback(rclcpp::Client<interfaces::srv::LedControl>::SharedFuture future)
	{
		//RCLCPP_INFO(this->get_logger(), "response LedControl_callback");

	}
	void LedToggleControl_callback(rclcpp::Client<interfaces::srv::LedToggleControl>::SharedFuture future)
	{
		//RCLCPP_INFO(this->get_logger(), "response LedToggleControl_callback");

	}
	void ToggleOn_callback(rclcpp::Client<interfaces::srv::ToggleOn>::SharedFuture future)
	{
		//RCLCPP_INFO(this->get_logger(), "response ToggleOn_callback");
	}
	
	void ImuReset_callback(rclcpp::Client<interfaces::srv::ImuReset>::SharedFuture future)
	{
		//RCLCPP_INFO(this->get_logger(), "response ImuReset_callback");
	}  


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	bool Depart_Station2Move()
	{
		bool bResult = false;
		if(_pAR_tag_pose.m_transform_pose_x <= 0.6) //600mm depart move
		{
			if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
			{
				cmd.linear.x =  0.0; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
				bResult = false;
			}
			else
			{
				cmd.linear.x =  -0.05; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
				bResult = false;
			}    
		}
		else
		{
			cmd.linear.x =  0.0; 
			cmd.angular.z = 0.0;
			cmd_vel_publisher->publish(cmd);

			//setGoal(goal);

			m_iDocking_CommandMode = 0;

			bResult = true;
		}
		
		return bResult;

	}


	double Rotation_Movement()
	{
		double iResult = 0.1;

		if(_pFlag_Value.m_bFlag_Obstacle_Right)
		{
			m_iRotation_Mode = 1;
			printf(" CCW Rotation--- \n");
		}
		else if(_pFlag_Value.m_bFlag_Obstacle_Left)
		{
			m_iRotation_Mode = 2;
			printf(" CW Rotation+++ \n");
		}

		switch(m_iRotation_Mode)
		{
			case 0:
				iResult = 0.1;
				break;
			case 1:
				iResult = -0.1;
				break;
			case 2:
				iResult = 0.1;
				break;
		}

		return iResult;
	}

	//Charging Station Docking Function//
	bool ChargingStation_tracking(bool bOn, int marker_id)
	{
		bool bResult = false;
		//Todo.....
		if(bOn)
		{
			float m_fdistance = 0.0;
			printf("_pAR_tag_pose.m_iAR_tag_id: %d \n", _pAR_tag_pose.m_iAR_tag_id);
			if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
			{	
				m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
				printf("master_distance ->: %.5f \n", m_fdistance);
				if(m_fdistance > 0.41 /*&& m_fdistance < 1.5*/)
				{
					printf("master_distance Go: %.5f \n", m_fdistance);
					cmd.linear.x = 1.0 * (m_fdistance /1.2) * 0.15; 
					//printf("linear velocity: %.2f \n", cmd.linear.x );

					if(cmd.linear.x > 1.0)
					{
						//Linear Over speed exit loop...
						cmd.linear.x =  0.0; 
						cmd.angular.z = 0.0;
						printf("[Linear Over speed]: follower is closing \n");
						return false;
					}
						
					cmd.angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
					//printf("angular velocity: %.2f \n", cmd.angular.z);
					if((cmd.angular.z > 1.0) || (cmd.angular.z < -1.0))
					{
						//Angular Over speed exit loop......
						cmd.linear.x =  0.0; 
						cmd.angular.z = 0.0;
						printf("[Angular Over speed]: follower is closing \n");
						return false;
					}

					cmd_vel_publisher->publish(cmd);
					
				}
				else
				{
					cmd.linear.x =  0.0;
					cmd_vel_publisher->publish(cmd);
					printf("Tracking STOP !! \n");
					printf("master_distance Stop: %.5f \n", m_fdistance);

					_pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_fAR_tag_pitch;

					if(_pAR_tag_pose.m_target_yaw <= 0.0174533 && _pAR_tag_pose.m_target_yaw >= -0.0174533) //+- 1.0deg
					{
						m_iDocking_CommandMode = 4;
						printf("[OK]_pAR_tag_pose.m_target_yaw: %.5f \n", _pAR_tag_pose.m_target_yaw);
					}
					else
					{
						m_iDocking_CommandMode = 3;
						printf("[NG]_pAR_tag_pose.m_target_yaw: %.5f \n", _pAR_tag_pose.m_target_yaw);
					}

					m_iNoMarker_cnt = 0;
				}
			}
			else
			{
				printf("No Marker, Rotation Movement !! \n");
				cmd.angular.z = Rotation_Movement(); //0.1;
				cmd_vel_publisher->publish(cmd);

				if(m_iNoMarker_cnt > 4000) //retry timeout!!
				{
					m_iNoMarker_cnt = 0;
					cmd.linear.x =  0.0; 
					cmd.angular.z = 0.0;
					cmd_vel_publisher->publish(cmd);
					printf("DockingStation scan Fail !! \n");
					m_iDocking_CommandMode = 9;
				}
				else
				{
					m_iNoMarker_cnt++;
				}

			}

		}
		else
		{
			cmd.linear.x =  0.0; 
			cmd.angular.z = 0.0;
			cmd_vel_publisher->publish(cmd);
			printf("Docking Loop STOP!_not find Marker!! \n");
			m_iDocking_CommandMode = 0;
		}
			
		bResult = true;
		return bResult;
	}

	bool ChargingStation_Yaw_tracking()
	{ 
		bool bResult = false;
	
		if(m_iTrun_cnt < 50)
		{
			if(_pAR_tag_pose.m_target_yaw > 0)
			{
				printf("[++dir] _pAR_tag_pose.m_target_yaw: %.5f \n", _pAR_tag_pose.m_target_yaw);
				cmd.angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
				cmd_vel_publisher->publish(cmd);
				m_iRotation_Mode = 2;
				//sleep(1);
				m_iTrun_cnt++;
			}
			else
			{
				printf("[--dir] _pAR_tag_pose.m_target_yaw: %.5f \n", _pAR_tag_pose.m_target_yaw);
				cmd.angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
				cmd_vel_publisher->publish(cmd);
				m_iRotation_Mode = 1;
				//sleep(1);
				m_iTrun_cnt++;

			}
		}
		else
		{
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.0;
			cmd_vel_publisher->publish(cmd);

			m_iTrun_cnt = 0;
			m_iDocking_CommandMode = 31;
		}
		
		bResult = true;
		return bResult;
	}

	bool ChargingStation_back_step()
	{
		bool bResult = false;
		float m_fdistance = 0.0;
		m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
		printf("Yaw_tracking_distance: %.5f \n", m_fdistance);

		if(m_iBack_cnt < 50)
		{
			if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
			{
				cmd.angular.z = 0.0;
				cmd.linear.x = 0.0;
				cmd_vel_publisher->publish(cmd);
				printf("Drive in reverse STOP_obstacle check! \n");
					
			}
			else
			{
				cmd.angular.z = 0.0;
				cmd.linear.x = -1.0 * m_fdistance * 0.2;
				printf("Drive in reverse.... \n");
				cmd_vel_publisher->publish(cmd);
				m_iBack_cnt++;
			}
		}
		else
		{
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.0;
			cmd_vel_publisher->publish(cmd);

			m_iBack_cnt = 0;
			m_iDocking_CommandMode = 2;
		}


		bResult = true;
		return bResult;
	}

	bool ChargingStation_tracking2(int marker_id)
	{
		bool bResult = false;
		
		float m_fdistance = 0.0;
		if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
		{
			m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
			//printf("master_distance: %.5f \n", m_fdistance);
			if(_pRobot.m_iCallback_Charging_status < 2)
			{
				cmd.linear.x = 1.0 * (m_fdistance /1.2) * 0.1; //max speed 0.1m/s
				if(cmd.linear.x > 1.0)
				{
					//Linear Over speed exit loop......
					cmd.linear.x =  0.0; 
					cmd.angular.z = 0.0;
					printf("[Linear Over speed]: follower is closing \n");
					return false;
				}
				
				cmd.angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
				if((cmd.angular.z > 1.0) || (cmd.angular.z < -1.0))
				{
					//Angular Over speed exit loop......
					cmd.linear.x =  0.0; 
					cmd.angular.z = 0.0;
					printf("[Angular Over speed]: follower is closing \n");
					return false;
				}
				
				cmd_vel_publisher->publish(cmd);
			}
			else
			{
				cmd.linear.x =  0.0; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
				printf("Tracking STOP & Docking Finish !! \n");
				m_iDocking_CommandMode = 6; //5;
				m_iNoMarker_cnt = 0;
			}
		}
		else
		{
			cmd.linear.x =  0.0; 
			cmd.angular.z = 0.0;
			cmd_vel_publisher->publish(cmd);
			printf("No Marker 2! \n");
			if(m_iNoMarker_cnt >= 10)
			{
				m_iNoMarker_cnt = 0;
				m_iDocking_CommandMode = 6;
				printf("No Marker 2_Timeout! \n");
			}
			else
			{
				m_iNoMarker_cnt++;
			}
		}

		bResult = true;
		return bResult;
	}

	void Docking_EXIT()
	{
		cmd.linear.x =  0.0; 
		cmd.angular.z = 0.0;
		cmd_vel_publisher->publish(cmd);
		m_iDocking_CommandMode = 0;
		printf("[EXIT]: Docking Exit !! \n");
	}

	void Reset_EKF_SetPose()
	{
		initPose.header.stamp = rclcpp::Time();
		initPose.header.frame_id = "map";
		//position
		initPose.pose.pose.position.x = 0.0;
		initPose.pose.pose.position.y = 0.0;
		initPose.pose.pose.position.z = 0.0;
		//orientation
		initPose.pose.pose.orientation.x = 0.0;
		initPose.pose.pose.orientation.y = 0.0;
		initPose.pose.pose.orientation.z = 0.0;
		initPose.pose.pose.orientation.w = 1.0;
		initPose.pose.covariance[0] = 0.25;
		initPose.pose.covariance[6 * 1 + 1] = 0.25;
		initPose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

		initial_pose_publisher->publish(initPose);

	}

	bool Marker_Reset_Robot_Pose()
	{
		_pFlag_Value.m_bFlag_nomotion = false;
		
		bool bResult = false;
		string m_strFilePathName;
		string landmark_name = "marker_" + std::to_string(_pAR_tag_pose.m_iAR_tag_id);
		if (_pAR_tag_pose.m_iAR_tag_id == -1) 
		{
			bResult = false;
			return -1;
		}
		else 
		{
			printf("#Docking Reset Marker_ID: %d \n", _pAR_tag_pose.m_iAR_tag_id);
			bResult = true;
		}

	
		m_strFilePathName = "/home/tetra/LANDMARK/" + landmark_name + ".txt";
		fp = fopen(m_strFilePathName.c_str(), "r");

		if(_pAR_tag_pose.m_iAR_tag_id > 0 && fp == NULL) 
		{
			_pFlag_Value.m_bFlag_nomotion = true;
			return false;
		}

		if (fp != NULL) //File Open Check
		{
			while (!feof(fp))
			{
				if (fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
				{
					char* ptr = strtok(Textbuffer, ",");
					int icnt = 0;

					while (ptr != NULL)
					{
						ptr = strtok(NULL, ",");
						switch (icnt)
						{
						case 6:
							_pLandMarkPose.init_position_x = atof(ptr);
							break;
						case 7:
							_pLandMarkPose.init_position_y = atof(ptr);
							break;
						case 8:
							_pLandMarkPose.init_position_z = atof(ptr);
							break;
						case 9:
							_pLandMarkPose.init_orientation_z = atof(ptr);
							break;
						case 10:
							_pLandMarkPose.init_orientation_w = atof(ptr);
							break;
						}

						icnt++;
					}
				}
			}
			fclose(fp);
			initPose.header.stamp = rclcpp::Time();
			initPose.header.frame_id = "map";
			//position
			initPose.pose.pose.position.x = _pLandMarkPose.init_position_x;
			initPose.pose.pose.position.y = _pLandMarkPose.init_position_y;
			initPose.pose.pose.position.z = _pLandMarkPose.init_position_z;
			//orientation
			initPose.pose.pose.orientation.x = 0.0;
			initPose.pose.pose.orientation.y = 0.0;
			initPose.pose.pose.orientation.z = -1.0 * _pLandMarkPose.init_orientation_z;
			initPose.pose.pose.orientation.w = _pLandMarkPose.init_orientation_w;

			initPose.pose.covariance[0] = 0.25;
			initPose.pose.covariance[6 * 1 + 1] = 0.25;
			initPose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

			//publish msg
			initial_pose_publisher->publish(initPose);
			printf("$$$$ init_position_x: %f , init_position_y: %f \n", _pLandMarkPose.init_position_x, _pLandMarkPose.init_position_y);
		}
		else
		{
			bResult = false;
		}

		_pFlag_Value.m_bFlag_nomotion = true;

		return bResult;
	}

	void Reset_Robot_Pose()
	{
		if(_pRobot.HOME_id == _pAR_tag_pose.m_iAR_tag_id) // Same as Home ID... Loop 
		{
			_pFlag_Value.m_bFlag_nomotion = false;
			//IMU reset//
			ImuReset_Call();

			//tetra odometry Reset//
			pose_reset_data.data = 1;
			pose_reset_publisher->publish(pose_reset_data);
			sleep(1);

			Reset_EKF_SetPose();
		}

		Marker_Reset_Robot_Pose();

		_pFlag_Value.m_bFlag_Initialpose = true;
		LedToggleControl_Call(1,3,100,3,1);
    	ToggleOn_Call(63);
	}

	void DockingThread_function()
	{
			switch(m_iDocking_CommandMode)
			{
				case 0:
					//printf(" #####Timer Loop ######\n");
					break;
				/****************************************************************/
				// Station Docking Loop//
				case 1:
					printf("Docking Loop 1 Start... \n");
					LedToggleControl_Call(1,3,100,3,100);
    				ToggleOn_Call(63);
					m_iDocking_CommandMode = 2;
					break;
				case 2:
					printf("Docking Loop 2... \n");
					_pRobot.HOME_id = _pRobot.m_iDocking_id;
					ChargingStation_tracking(true, _pRobot.HOME_id);
					if(_pRobot.m_bFalg_DockingExit)
					{
						Docking_EXIT();
						m_iDocking_CommandMode = 0;
					}
					break;
				case 3:
					printf("Docking Loop 3... \n");
					//_pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_fAR_tag_pitch;
					ChargingStation_Yaw_tracking();
					if(_pFlag_Value.m_bfalg_DockingExit)
					{
					    Docking_EXIT();
					    m_iDocking_CommandMode = 0;
					}
					break;
				case 4:
					printf("Docking Loop 4... \n");
					ChargingStation_tracking2(_pRobot.HOME_id);
					if(_pFlag_Value.m_bfalg_DockingExit)
					{
					    Docking_EXIT();
					    m_iDocking_CommandMode = 0;
					}
					break;
				case 5:
					printf("case 5... \n");
					// Approach_Station2Move();
					break;
				case 6:
					printf("Docking End Loop 6... \n");
					LedToggleControl_Call(1,3,100,3,100);
    				ToggleOn_Call(9);
					printf("TETRA POSE Reset! \n");
					////PoseReset_call
					Reset_Robot_Pose();
					LedToggleControl_Call(1, 5,100,5,1);
					ToggleOn_Call(63);

					m_iDocking_CommandMode = 0;
					break;
				case 9:
					printf("Docking FAIL ! \n");
					LedToggleControl_Call(1, 10,100,10,1);
    				ToggleOn_Call(18);

					// m_iDocking_CommandMode = 119;
					m_iDocking_CommandMode = 0;
					break;
				case 10:
					printf("case 10.. \n");
					Depart_Station2Move();
					break;
				/****************************************************************/
				// Conveyor Docking Loop//
				case 11:
					
					break;
				case 12:
					
					break;
				case 13:
					
					break;
				case 14:
					
					break;
				case 15:
					
					break;
				case 16:
					
					break;
				/****************************************************************/
				case 30:
					
					break;
				case 31:
					printf("Docking Loop 31... \n");
					ChargingStation_back_step();
					break;
				case 32:
					printf("Docking Loop 32... \n");
					
					break;
				case 100: //Check Bumper
					//BumperCollision_Behavior();
					break;
				case 119: //Retry Goto Home...
					// printf(" Retry Goto Home ! \n");
					// //costmap clear call//
					// clear_costmap_client.call(m_request);
					// _pGoal_pose.goal_positionX = _pHomePose.HOME_dPOSITION_X;
					// _pGoal_pose.goal_positionY = _pHomePose.HOME_dPOSITION_Y;
					// _pGoal_pose.goal_positionZ = _pHomePose.HOME_dPOSITION_Z;
					// _pGoal_pose.goal_quarterX = _pHomePose.HOME_dQUATERNION_X;
					// _pGoal_pose.goal_quarterY = _pHomePose.HOME_dQUATERNION_Y;
					// _pGoal_pose.goal_quarterZ = _pHomePose.HOME_dQUATERNION_Z;
					// _pGoal_pose.goal_quarterW = _pHomePose.HOME_dQUATERNION_W;

					// goto_goal_id.id = "HOME";
					// setGoal(goal);
					// _pFlag_Value.m_bflag_ComebackHome = true;
					// m_iDocking_CommandMode = 0;
					break;
				default:
					break;
			}
	}


public:
	//Server Function ///////////////////////////////////////////////////////////////
	bool Docking_Command(
		const std::shared_ptr<interfaces::srv::DockingControl::Request> request, 
		const std::shared_ptr<interfaces::srv::DockingControl::Response> response)
	{
		bool bResult = false;
		_pRobot.m_iDocking_id = request->id;
		m_iDocking_CommandMode = request->mode;

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Docking_Stop_Command(
		const std::shared_ptr<interfaces::srv::DockingStop::Request> request, 
		const std::shared_ptr<interfaces::srv::DockingStop::Response> response)
	{
		bool bResult = false;

		printf("[STOP Call]: Docking Stop !! \n");
		
		Docking_EXIT();

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Save_Map_Command(
		const std::shared_ptr<interfaces::srv::SaveMap::Request> request, 
		const std::shared_ptr<interfaces::srv::SaveMap::Response> response)
	{
		bool bResult = false;
		printf("Save Map Call _ %s \n", request->map_name.c_str());

		//call rosrun command//
		string str_command = "gnome-terminal -- /home/tetra/mapsave.sh ";
		string str_command2 = str_command + request->map_name.c_str();

		std::vector<char> writable1(str_command2.begin(), str_command2.end());
		writable1.push_back('\0');
		char* ptr1 = &writable1[0];
		int iResult = std::system(ptr1);
		
		response->command_result = bResult;
		return true;

	}
	
	//Client Function ///////////////////////////////////////////////////////////////
	void LedControl_Call(int id, int led_brightness) 
    {
        auto request = std::make_shared<interfaces::srv::LedControl::Request>();
        request->id = id;
        request->led_brightness = led_brightness;

        auto result_future = led_control_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::LedControl_callback, this, std::placeholders::_1));
    }

	void LedToggleControl_Call(int de_index, int light_accel, int led_high_brightness, int light_decel, int led_low_brightness) 
    {
        auto request = std::make_shared<interfaces::srv::LedToggleControl::Request>();
        request->de_index = de_index;
        request->light_accel = light_accel;
		request->led_high_brightness = led_high_brightness;
		request->light_decel = light_decel;
		request->led_low_brightness = led_low_brightness;


        auto result_future = led_toggle_control_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::LedToggleControl_callback, this, std::placeholders::_1));
    }

	void ToggleOn_Call(int id) 
    {
        auto request = std::make_shared<interfaces::srv::ToggleOn::Request>();
        request->id = id;

        auto result_future = toggle_on_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::ToggleOn_callback, this, std::placeholders::_1));
    }

	void ImuReset_Call() 
    {
        auto request = std::make_shared<interfaces::srv::ImuReset::Request>();

        auto result_future = imu_reset_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::ImuReset_callback, this, std::placeholders::_1));
    }
 
private:
	////Client/////////////////////////////////////////////////////////////////////////////////
	rclcpp::Client<interfaces::srv::LedControl>::SharedPtr led_control_client;
	rclcpp::Client<interfaces::srv::LedToggleControl>::SharedPtr led_toggle_control_client;
	rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client;
	rclcpp::Client<interfaces::srv::ImuReset>::SharedPtr imu_reset_client;

	////Service/////////////////////////////////////////////////////////////////////////////////
	rclcpp::Service<interfaces::srv::DockingControl>::SharedPtr docking_cmd_srv;
	rclcpp::Service<interfaces::srv::DockingStop>::SharedPtr docking_stop_cmd_srv;
	rclcpp::Service<interfaces::srv::SaveMap>::SharedPtr save_map_cmd_srv;
	
};


//Main Loop//
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<TETRA_SERVICE>();
	TETRA_SERVICE tetra_service;

	//init
	tetra_service.current_time = node->now();
	rclcpp::WallRate loop_rate(30);

	//LED On
	tetra_service.LedToggleControl_Call(1,3,100,3,1);
    tetra_service.ToggleOn_Call(63);

	/////////////////////////////////////////////////////////////////
	printf("□□■■■■□□□■■■■■■□□■■■■■□□□■□□□□□■□■□□□□■■■■□□□■■■■■■□\n");
	printf("□■□□□□■□□■□□□□□□□■□□□□■□□■□□□□□■□■□□□■□□□□■□□■□□□□□□\n");
	printf("□■□□□□■□□■□□□□□□□■□□□□■□□■□□□□□■□■□□■□□□□□□□□■□□□□□□\n");
	printf("□■□□□□□□□■□□□□□□□■□□□□■□□□■□□□■□□■□□■□□□□□□□□■□□□□□□\n");
	printf("□□■■□□□□□■■■■■■□□■■■■■□□□□■□□□■□□■□□■□□□□□□□□■■■■■■□\n");
	printf("□□□□■■□□□■□□□□□□□■□□□□■□□□■□□□■□□■□□■□□□□□□□□■□□□□□□\n");
	printf("□□□□□□■□□■□□□□□□□■□□□□■□□□□■□■□□□■□□■□□□□□□□□■□□□□□□\n");
	printf("□■□□□□■□□■□□□□□□□■□□□□■□□□□■□■□□□■□□■□□□□□□□□■□□□□□□\n");
	printf("□■□□□□■□□■□□□□□□□■□□□□■□□□□□■□□□□■□□□■□□□□■□□■□□□□□□\n");
	printf("□□■■■■□□□■■■■■■□□■□□□□■□□□□□■□□□□■□□□□■■■■□□□■■■■■■□\n");
	/////////////////////////////////////////////////////////////////   


	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		//To do...


		
		loop_rate.sleep();
    }

	rclcpp::shutdown();
    return 0;
}
