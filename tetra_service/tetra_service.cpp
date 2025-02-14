#include "rclcpp/rclcpp.hpp"
//nav2 msg//
#include "nav2_msgs/msg/particle.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_amcl/amcl_node.hpp"
//std//
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
//geometry msg//
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point.hpp"
//sensor msg//
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

//tf2// 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//Custom Service//
#include "interfaces/srv/led_control.hpp"
#include "interfaces/srv/led_toggle_control.hpp"
#include "interfaces/srv/toggle_on.hpp"
#include "interfaces/srv/docking_control.hpp" //docking
#include "interfaces/srv/docking_stop.hpp" //docking_stop
#include "interfaces/srv/imu_reset.hpp" //imu reset
#include "interfaces/srv/save_map.hpp" //Map save service
#include "interfaces/srv/run_mapping.hpp" //cartographer run service
#include "interfaces/srv/run_navigation.hpp" //NAV2 run service
#include "interfaces/srv/get_information.hpp" //Get Information service
#include "interfaces/srv/set_location.hpp" //Save Location to string Location name
#include "interfaces/srv/goto_location.hpp" //Goto string Location name
#include "interfaces/srv/goto_location2.hpp" //Goto location Pose
#include "interfaces/srv/goto_cancel.hpp" //Goto Command Cancel
#include "interfaces/srv/set_maxspeed.hpp" //Set Maxspeed

//Virtual Obstacles add...
//#include "interfaces/srv/add_virtual_obstacles.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//Action//
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//apriltag_ros
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

//robot_localization Client service
#include "robot_localization/srv/set_pose.hpp"

#include <chrono>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <atomic>

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;


#define HIGH_BATTERY 95
#define LOW_BATTERY 15
#define MAX_RETRY_CNT 10
#define BUF_LEN 4096
int m_dHome_ID = 0;

FILE *fp;
int status;
char Textbuffer[BUF_LEN];

bool m_bActive_map = false;
int  m_iDocking_CommandMode = 0;
int  m_iNoMarker_cnt = 0;
int  m_iRotation_Mode = 0; //Docking Rotation Mode Select
int  m_iTrun_cnt = 0;
int  m_iBack_cnt = 0;
int  m_iPose_reset_cnt = 0;
//ros2 launch mode check//
int  ex_ilaunchMode = 0;

//CALC TF distance data
double m_dTF_calc_poseX = 0.0;
double m_dTF_calc_poseY = 0.0;
double m_dTF_calc_theta = 0.0;

typedef struct ODOMETRY
{
  double dOdom_position_x = 0.0;
  double dOdom_position_y = 0.0;
  double dOdom_position_z = 0.0;
  double dOdom_quaternion_x = 0.0;
  double dOdom_quaternion_y = 0.0;
  double dOdom_quaternion_z = 0.0;
  double dOdom_quaternion_w = 1.0;

  double dOdom_euler_roll = 0.0;
  double dOdom_euler_pitch = 0.0;
  double dOdom_euler_yaw = 0.0;

  //calc rad to degree
  double dOdom_theta_deg = 0.0;

}ODOMETRY;
ODOMETRY _pOdometry;

typedef struct HOME_POSE
{
    string HOME_strLOCATION = "HOME";
    double HOME_dPOSITION_X = 0.8;
    double HOME_dPOSITION_Y = 0.0;
    double HOME_dPOSITION_Z = 0.0;
    double HOME_dQUATERNION_X = 0.0;
    double HOME_dQUATERNION_Y = 0.0;
    double HOME_dQUATERNION_Z = 0.0;
    double HOME_dQUATERNION_W = 1.0;

}HOME_POSE;
HOME_POSE _pHomePose;

//goal_pose...
typedef struct GOAL_POSE
{
    double goal_positionX = 0.0;
    double goal_positionY = 0.0;
    double goal_positionZ = 0.0;
    double goal_quarterX = 0.0;
    double goal_quarterY = 0.0;
    double goal_quarterZ = 0.0;
    double goal_quarterW = 1.0;

}GOAL_POSE;
GOAL_POSE _pGoal_pose;

//April_TAG Pose
typedef struct AR_TAG_POSE
{
	string m_strFamily = "tag36h11:";
    int m_iSelect_April_tag_id = 0;
    int m_iApril_tag_id = -1;
    double m_dApril_tag_pose_x = 0.0;
    double m_dApril_tag_pose_y = 0.0;
    double m_dApril_tag_pose_z = 0.0;
    double m_dApril_tag_orientation_x = 0.0;
    double m_dApril_tag_orientation_y = 0.0;
    double m_dApril_tag_orientation_z = 0.0;
    double m_dApril_tag_orientation_w = 1.0;
    double m_dApril_tag_roll = 0.0;
    double m_dApril_tag_pitch = 0.0;
    double m_dApril_tag_yaw = 0.0;
    //Transform AR Tag Axis -> Robot Axis
	double m_transform_old_pose_x = 0.0;
    double m_transform_old_pose_y = 0.0;
    double m_transform_pose_x = 0.0;
    double m_transform_pose_y = 0.0;
    double m_dPositioning_Angle = 0.0;
    //Calc Odom to apriltag_TF
    double m_target_yaw = 0.0;
	double m_target_theta = 0.0;

}AR_TAG_POSE;
AR_TAG_POSE _pAR_tag_pose;

typedef struct LAND_MARK_POSE
{
    float init_position_x = 0.0;
    float init_position_y = 0.0;
    float init_position_z = 0.0;
    float init_orientation_z = 0.0;
    float init_orientation_w = 1.0;

}LAND_MARK_POSE;
LAND_MARK_POSE _pLandMarkPose;

typedef struct TF_POSE
{
	double poseTFx = 0.0;
	double poseTFy = 0.0;
	double poseTFz = 0.0;
	double poseTFqx = 0.0;
	double poseTFqy = 0.0;
	double poseTFqz = 0.0;
	double poseTFqw = 1.0;

	double pose_euler_roll = 0.0;
	double pose_euler_pitch = 0.0;
	double pose_euler_yaw = 0.0;

	double theta_deg = 0.0;
	
}TF_POSE;
TF_POSE _pTF_pose;


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
    //move result
    int  m_iMovebase_Result = 0; //Success: 3 | Aborted: 4 | Cancel: 1 | Unknown: 0
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

//dynamic parameter//
typedef struct DYNAMIC_PARAM
{
    double MAX_Linear_velocity = 1.0;
    double m_linear_vel = 0.0;
    double m_angular_vel = 0.0;
}DYNAMIC_PARAM;
DYNAMIC_PARAM _pDynamic_param;

typedef struct DOCKING_POSE
{
	double camera_TF_distance = -0.3955;
	double target_yaw = 0.0;
	double target_distance = 0.0;
	double before_TF_Pose = 0.0;
	
}DOCKING_POSE;
DOCKING_POSE _pDocking_pose;


std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}


class TETRA_SERVICE: public rclcpp::Node
{
public:
	TETRA_SERVICE() : Node("tetra_service")
	{

		// Initialize the TF2 buffer and listener
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

		//publish list/////////////////////////////////////////////////////////////////////////////////////
		//cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
		pose_reset_publisher = this->create_publisher<std_msgs::msg::Int32>("pose_reset", 10);
		initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
		//bumper_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("bumper_pointcloud", 10);

		//subscribe list////////////////////////////////////////////////////////////////////////////////////////////////////////////
		apriltag_subscriber = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
		 	"detections", 10, std::bind(&TETRA_SERVICE::ApriltagCallback, this, _1));

		sick_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
		  	"scan", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::SickCallback, this, _1));
		
		cygbot_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"scan2", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::CygbotCallback, this, _1));

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
			"joy", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::joyCallback, this, _1));

		particlecloud_subscriber = this->create_subscription<nav2_msgs::msg::ParticleCloud>(
			"particle_cloud", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::AMCL_PaticleCloud_Callback, this, _1)); //particle_cloud
			
		cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::cmd_vel_Callback, this, _1));

		initialpose_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::Initialpose_Callback, this, _1));

		map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			"map", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::map_Callback, this, _1));

		
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

		run_mapping_cmd_srv = create_service<interfaces::srv::RunMapping>(
        	"run_mapping_cmd", 
		std::bind(&TETRA_SERVICE::Mapping_Mode_Command, this, std::placeholders::_1, std::placeholders::_2));

		run_navigation_cmd_srv = create_service<interfaces::srv::RunNavigation>(
        	"run_navigation_cmd", 
		std::bind(&TETRA_SERVICE::Navigation_Mode_Command, this, std::placeholders::_1, std::placeholders::_2));

		get_information_cmd_srv = create_service<interfaces::srv::GetInformation>(
        	"get_information_cmd", 
		std::bind(&TETRA_SERVICE::Get_Information_Command, this, std::placeholders::_1, std::placeholders::_2));

		set_location_cmd_srv = create_service<interfaces::srv::SetLocation>(
        	"set_location_cmd", 
		std::bind(&TETRA_SERVICE::Set_Location_Command, this, std::placeholders::_1, std::placeholders::_2));

		goto_location_cmd_srv = create_service<interfaces::srv::GotoLocation>(
        	"goto_location_cmd", 
		std::bind(&TETRA_SERVICE::Goto_Location_Command, this, std::placeholders::_1, std::placeholders::_2));

		goto_location2_cmd_srv = create_service<interfaces::srv::GotoLocation2>(
        	"goto_location2_cmd", 
		std::bind(&TETRA_SERVICE::Goto_Location2_Command, this, std::placeholders::_1, std::placeholders::_2));

		goto_cancel_cmd_srv = create_service<interfaces::srv::GotoCancel>(
        	"goto_cancel_cmd", 
		std::bind(&TETRA_SERVICE::Goto_Cancel_Command, this, std::placeholders::_1, std::placeholders::_2));

		set_maxspeed_cmd_srv = create_service<interfaces::srv::SetMaxspeed>(
        	"set_maxspeed_cmd", 
		std::bind(&TETRA_SERVICE::Set_Maxspeed_Command, this, std::placeholders::_1, std::placeholders::_2));

		//Client list///////////////////////////////////////////////////////////////////////////////////////
		led_control_client = this->create_client<interfaces::srv::LedControl>("led_cmd");
		led_toggle_control_client = this->create_client<interfaces::srv::LedToggleControl>("led_toggle_cmd");
		toggle_on_client = this->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");
		imu_reset_client = this->create_client<interfaces::srv::ImuReset>("all_data_reset");
		//request_nomotion_update Service Client
  		request_nomotion_update_client = this->create_client<std_srvs::srv::Empty>("request_nomotion_update");
		// Clear Costmap Service Client
		clear_entire_global_costmap_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");
		clear_entire_local_costmap_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("local_costmap/clear_entirely_local_costmap");
		//robot_localization Service Client
		set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");
		
		// Set Max Speed parameter client
        set_speed_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "controller_server");

		//Action list///////////////////////////////////////////////////////////////////////////////////////
		nav_to_pose_action_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

		//PARAM
		this->declare_parameter("m_dHome_ID", rclcpp::PARAMETER_INTEGER);
		m_dHome_ID_param = this->get_parameter("m_dHome_ID");
		//Get Param
		m_dHome_ID = m_dHome_ID_param.as_int();
		printf("## Get_parameter(m_dHome_ID): %d \n", m_dHome_ID);
		
		//Timer
		timer_ = this->create_wall_timer(20ms, std::bind(&TETRA_SERVICE::DockingThread_function, this));
		//TF Calc Timer
		TF_timer_ = this->create_wall_timer(20ms, std::bind(&TETRA_SERVICE::TF_CALC_Timer, this));

	}

	////value//////////////////////////////////////////////////////////////////////////////
	rclcpp::Time current_time;
	//std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	geometry_msgs::msg::Twist cmd;
	std_msgs::msg::Int32 pose_reset_data;
	geometry_msgs::msg::PoseWithCovarianceStamped initPose;
	rclcpp::Parameter m_dHome_ID_param;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr TF_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

	//Publisher ////////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pose_reset_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;
	//Subscription ////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sick_scan_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr cygbot_scan_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bumper_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr emg_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tetra_battery_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr docking_status_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr conveyor_sensor_subscriber;   //Option
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr conveyor_movement_subscriber; //Option
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber; //joystick
	rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr particlecloud_subscriber; //AMCL particlecloud Subscribe
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber; //2D Pose Estimate
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
	//add... odometry
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

	////Action/////////////////////////////////////////////////////////////////////////////////
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
  	using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
	rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_action_client;

	//Subscribe Callback Function ////////////////////////////////////////////////////////////////////////////////////
	//cmd_vel
	void cmd_vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		_pDynamic_param.m_linear_vel  = msg->linear.x;
		_pDynamic_param.m_angular_vel = msg->angular.z;
	}

	//AMCL_
	void AMCL_PaticleCloud_Callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg) //nav2_msgs::msg::ParticleCloud
	{
		//printf("######### Call AMCL_PaticleCloud_Callback !!\n");
		auto m_request3 = std::make_shared<std_srvs::srv::Empty::Request>();

		int m_iParticleCloud_size = 0;
		m_iParticleCloud_size = msg->particles.size();

		//printf("[amcl] m_iParticleCloud_size = %d \n", m_iParticleCloud_size);
		if(m_iParticleCloud_size > 501 && _pDynamic_param.m_linear_vel == 0.0 && _pDynamic_param.m_angular_vel == 0.0 && _pFlag_Value.m_bFlag_Initialpose)
		{
			if(_pFlag_Value.m_bFlag_nomotion)
			{
				while(!request_nomotion_update_client->wait_for_service(1s))
				{
					if(!rclcpp::ok())
					{
						RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
						return;
					}
					RCLCPP_INFO_STREAM(this->get_logger(), "[AMCL_callback]service nomotion update not available, waiting again...");
				}
				request_nomotion_update_client->async_send_request(m_request3);
				printf("[amcl-1] m_iParticleCloud_size = %d \n", m_iParticleCloud_size);
			}   
		}
		else
		{
			_pFlag_Value.m_bFlag_Initialpose = false;
		}
		
	}

	void Initialpose_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		//printf("## [2D Pose Estimate] InitialposeCallback ## \n");
  		_pFlag_Value.m_bFlag_Initialpose = true;

	}

	void map_Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
	{
		//printf(" !!! map size: %d \n", msg->data.size());
		if(msg->data.size() > 0)
		{
			m_bActive_map = true;
		}
		else
		{
			m_bActive_map = false;
		}
	}


	/*
	void odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		_pOdometry.dOdom_position_x = msg->pose.pose.position.x;
		_pOdometry.dOdom_position_y = msg->pose.pose.position.y;
		_pOdometry.dOdom_position_z = msg->pose.pose.position.z;
		_pOdometry.dOdom_quaternion_x = msg->pose.pose.orientation.x;
		_pOdometry.dOdom_quaternion_y = msg->pose.pose.orientation.y;
		_pOdometry.dOdom_quaternion_z = msg->pose.pose.orientation.z;
		_pOdometry.dOdom_quaternion_w = msg->pose.pose.orientation.w;

		// Declaration of quaternion
		tf2::Quaternion q;
		q.setW(_pOdometry.dOdom_quaternion_w);
		q.setX(_pOdometry.dOdom_quaternion_x);
		q.setY(_pOdometry.dOdom_quaternion_y);
		q.setZ(_pOdometry.dOdom_quaternion_z);
		//<quaternion -> rotation Matrix
		tf2::Matrix3x3 m(q);
		// rotation Matrix -> quaternion
		m.getRotation(q);
		// rotation Matrix -> rpy
		m.getRPY(_pOdometry.dOdom_euler_roll, _pOdometry.dOdom_euler_pitch, _pOdometry.dOdom_euler_yaw);
		_pOdometry.dOdom_theta_deg = _pOdometry.dOdom_euler_yaw * (180.0/M_PI);
		//printf("_pOdometry.dOdom_theta_deg: %.3f \n", _pOdometry.dOdom_theta_deg);
	}
	*/

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

	//Apriltag Callback Function
	void ApriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) 
	{
		int size = msg->detections.size();
		//printf("size = %d \n", size);
		if(size != 0) //detection maker
		{
			for (const auto &detection : msg->detections) 
			{
				// Marker ID
				_pAR_tag_pose.m_iApril_tag_id = detection.id;
				//RCLCPP_INFO(this->get_logger(), "Marker ID: %d", marker_id);

			}
			string strTF_frame_name = _pAR_tag_pose.m_strFamily + to_string(_pAR_tag_pose.m_iApril_tag_id);
			//TF
			rclcpp::Time now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped transformStamped;
			transformStamped = tf2_buffer_->lookupTransform("camera", strTF_frame_name, tf2::TimePointZero);

			//position
			_pAR_tag_pose.m_dApril_tag_pose_x = transformStamped.transform.translation.x;
			_pAR_tag_pose.m_dApril_tag_pose_y = transformStamped.transform.translation.y;
			_pAR_tag_pose.m_dApril_tag_pose_z = transformStamped.transform.translation.z;
			//rotation//
			_pAR_tag_pose.m_dApril_tag_orientation_x = transformStamped.transform.rotation.x;
			_pAR_tag_pose.m_dApril_tag_orientation_y = transformStamped.transform.rotation.y;
			_pAR_tag_pose.m_dApril_tag_orientation_z = transformStamped.transform.rotation.z;
			_pAR_tag_pose.m_dApril_tag_orientation_w = transformStamped.transform.rotation.w;
			
			// Declaration of quaternion
			tf2::Quaternion q;
			q.setW(transformStamped.transform.rotation.w);
			q.setX(transformStamped.transform.rotation.x);
			q.setY(transformStamped.transform.rotation.y);
			q.setZ(transformStamped.transform.rotation.z);
			//<quaternion -> rotation Matrix
			tf2::Matrix3x3 m(q);
			// rotation Matrix -> quaternion
			m.getRotation(q);
			// rotation Matrix -> rpy
			m.getRPY(_pAR_tag_pose.m_dApril_tag_roll, _pAR_tag_pose.m_dApril_tag_pitch, _pAR_tag_pose.m_dApril_tag_yaw);
			//Transform Axis
			_pAR_tag_pose.m_dPositioning_Angle = _pAR_tag_pose.m_dApril_tag_pitch;
			_pAR_tag_pose.m_transform_old_pose_x = _pAR_tag_pose.m_dApril_tag_pose_z;
			_pAR_tag_pose.m_transform_old_pose_y = _pAR_tag_pose.m_dApril_tag_pose_x;

			if(_pAR_tag_pose.m_dPositioning_Angle > 0.0)
			{
				_pAR_tag_pose.m_transform_pose_x = _pAR_tag_pose.m_transform_old_pose_x;
				if(_pAR_tag_pose.m_transform_old_pose_y > 0)
				{
					_pAR_tag_pose.m_transform_pose_y = _pAR_tag_pose.m_transform_old_pose_y * -1.0; 
				}
				else
				{
					_pAR_tag_pose.m_transform_pose_y = _pAR_tag_pose.m_transform_old_pose_y; 
				}
			}
			else
			{
				_pAR_tag_pose.m_transform_pose_x = _pAR_tag_pose.m_transform_old_pose_x;
				if(_pAR_tag_pose.m_transform_old_pose_y < 0)
				{
					_pAR_tag_pose.m_transform_pose_y = _pAR_tag_pose.m_transform_old_pose_y * -1.0; 
				}
				else
				{
					_pAR_tag_pose.m_transform_pose_y = _pAR_tag_pose.m_transform_old_pose_y; 
				}
			}
			// printf("m_transform_old_pose_x: %.5f \n", _pAR_tag_pose.m_transform_old_pose_x);
			// printf("m_transform_old_pose_y: %.5f \n", _pAR_tag_pose.m_transform_old_pose_y);
			// printf("_pAR_tag_pose_pose_x:   %.5f \n", _pAR_tag_pose.m_transform_pose_x);
			// printf("_pAR_tag_pose_pose_y:   %.5f \n", _pAR_tag_pose.m_transform_pose_y);
			// printf(" m_dPositioning_Angle:  %.5f \n", _pAR_tag_pose.m_dPositioning_Angle);
			// printf("-----------------------------------------------\n");

		}
		else //No Maker
		{
			_pAR_tag_pose.m_iApril_tag_id = -1;
			_pAR_tag_pose.m_transform_pose_x = 0.0;
			_pAR_tag_pose.m_transform_pose_y = 0.0;
			_pAR_tag_pose.m_dApril_tag_pitch = 0.0;
			_pAR_tag_pose.m_dApril_tag_yaw = 0.0;
			_pAR_tag_pose.m_dPositioning_Angle = 0.0;

		}
	}

	//Sick Tim571 Lidar Callback
	void SickCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		int size = msg->ranges.size();
		//printf("sick_lidar_size: %d \n", size);

		//Center Check//
		int C_minIndex = 268;//270;
		int C_maxIndex = 536; //540;
		int C_closestIndex = -1;
		double C_minVal = 0.3;

		for (int i = C_minIndex; i < C_maxIndex; i++)
		{
			if ((msg->ranges[i] <= C_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
			{
				C_minVal = msg->ranges[i];
				C_closestIndex = i;
			}
		}
		//printf("C_closestIndex: %d || check: %f \n" , C_closestIndex, msg->ranges[C_closestIndex]);
		if(msg->ranges[C_closestIndex] > 0.1)
		{
			//printf("C_closestIndex: %d || check: %f \n" , C_closestIndex, msg->ranges[C_closestIndex]);
			_pFlag_Value.m_bFlag_Obstacle_Center = true;
		}
		else
			_pFlag_Value.m_bFlag_Obstacle_Center = false;

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
		if(_pAR_tag_pose.m_transform_pose_x <= 0.65 && _pAR_tag_pose.m_iApril_tag_id != -1) //650mm depart move
		{
			if(_pFlag_Value.m_bFlag_Obstacle_Center)
			{
				cmd.linear.x =  0.0; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
				bResult = false;
			}
			else
			{
				cmd.linear.x =  0.05; 
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

			//Nav Goal call///////////////////////////////////////////////////////
			Set_goal(_pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY, _pGoal_pose.goal_positionZ,
					 _pGoal_pose.goal_quarterX, _pGoal_pose.goal_quarterY, _pGoal_pose.goal_quarterZ, _pGoal_pose.goal_quarterW);

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
		//Reset robot localization reset call/////////////////////////////////////////////
		auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        // Fill the request with the desired pose
        request->pose.pose.pose.position.x = 0.0; // Set desired x position
        request->pose.pose.pose.position.y = 0.0; // Set desired y position
        request->pose.pose.pose.position.z = 0.0; // Set desired z position
        request->pose.pose.pose.orientation.x = 0.0; // Set desired orientation
        request->pose.pose.pose.orientation.y = 0.0; // Set desired orientation
        request->pose.pose.pose.orientation.z = 0.0; // Set desired orientation
        request->pose.pose.pose.orientation.w = 1.0; // Set desired orientation
        // Optionally set covariance
		request->pose.pose.covariance[0] = 0.25;
		request->pose.pose.covariance[6 * 1 + 1] = 0.25;
		request->pose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;
		// Call the service
        auto result_future = set_pose_client_->async_send_request(request);
		///////////////////////////////////////////////////////////////////////////////////

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
		string landmark_name = "marker_" + std::to_string(_pAR_tag_pose.m_iApril_tag_id);
		if (_pAR_tag_pose.m_iApril_tag_id == -1) 
		{
			bResult = false;
			return -1;
		}
		else 
		{
			//printf("#Docking Reset Marker_ID: %d \n", _pAR_tag_pose.m_iApril_tag_id);
			bResult = true;
		}

	
		m_strFilePathName = "/home/tetra/LANDMARK/" + landmark_name + ".txt";
		fp = fopen(m_strFilePathName.c_str(), "r");

		if(_pAR_tag_pose.m_iApril_tag_id > 0 && fp == NULL) 
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
			//printf("$$$$ init_position_x: %f , init_position_y: %f \n", _pLandMarkPose.init_position_x, _pLandMarkPose.init_position_y);
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
		if(_pRobot.HOME_id == _pAR_tag_pose.m_iApril_tag_id) // Same as Home ID... Loop 
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

		//Marker_Reset_Robot_Pose();

		_pFlag_Value.m_bFlag_Initialpose = true;
		LedToggleControl_Call(1,3,100,3,1);
    	ToggleOn_Call(63);
	}

	void TF_CALC_Timer()
	{
		try 
		{
			rclcpp::Time now = this->get_clock()->now();
            // Lookup the transform from the target frame to "footprint"
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf2_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

			//position
			_pTF_pose.poseTFx = transformStamped.transform.translation.x;
			_pTF_pose.poseTFy = transformStamped.transform.translation.y;
			_pTF_pose.poseTFz = transformStamped.transform.translation.z;
			//rotation//
			_pTF_pose.poseTFqx = transformStamped.transform.rotation.x;
			_pTF_pose.poseTFqy = transformStamped.transform.rotation.y;
			_pTF_pose.poseTFqz = transformStamped.transform.rotation.z;
			_pTF_pose.poseTFqw = transformStamped.transform.rotation.w;
			
			// Declaration of quaternion
			tf2::Quaternion q;
			q.setW(transformStamped.transform.rotation.w);
			q.setX(transformStamped.transform.rotation.x);
			q.setY(transformStamped.transform.rotation.y);
			q.setZ(transformStamped.transform.rotation.z);
			//<quaternion -> rotation Matrix
			tf2::Matrix3x3 m(q);
			// rotation Matrix -> quaternion
			m.getRotation(q);
			// rotation Matrix -> rpy
			m.getRPY(_pTF_pose.pose_euler_roll, _pTF_pose.pose_euler_pitch, _pTF_pose.pose_euler_yaw);
			_pTF_pose.theta_deg = _pTF_pose.pose_euler_yaw * (180.0/M_PI);

        } 
		catch (tf2::TransformException &ex) 
		{
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to footprint: %s", ex.what());
        }

	}

	bool SaveLocation(string str_location, 
    				double m_dPoseTFx, 	double m_dPoseTFy,  double m_dPoseTFz,
    				double m_dPoseTFqx, double m_dPoseTFqy, double m_dPoseTFqz, double m_dPoseTFqw)
	{
		bool bResult = false;

		string m_strFilePathName;
		m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";    
		fp = fopen(m_strFilePathName.c_str(), "w");
		if(fp == NULL)
		{ 
			printf("file is null \n");
			bResult = false;
		}
		else
		{
			fprintf(fp, "0,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",
					m_dPoseTFx, m_dPoseTFy, m_dPoseTFz, m_dPoseTFqx, m_dPoseTFqy, m_dPoseTFqz, m_dPoseTFqw);
			fclose(fp);
			bResult = true;
		}

		return bResult;
	}

	bool OpenLocationFile(string str_location)
	{
		bool bResult = false;
		string m_strFilePathName;

		if(str_location == "HOME")
		{
			_pGoal_pose.goal_positionX = _pHomePose.HOME_dPOSITION_X;
			_pGoal_pose.goal_positionY = _pHomePose.HOME_dPOSITION_Y;
			_pGoal_pose.goal_positionZ = _pHomePose.HOME_dPOSITION_Z;
			_pGoal_pose.goal_quarterX = _pHomePose.HOME_dQUATERNION_X;
			_pGoal_pose.goal_quarterY = _pHomePose.HOME_dQUATERNION_Y;
			_pGoal_pose.goal_quarterZ = _pHomePose.HOME_dQUATERNION_Z;
			_pGoal_pose.goal_quarterW = _pHomePose.HOME_dQUATERNION_W;
	
			bResult = true; 
		}
		else
		{
			m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";  
			fp = fopen(m_strFilePathName.c_str(), "r");  

			if(fp != NULL) //File Open Check
			{
				while(!feof(fp))
				{
					if(fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
					{
						char* ptr = strtok(Textbuffer, ",");
						int icnt = 0;

						while(ptr != NULL)
						{   
							ptr = strtok(NULL, ",");
							switch(icnt)
							{
								case 0:
									if(ptr != NULL)
									{
										_pGoal_pose.goal_positionX = atof(ptr);
									}
									break;
								case 1:
									_pGoal_pose.goal_positionY = atof(ptr);
									break;
								case 2:
									_pGoal_pose.goal_quarterX = atof(ptr);
									break;
								case 3:
									_pGoal_pose.goal_quarterY = atof(ptr);
									break;
								case 4:
									_pGoal_pose.goal_quarterZ = atof(ptr);  
									break;
								case 5:
									_pGoal_pose.goal_quarterW = atof(ptr);
									break;
							}
							icnt++;
						}
						bResult = true; 
					}
					else
					{
						bResult = false; 
					}
				}                
				fclose(fp);
			}
			else
			{
				RCLCPP_INFO(this->get_logger(),"File Open Fail Error!: %s", str_location.c_str());
				bResult = false;
			}
		}

		return bResult;
	}
	/* TEST FUNCTION*/
	bool Docking_Step1(int i_Docking_id)
	{
		bool bResult = false;
		float m_fdistance = 0.0;
		if(_pAR_tag_pose.m_iApril_tag_id == i_Docking_id)
		{	
			m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
			printf("pose_x: %.3f , pose_y: %.3f , tag_yaw: %.3f , distance: %.3f \n",_pAR_tag_pose.m_transform_pose_x,_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_dApril_tag_pitch, m_fdistance);
			printf("TF_pose_x: %.3f , TF_pose_y: %.3f , pose_yaw: %.3f \n", _pTF_pose.poseTFx, _pTF_pose.poseTFy, _pTF_pose.pose_euler_yaw);

			sleep(1);
			
			//CALC Target Yaw & Distance
			_pDocking_pose.target_yaw = 1.5708 - _pAR_tag_pose.m_dApril_tag_pitch; //_pAR_tag_pose.m_dApril_tag_yaw;
			printf("[calc] _pDocking_pose.target_yaw: %.3f \n", _pDocking_pose.target_yaw);
			_pDocking_pose.target_distance = _pAR_tag_pose.m_transform_pose_y;
			_pDocking_pose.before_TF_Pose = _pAR_tag_pose.m_transform_pose_y;
			
			m_iDocking_CommandMode = 3;
		}
		else
		{
			printf("[ChargingStation_tracking] No Marker, Rotation Movement !! \n");
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
		bResult = true;
		return bResult;

	}

	bool Docking_Step2()
	{
		bool bResult = false;
		/*
		1. position check!
		2. rotation move
		3. (pose_euler_yaw  == target pose yaw) check
		*/
		if(_pTF_pose.pose_euler_yaw <= (1.5708 + 0.004363323) && _pTF_pose.pose_euler_yaw >= (1.5708 - 0.004363323)) //+- 0.25deg
		{
			//finish Stop
			cmd.linear.x = 0.0; 
			cmd.angular.z = 0.0;
			cmd_vel_publisher->publish(cmd);
			printf("_pTF_pose.pose_euler_yaw: %.5f \n", _pTF_pose.pose_euler_yaw);
			m_iDocking_CommandMode = 4;
		}
		else
		{
			if(_pFlag_Value.m_bFlag_Obstacle_Center || _pFlag_Value.m_bFlag_Obstacle_cygbot)
			{
				//rotation move
				cmd.linear.x = 0.0; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
			}
			else
			{
				//Stop
				cmd.linear.x = 0.0; 
				cmd.angular.z = 0.174533;
				cmd_vel_publisher->publish(cmd);
				//printf("[Stop]: Obstacle Detections! (Front: %d | Rear: %d) \n", _pFlag_Value.m_bFlag_Obstacle_Center, _pFlag_Value.m_bFlag_Obstacle_cygbot);
			}


		}


		bResult = true;
		return bResult;

	}

	bool Docking_Step3(double dTaget_distance) //_pDocking_pose.target_distance
	{
		bool bResult = false;
		/*
		1. approach target distance move start
		2. approach target distance move finish check
		*/
		if(dTaget_distance > 0)
		{
			if(_pTF_pose.poseTFy >= (dTaget_distance - _pDocking_pose.before_TF_Pose))
			{
				//finish Stop
				cmd.linear.x = 0.0; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
				printf("[+]_pTF_pose.poseTFy: %.5f  | CALC: %.5f \n", _pTF_pose.poseTFy, dTaget_distance - _pDocking_pose.before_TF_Pose);
				m_iDocking_CommandMode = 5;
			}
			else
			{
				if(_pFlag_Value.m_bFlag_Obstacle_Center)
				{
					cmd.linear.x = 0.0; 
					cmd.angular.z = 0.0;
					cmd_vel_publisher->publish(cmd);
				}
				else
				{
					//approach move
					cmd.linear.x = 0.2; 
					cmd.angular.z = 0.0;
					cmd_vel_publisher->publish(cmd);
					//printf("[+]_pTF_pose.poseTFy: %.5f  | CALC: %.5f \n", _pTF_pose.poseTFy, dTaget_distance - _pDocking_pose.before_TF_Pose);
				}
			}
		}
		else
		{
			if(_pTF_pose.poseTFy <= (dTaget_distance - _pDocking_pose.before_TF_Pose))
			{
				//finish Stop
				cmd.linear.x = 0.0; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
				printf("[-]_pTF_pose.poseTFy: %.5f  | CALC: %.5f \n", _pTF_pose.poseTFy, dTaget_distance - _pDocking_pose.before_TF_Pose);
				m_iDocking_CommandMode = 5;
			}
			else
			{
				if(_pFlag_Value.m_bFlag_Obstacle_cygbot)
				{
					//approach move
					cmd.linear.x = 0.0; 
					cmd.angular.z = 0.0;
					cmd_vel_publisher->publish(cmd);
				}
				else
				{
					//approach move
					cmd.linear.x = -0.2; 
					cmd.angular.z = 0.0;
					cmd_vel_publisher->publish(cmd);
					//printf("[-]_pTF_pose.poseTFy: %.5f  | CALC: %.5f \n", _pTF_pose.poseTFy, dTaget_distance - _pDocking_pose.before_TF_Pose);
				}
			}

		}

		bResult = true;
		return bResult;

	}

	bool Docking_Step4()
	{
		bool bResult = false;

		if(_pTF_pose.pose_euler_yaw <= 0.0)
		{
			//finish Stop
			cmd.linear.x = 0.0; 
			cmd.angular.z = 0.0;
			cmd_vel_publisher->publish(cmd);
			//printf("_pTF_pose.pose_euler_yaw: %.5f \n", _pTF_pose.pose_euler_yaw);
			m_iDocking_CommandMode = 6;
		}
		else
		{
			if(_pFlag_Value.m_bFlag_Obstacle_Center || _pFlag_Value.m_bFlag_Obstacle_cygbot)
			{
				//Stop
				cmd.linear.x = 0.0; 
				cmd.angular.z = 0.0;
				cmd_vel_publisher->publish(cmd);
				//printf("[Stop]: Obstacle Detections! (Front: %d | Rear: %d) \n", _pFlag_Value.m_bFlag_Obstacle_Center, _pFlag_Value.m_bFlag_Obstacle_cygbot);
			}
			else
			{
				//rotation move
				cmd.linear.x = 0.0; 
				cmd.angular.z = -0.174533;
				cmd_vel_publisher->publish(cmd);
			}
			

		}
		bResult = true;
		return bResult;

	}

	bool Docking_Step5()
	{
		bool bResult = false;
		float m_fdistance = 0.0;

		m_fdistance = sqrt(_pAR_tag_pose.m_transform_old_pose_x * _pAR_tag_pose.m_transform_old_pose_x + _pAR_tag_pose.m_transform_old_pose_y * _pAR_tag_pose.m_transform_old_pose_y);
		//printf("master_distance: %.5f \n", m_fdistance);
		if(_pRobot.m_iCallback_Charging_status < 2)
		{
			cmd.linear.x = -1.0 * (m_fdistance /1.2) * 0.1; //max speed 0.1m/s
			if(cmd.linear.x > 1.0)
			{
				//Linear Over speed exit loop......
				cmd.linear.x =  0.0; 
				cmd.angular.z = 0.0;
				printf("[Linear Over speed]: follower is closing \n");
				return false;
			}
			
			cmd.angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_old_pose_y, _pAR_tag_pose.m_transform_old_pose_x) / 1.25;
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
			m_iDocking_CommandMode = 7;
		}
		
		
		bResult = true;
		return bResult;

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
					Docking_Step1(_pRobot.HOME_id);
					if(_pRobot.m_bFalg_DockingExit)
					{
						Docking_EXIT();
						m_iDocking_CommandMode = 0;
					}
					break;
				case 3:
					printf("Docking Loop 3... \n");
					Docking_Step2();
					if(_pFlag_Value.m_bfalg_DockingExit)
					{
					    Docking_EXIT();
					    m_iDocking_CommandMode = 0;
					}
					break;
				case 4:
					printf("Docking Loop 4... \n");
					Docking_Step3(_pDocking_pose.target_distance); 
					if(_pFlag_Value.m_bfalg_DockingExit)
					{
					    Docking_EXIT();
					    m_iDocking_CommandMode = 0;
					}
					break;
				case 5:
					printf("Docking Loop 5... \n");
					Docking_Step4();
					if(_pFlag_Value.m_bfalg_DockingExit)
					{
					    Docking_EXIT();
					    m_iDocking_CommandMode = 0;
					}
					break;
				case 6:
					printf("Docking Loop 6... \n");
					Docking_Step5();
					if(_pFlag_Value.m_bfalg_DockingExit)
					{
					    Docking_EXIT();
					    m_iDocking_CommandMode = 0;
					}
					break;
				case 7:
					printf("Docking End Loop 7... \n");
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
					//printf("Docking FAIL ! \n");
					LedToggleControl_Call(1, 10,100,10,1);
    				ToggleOn_Call(18);

					// m_iDocking_CommandMode = 119;
					m_iDocking_CommandMode = 0;
					break;
				case 10:
					//printf("case 10.. \n");
					Depart_Station2Move();
					break;
				/****************************************************************/
				case 30:
					
					break;
				case 31:
					
					break;
				case 32:
					//printf("Docking Loop 32... \n");
					
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

	//Server Callback Function ///////////////////////////////////////////////////////////////
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

		//call rosrun command//
		string str_command = "gnome-terminal -- /home/tetra/mapsave.sh ";
		string str_command2 = str_command + request->map_name.c_str();

		std::vector<char> writable1(str_command2.begin(), str_command2.end());
		writable1.push_back('\0');
		char* ptr1 = &writable1[0];
		int iResult = std::system(ptr1);

		printf("Save Map Name: %s \n", request->map_name.c_str());	
		
		response->command_result = true;
		return true;

	}

	bool Mapping_Mode_Command(
		const std::shared_ptr<interfaces::srv::RunMapping::Request> request, 
		const std::shared_ptr<interfaces::srv::RunMapping::Response> response)
	{
		bool bResult = false;

		string str_command = "gnome-terminal -- /home/tetra/mapping.sh ";
		std::vector<char> writable2(str_command.begin(), str_command.end());
		writable2.push_back('\0');
		char* ptr2 = &writable2[0];
		int iResult = std::system(ptr2);

		ex_ilaunchMode = 1;

		bResult = true;
		response->command_result = bResult;
		return bResult;

	}

	bool Navigation_Mode_Command(
		const std::shared_ptr<interfaces::srv::RunNavigation::Request> request, 
		const std::shared_ptr<interfaces::srv::RunNavigation::Response> response)
	{
		bool bResult = false;

		printf("Load Map Call _ %s \n", request->map_name.c_str());		
		string str_command = "gnome-terminal -- /home/tetra/navigation.sh ";
		string str_command2 = str_command + request->map_name.c_str();
		std::vector<char> writable3(str_command2.begin(), str_command2.end());
		writable3.push_back('\0');
		char* ptr3 = &writable3[0];
		int iResult = std::system(ptr3);

		ex_ilaunchMode = 2;

		bResult = true;
		response->command_result = bResult;
		return bResult;

	}

	bool Get_Information_Command(
		const std::shared_ptr<interfaces::srv::GetInformation::Request> request, 
		const std::shared_ptr<interfaces::srv::GetInformation::Response> response)
	{
		bool bResult = false;

		//Get Data
		response->battery = _pRobot.m_iCallback_Battery;
		response->error_code = _pRobot.m_iCallback_ErrorCode;
		response->emg = _pRobot.m_iCallback_EMG;
		response->bumper = _pRobot.m_iCallback_Bumper;
		response->charging = _pRobot.m_iCallback_Charging_status;
		response->running_mode = ex_ilaunchMode; //0:nomal, 1:mapping, 2:navigation
		/*
		---
		bool command_result
		int32 battery
		int32 error_code
		bool emg
		bool bumper
		bool charging
		*/

		bResult = true;
		response->command_result = bResult;
		return bResult;

	}

	bool Set_Location_Command(
		const std::shared_ptr<interfaces::srv::SetLocation::Request> request, 
		const std::shared_ptr<interfaces::srv::SetLocation::Response> response)
	{
		bool bResult = false;

		if(request->location == "HOME")
		{
			response->command_result = false;
			printf("[Location Save ERROR]: Cannot save with the name 'HOME'!!\n");
		}
		else
		{
			response->command_result = SaveLocation(request->location,
										_pTF_pose.poseTFx, _pTF_pose.poseTFy, _pTF_pose.poseTFz,
										_pTF_pose.poseTFqx, _pTF_pose.poseTFqy, _pTF_pose.poseTFqz, _pTF_pose.poseTFqw);
			
			response->pose_x = _pTF_pose.poseTFx;
			response->pose_y = _pTF_pose.poseTFy;
			response->pose_z = _pTF_pose.poseTFz;
			response->pose_qx = _pTF_pose.poseTFqx;
			response->pose_qy = _pTF_pose.poseTFqy;
			response->pose_qz = _pTF_pose.poseTFqz;
			response->pose_qw = _pTF_pose.poseTFqw;

		}
		/*
		string location
		---
		float64 pose_x
		float64 pose_y
		float64 pose_z
		float64 pose_qx
		float64 pose_qy
		float64 pose_qz
		float64 pose_qw
		bool command_result
		*/
		response->command_result = bResult;
		return bResult;
	}

	// Send Goal function//
    void Set_goal(double position_x, double position_y, double position_z, 
				   double orientation_x, double orientation_y,double  orientation_z, double orientation_w) 
	{
        //Wait Action server..
		while (!this->nav_to_pose_action_client->wait_for_action_server()) 
		{
			RCLCPP_INFO(get_logger(), "Waiting for action server...");
		}

		auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
		goal_msg.pose.header.frame_id = "map";
		goal_msg.pose.pose.position.x = position_x;
		goal_msg.pose.pose.position.y = position_y;
		goal_msg.pose.pose.position.z = position_z;
		goal_msg.pose.pose.orientation.x = orientation_x;
		goal_msg.pose.pose.orientation.y = orientation_y;
		goal_msg.pose.pose.orientation.z = orientation_z;
		goal_msg.pose.pose.orientation.w = orientation_w;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
		send_goal_options.feedback_callback = std::bind(&TETRA_SERVICE::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
		send_goal_options.result_callback = std::bind(&TETRA_SERVICE::resultCallback, this, std::placeholders::_1);

		//Send Goal//
		nav_to_pose_action_client->async_send_goal(goal_msg, send_goal_options);
    }

	bool Goto_Location_Command(
		const std::shared_ptr<interfaces::srv::GotoLocation::Request> request, 
		const std::shared_ptr<interfaces::srv::GotoLocation::Response> response)
	{
		bool bResult = false;

		//Clear Costmap Call
		Clear_Costmap();

		//LED Toggle Call
		LedToggleControl_Call(1,3,100,3,1);
		ToggleOn_Call(63); //White led
	
		//Location file load..
		bResult = OpenLocationFile(request->location);
		response->pose_x  = _pGoal_pose.goal_positionX;
		response->pose_y  = _pGoal_pose.goal_positionY;
		response->pose_z  = _pGoal_pose.goal_positionZ;
		response->pose_qx = _pGoal_pose.goal_quarterX;
		response->pose_qy = _pGoal_pose.goal_quarterY;
		response->pose_qz = _pGoal_pose.goal_quarterZ;
		response->pose_qw = _pGoal_pose.goal_quarterW;
		

		//Check robot status : noaml or docking?
		if(_pRobot.m_iCallback_Charging_status <= 1 && (_pAR_tag_pose.m_iApril_tag_id == -1 || _pAR_tag_pose.m_transform_pose_x <= 0.5)) //Nomal
    	{
			RCLCPP_INFO(get_logger(), "Goto Nomal Loop !!");

			//Nav Goal call///////////////////////////////////////////////////////
			Set_goal(_pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY, _pGoal_pose.goal_positionZ,
					 _pGoal_pose.goal_quarterX, _pGoal_pose.goal_quarterY, _pGoal_pose.goal_quarterZ, _pGoal_pose.goal_quarterW);

			bResult = true;
		}
		else
		{
			m_iDocking_CommandMode = 10; //Depart Move
        	bResult = true;

		}

		//Check Home Location
		if(request->location == "HOME")
		{
			_pFlag_Value.m_bflag_ComebackHome = true;
		}

		/*
		string location
		---
		float64 pose_x
		float64 pose_y
		float64 pose_z
		float64 pose_qx
		float64 pose_qy
		float64 pose_qz
		float64 pose_qw
		bool command_result
		*/
		bResult = true;
		response->command_result = bResult;
		return bResult;
	}

	bool Goto_Location2_Command(
		const std::shared_ptr<interfaces::srv::GotoLocation2::Request> request, 
		const std::shared_ptr<interfaces::srv::GotoLocation2::Response> response)
	{
		bool bResult = false;

		//Clear Costmap Call
		Clear_Costmap();

		//LED Toggle Call
		LedToggleControl_Call(1,3,100,3,1);
		ToggleOn_Call(63); //White led

		//Nav Goal call///////////////////////////////////////////////////////
		Set_goal(request->pose_x, request->pose_y, request->pose_z,
				request->pose_qx, request->pose_qy, request->pose_qz, request->pose_qw);

		/*
		float64 pose_x
		float64 pose_y
		float64 pose_z
		float64 pose_qx
		float64 pose_qy
		float64 pose_qz
		float64 pose_qw
		---
		bool command_result
		*/
		bResult = true;
		response->command_result = bResult;
		return true;
	}
	//Goto_Cancel_Command
	bool Goto_Cancel_Command(
		const std::shared_ptr<interfaces::srv::GotoCancel::Request> request, 
		const std::shared_ptr<interfaces::srv::GotoCancel::Response> response)
	{
		bool bResult = false;

		//Wait Action server..
		while (!this->nav_to_pose_action_client->wait_for_action_server()) 
		{
			RCLCPP_INFO(get_logger(), "Waiting for action server...");
		}

		//Goal Cancel Send//
		nav_to_pose_action_client->async_cancel_all_goals();
		
		/*
		---
		bool command_result
		*/
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	//Set MaxSpeed_Command
	bool Set_Maxspeed_Command(
		const std::shared_ptr<interfaces::srv::SetMaxspeed::Request> request, 
		const std::shared_ptr<interfaces::srv::SetMaxspeed::Response> response)
	{
		bool bResult = false;

		// Update the desired_linear_vel parameter
        auto parameters = std::vector<rclcpp::Parameter>();
        parameters.emplace_back(rclcpp::Parameter("FollowPath.desired_linear_vel", request->max_speed));
        set_speed_parameter_client_->set_parameters(parameters);
		
		/*
		float32 max_speed
		---
		bool command_result
		*/
		bResult = true;
		response->command_result = bResult;
		return true;
	}


	//Navigation to Pose Feedback Callback//
	void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
	{
		RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
	}

	//Navigation to Pose Result Callback//
	void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
	{
		switch (result.code) 
		{
			case rclcpp_action::ResultCode::SUCCEEDED:
				RCLCPP_INFO(get_logger(), "NavigateToPose: Success!!!");
				_pRobot.m_iMovebase_Result = 3;
				//LED Toggle Call
				LedToggleControl_Call(1,3,100,3,1);
				ToggleOn_Call(63); //White led
				sleep(1);

				if(_pFlag_Value.m_bflag_ComebackHome)
				{
					_pAR_tag_pose.m_iSelect_April_tag_id = m_dHome_ID; //0;
        			m_iDocking_CommandMode = 1;
					_pFlag_Value.m_bflag_ComebackHome = false;
				}
				break;
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_ERROR(get_logger(), "NavigateToPose: Goal was aborted");
				_pRobot.m_iMovebase_Result = 4;
				//LED Toggle Call
				LedToggleControl_Call(1,10,100,10,1);
				ToggleOn_Call(18); //Red led
				return;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_ERROR(get_logger(), "NavigateToPose: Goal was canceled");
				_pRobot.m_iMovebase_Result = 1;
				//LED Toggle Call
				LedToggleControl_Call(1,10,100,10,1);
				ToggleOn_Call(18); //Red led
				return;
			default:
				RCLCPP_ERROR(get_logger(), "NavigateToPose: Unknown result code");
				_pRobot.m_iMovebase_Result = 0;
				return;
		}
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

	bool Clear_Costmap() 
	{
		// Check if the service is available
		if (!clear_entire_global_costmap_client->wait_for_service(std::chrono::seconds(1))) 
		{
            RCLCPP_ERROR(this->get_logger(), "Clear global_Costmap Service not available.");
            return false;
        }

        if (!clear_entire_local_costmap_client->wait_for_service(std::chrono::seconds(1))) 
		{
            RCLCPP_ERROR(this->get_logger(), "Clear local_Costmap Service not available.");
            return false;
        }

        // Create and send the request
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
		auto request2 = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
		auto future = clear_entire_global_costmap_client->async_send_request(request);
        auto future2 = clear_entire_local_costmap_client->async_send_request(request2);
        
		return true;
    }

 
private:
	////Client/////////////////////////////////////////////////////////////////////////////////
	rclcpp::Client<interfaces::srv::LedControl>::SharedPtr led_control_client;
	rclcpp::Client<interfaces::srv::LedToggleControl>::SharedPtr led_toggle_control_client;
	rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client;
	rclcpp::Client<interfaces::srv::ImuReset>::SharedPtr imu_reset_client;
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr request_nomotion_update_client;
	rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_local_costmap_client;
	rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_global_costmap_client;
	//robot_localization Service Client
	rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
	std::shared_ptr<rclcpp::AsyncParametersClient> set_speed_parameter_client_;

	////Service/////////////////////////////////////////////////////////////////////////////////
	rclcpp::Service<interfaces::srv::DockingControl>::SharedPtr docking_cmd_srv;
	rclcpp::Service<interfaces::srv::DockingStop>::SharedPtr docking_stop_cmd_srv;
	rclcpp::Service<interfaces::srv::SaveMap>::SharedPtr save_map_cmd_srv;
	rclcpp::Service<interfaces::srv::RunMapping>::SharedPtr run_mapping_cmd_srv;
	rclcpp::Service<interfaces::srv::RunNavigation>::SharedPtr run_navigation_cmd_srv;
	rclcpp::Service<interfaces::srv::GetInformation>::SharedPtr get_information_cmd_srv;
	rclcpp::Service<interfaces::srv::SetLocation>::SharedPtr set_location_cmd_srv;
	rclcpp::Service<interfaces::srv::GotoLocation>::SharedPtr goto_location_cmd_srv;
	rclcpp::Service<interfaces::srv::GotoLocation2>::SharedPtr goto_location2_cmd_srv;
	rclcpp::Service<interfaces::srv::GotoCancel>::SharedPtr goto_cancel_cmd_srv;
	rclcpp::Service<interfaces::srv::SetMaxspeed>::SharedPtr set_maxspeed_cmd_srv;

};


//Main Loop//
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<TETRA_SERVICE>();

	//init
	node->current_time = node->now();
	rclcpp::WallRate loop_rate(30);

	//LED On
	node->LedToggleControl_Call(1,3,100,3,1);
    node->ToggleOn_Call(63);

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


	while (rclcpp::ok() && !stop_requested)
	{
		rclcpp::spin_some(node);
		
		//Pose & IMU & costmap Reset Loop///////////
		if(m_bActive_map)
		{
			if(m_iPose_reset_cnt > 500)
			{
				//printf("[SUM]m_iPose_reset_cnt: %d \n", m_iPose_reset_cnt);
				printf("RESET Robot POSE Call!!!\n");
				node->Clear_Costmap();
				node->Reset_Robot_Pose();
				m_iPose_reset_cnt = 0;
			}
			else
			{
				if(_pRobot.m_iCallback_Charging_status == 2 || _pRobot.m_iCallback_Charging_status == 3 || _pRobot.m_iCallback_Charging_status == 6 || _pRobot.m_iCallback_Charging_status == 7)
				{
					m_iPose_reset_cnt++;
					//printf("m_iPose_reset_cnt: %d \n"), m_iPose_reset_cnt;
				}
				else
				{
					m_iPose_reset_cnt = 0;
				}
			}
		}
		
	
		loop_rate.sleep();
    }

	rclcpp::shutdown();
    return 0;
}
