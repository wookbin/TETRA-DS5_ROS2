#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
//arucon message
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "visualization_msgs/msg/marker.hpp"

// //Custom Service//
#include "interfaces/srv/save_maker.hpp" //Maker Save


#include <chrono>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>

#define BUF_LEN 4096
using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

FILE *fp;
int status;
char Textbuffer[BUF_LEN];

//Tracked Pose
float m_fTracked_pose_x = 0.0;
float m_fTracked_pose_y = 0.0;
float m_fTracked_pose_z = 0.0;
float m_fTracked_orientation_x = 0.0;
float m_fTracked_orientation_y = 0.0;
float m_fTracked_orientation_z = 0.0;
float m_fTracked_orientation_w = 1.0;

typedef struct MARKER
{
	int count = 0;
    int id = -1;
    double pose_position_x = 0.0;
    double pose_position_y = 0.0;
    double pose_position_z = 0.0;
    double pose_orientation_x = 0.0;
    double pose_orientation_y = 0.0;
    double pose_orientation_z = 0.0;
    double pose_orientation_w = 0.0;
	//Quaternion to RPY
	double m_dEuler_roll = 0.0;
    double m_dEuler_yaw = 0.0;
    double m_dEuler_pitch = 0.0;
	//Transform Marker Axis to Robot Axis
	double m_dTransform_pose_x = 0.0;
    double m_dTransform_pose_y = 0.0;
    double m_dPositioning_Angle = 0.0;
	//Calc Odom to AR_Marker TF
    double m_dTarget_yaw = 0.0;

}MARKER;
MARKER _pMaker;

//LandMark Pose//
MARKER _pLandMark[255]; //max landmark count...
//Landmark info//
int m_iTotal_landmark_num = 0;


class TETRA_LANDMARK: public rclcpp::Node
{
public:
	TETRA_LANDMARK() : Node("tetra_landmark")
	{
		//tf2_ros
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		//publish list/////////////////////////////////////////////////////////////////////////////////////
		landmark_publisher = this->create_publisher<visualization_msgs::msg::Marker>("marker/node", 100);

		//subscribe list////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		aruco_markers_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
			"aruco_markers", 10, std::bind(&TETRA_LANDMARK::MakerCallback, this, _1));

		tracked_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"tracked_pose", 1, std::bind(&TETRA_LANDMARK::TrackedPosesCallback, this, _1));

		joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
			"joy", 10, std::bind(&TETRA_LANDMARK::joyCallback, this, _1));

		
		//Service list///////////////////////////////////////////////////////////////////////////////////////
		save_maker_cmd_srv = create_service<interfaces::srv::SaveMaker>(
        	"savemark_cmd", 
		std::bind(&TETRA_LANDMARK::SaveMaker_Command, this, std::placeholders::_1, std::placeholders::_2));

		//Client list///////////////////////////////////////////////////////////////////////////////////////

		
		//PARAM


	}

	////value//////////////////////////////////////////////////////////////////////////////
	rclcpp::Time current_time;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	visualization_msgs::msg::Marker node;

	//Publisher ////////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_publisher;

	//Subscription ////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_markers_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

	//Joystick _Logitech F710 Gamepad
	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
	{
		if(joy->buttons[5]) //RB button
		{	
			//Save Landmark Data Enable 
        	//m_bSave_Enable = true;

		}

	}

	//Subscribe Callback Function
	bool MakerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
	{
		_pMaker.count = msg->marker_ids.size();
		if(_pMaker.count < 1) return false;
		_pMaker.id = msg->marker_ids[_pMaker.count - 1];
		if(_pMaker.id == -1) return false;

		_pMaker.pose_position_x = msg->poses[_pMaker.count - 1].position.x;
		_pMaker.pose_position_y = msg->poses[_pMaker.count - 1].position.y;
		_pMaker.pose_position_z = msg->poses[_pMaker.count - 1].position.z;
		_pMaker.pose_orientation_x = msg->poses[_pMaker.count - 1].orientation.x;
		_pMaker.pose_orientation_y = msg->poses[_pMaker.count - 1].orientation.y;
		_pMaker.pose_orientation_z = msg->poses[_pMaker.count - 1].orientation.z;
		_pMaker.pose_orientation_w = msg->poses[_pMaker.count - 1].orientation.w;
		//Quaternion to RPY Axis transform... x/y/z/w -> w/x/y/z
		tf2::Quaternion q(
        _pMaker.pose_orientation_w,
        _pMaker.pose_orientation_x,
        _pMaker.pose_orientation_y,
        _pMaker.pose_orientation_z);
		tf2::Matrix3x3 m(q);
		m.getRPY(_pMaker.m_dEuler_roll, _pMaker.m_dEuler_pitch, _pMaker.m_dEuler_yaw);
		//Transform Axis
		_pMaker.m_dPositioning_Angle = _pMaker.m_dEuler_pitch * (180.0/M_PI);
		_pMaker.m_dTransform_pose_x  = _pMaker.pose_position_z;
		_pMaker.m_dTransform_pose_y  = _pMaker.pose_position_x;

		return true;
	}

	void TrackedPosesCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		m_fTracked_pose_x = msg->pose.position.x;
		m_fTracked_pose_y = msg->pose.position.y;
		m_fTracked_pose_z = msg->pose.position.z;
		m_fTracked_orientation_x = msg->pose.orientation.x;
		m_fTracked_orientation_y = msg->pose.orientation.y;
		m_fTracked_orientation_z = msg->pose.orientation.z;
		m_fTracked_orientation_w = msg->pose.orientation.w;
	}

	//Server Function ///////////////////////////////////////////////////////////////
	bool SaveMaker_Command(
		const std::shared_ptr<interfaces::srv::SaveMaker::Request> request, 
		const std::shared_ptr<interfaces::srv::SaveMaker::Response> response)
	{
		bool bResult = false;

		string m_strFilePathName;
		m_strFilePathName = "/home/tetra/LANDMARK/" + to_string(_pMaker.id) + ".txt";    
		fp = fopen(m_strFilePathName.c_str(), "w");
		if(fp == NULL)
		{ 
			printf("file is null \n");
			bResult = false;
		}
		else
		{
			if(_pMaker.id != -1)
			{
				fprintf(fp, "0,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
						_pMaker.id,
						_pMaker.pose_position_x,
						_pMaker.pose_position_y,
						_pMaker.pose_position_z,
						_pMaker.pose_position_x + (m_fTracked_pose_x - _pMaker.pose_position_x), 
						_pMaker.pose_position_y + (m_fTracked_pose_y - _pMaker.pose_position_y),
						_pMaker.pose_position_z,
						_pMaker.pose_orientation_x + _pMaker.pose_orientation_y, 
						-1.0 * (_pMaker.pose_orientation_z - _pMaker.pose_orientation_w)
						);

					
				fclose(fp);
			}
			printf("The marker is not visible! \n");
		}

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool OpenLocationFile(int ifile_cnt, string str_location)
	{
		bool bResult = false;
		string m_strFilePathName;
		string m_header_frame_id;
		string m_ns;
		m_strFilePathName = "/home/tetra/LANDMARK/" + str_location;  
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
							case 0: //mark_id
								_pLandMark[ifile_cnt].id = atoi(ptr);
								printf("mark_id: %d \n", _pLandMark[ifile_cnt].id);
								break;
							case 1: //pose_position_x
								_pLandMark[ifile_cnt].pose_position_x = atof(ptr);
								printf("pose_position_x: %f \n", _pLandMark[ifile_cnt].pose_position_x);
								break;
							case 2: //pose_position_y
								_pLandMark[ifile_cnt].pose_position_y = atof(ptr);
								printf("pose_position_y: %f \n", _pLandMark[ifile_cnt].pose_position_y);
								break;
							case 3: //pose_position_z
								_pLandMark[ifile_cnt].pose_position_z = atof(ptr);
								printf("pose_position_z: %f \n", _pLandMark[ifile_cnt].pose_position_z);
								break;
						}
						icnt++;
					}

					printf("************************** \n");
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
			printf("File Open Fail: %s \n", str_location.c_str());
			bResult = false;
		}

		
		return bResult;
	}

	int LocationList_Command()
	{
		int iResult = -1;

		//Load File List
		int m_icnt =0;
		DIR *dir;
		struct dirent *ent;
		dir = opendir ("/home/tetra/LANDMARK/"); // Landmark file path
		if (dir != NULL) 
		{
			//print all the files and directories within directory
			while ((ent = readdir (dir)) != NULL)
			{
				if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
				{ 
					continue; 
				}
				OpenLocationFile(m_icnt, ent->d_name);
				m_icnt++;
			}

			//Total List number
			iResult = m_icnt;
			closedir (dir);
		} 
		else 
		{
			//could not open directory
			perror ("");
			iResult = -1;
		}


		return iResult;
	}

private:
	////Service/////////////////////////////////////////////////////////////////////////////////
	rclcpp::Service<interfaces::srv::SaveMaker>::SharedPtr save_maker_cmd_srv;
	
};


//Main Loop//
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<TETRA_LANDMARK>();
	TETRA_LANDMARK tetra_landmark;
	//init
	tetra_landmark.current_time = node->now();
	rclcpp::WallRate loop_rate(30);

	printf("TETRA_LANDMARK Node Start! \n");
	m_iTotal_landmark_num = tetra_landmark.LocationList_Command();
    printf("m_iTotal_landmark_num: %d \n", m_iTotal_landmark_num);

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		//To do...
		//Draw Landmark//
        for(int i=0; i<m_iTotal_landmark_num; i++)
        {
            tetra_landmark.node.header.frame_id = "map";
            tetra_landmark.node.header.stamp = node->now();
            tetra_landmark.node.type = visualization_msgs::msg::Marker::SPHERE;
            tetra_landmark.node.ns = "";
            tetra_landmark.node.id = _pLandMark[i].id;
            tetra_landmark.node.action = visualization_msgs::msg::Marker::ADD; 
            tetra_landmark.node.pose.position.x = _pLandMark[i].pose_position_x;
            tetra_landmark.node.pose.position.y = _pLandMark[i].pose_position_y;
            tetra_landmark.node.pose.position.z = _pLandMark[i].pose_position_z;
            
            tetra_landmark.node.pose.orientation.x = 0.0;
            tetra_landmark.node.pose.orientation.y = 0.0; 
            tetra_landmark.node.pose.orientation.z = 0.0; 
            tetra_landmark.node.pose.orientation.w = 1.0; 

            // Points are green 
            tetra_landmark.node.color.a = 0.8; 
            tetra_landmark.node.color.r = 0.5;
            tetra_landmark.node.color.g = 1.0;
            tetra_landmark.node.color.b = 0.0;  
            tetra_landmark.node.scale.x = 0.3;
            tetra_landmark.node.scale.y = 0.3;
            tetra_landmark.node.scale.z = 0.3;
            //Publish
            tetra_landmark.landmark_publisher->publish(tetra_landmark.node);
        }

		
		loop_rate.sleep();
    }

	rclcpp::shutdown();
    return 0;
}
