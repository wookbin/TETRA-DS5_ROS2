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
//ar_tag message
#include "ar_track_alvar_msgs/msg/alvar_markers.hpp" //AR_TAG
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
#define CAMERA_Z_AXIS 0.181

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

FILE *fp;
int status;
char Textbuffer[BUF_LEN];

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
//LandMark Pose//
LANDMARK_POSE _pLandMark[255];

//AR_TAG Pose
int m_iAR_tag_id = -1;
float m_fAR_tag_pose_x = 0.0;
float m_fAR_tag_pose_y = 0.0;
float m_fAR_tag_pose_z = 0.0;
float m_fAR_tag_orientation_x = 0.0;
float m_fAR_tag_orientation_y = 0.0;
float m_fAR_tag_orientation_z = 0.0;
float m_fAR_tag_orientation_w = 0.0;
//Tracked Pose
float m_fTracked_pose_x = 0.0;
float m_fTracked_pose_y = 0.0;
float m_fTracked_pose_z = 0.0;
float m_fTracked_orientation_x = 0.0;
float m_fTracked_orientation_y = 0.0;
float m_fTracked_orientation_z = 0.0;
float m_fTracked_orientation_w = 0.0;
//calc pose
float m_fTracked_calc_pose_x = 0.0;
float m_fTracked_calc_pose_y = 0.0;
//Landmark info//
int m_iTotal_landmark_num = 0;
bool m_bSave_Enable = true;


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
		ar_tag_markers_subscriber = this->create_subscription<ar_track_alvar_msgs::msg::AlvarMarkers>(
			"ar_pose_marker", 100, std::bind(&TETRA_LANDMARK::AR_tagCallback, this, _1));

		map2marker_subscriber = this->create_subscription<ar_track_alvar_msgs::msg::AlvarMarkers>(
			"map_to_marker_pose", 100, std::bind(&TETRA_LANDMARK::Map2Mark_Callback, this, _1));

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
	ar_track_alvar_msgs::msg::AlvarMarkers ar_tag_msg;

	//Publisher ////////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_publisher;

	//Subscription ////////////////////////////////////////////////////////////////////////////////////
	rclcpp::Subscription<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr ar_tag_markers_subscriber;
	rclcpp::Subscription<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr map2marker_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

	//Joystick _Logitech F710 Gamepad
	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
	{
		if(joy->buttons[5]) //RB button
		{	
			//Save Landmark Data Enable 
        	m_bSave_Enable = true;
			//SaveMaker();

		}

	}

	//Subscribe Callback Function
	void AR_tagCallback(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr msg) 
	{
		if (!msg->markers.empty()) 
		{
			//AR_Tag data update...
			m_iAR_tag_id = msg->markers[0].id;
			m_fAR_tag_pose_x = msg->markers[0].pose.pose.position.x;
			m_fAR_tag_pose_y = msg->markers[0].pose.pose.position.y;
			m_fAR_tag_pose_z = msg->markers[0].pose.pose.position.z;
			m_fAR_tag_orientation_x = msg->markers[0].pose.pose.orientation.x;
			m_fAR_tag_orientation_y = msg->markers[0].pose.pose.orientation.y;
			m_fAR_tag_orientation_z = msg->markers[0].pose.pose.orientation.z;
			m_fAR_tag_orientation_w = msg->markers[0].pose.pose.orientation.w;
		}
		else
		{
			m_iAR_tag_id = -1;
		}
	}

	bool TrackedPosesCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msgPose)
	{
		bool bResult = false;

		m_fTracked_pose_x = msgPose->pose.position.x;
		m_fTracked_pose_y = msgPose->pose.position.y;
		m_fTracked_pose_z = msgPose->pose.position.z;
		m_fTracked_orientation_x = msgPose->pose.orientation.x;
		m_fTracked_orientation_y = msgPose->pose.orientation.y;
		m_fTracked_orientation_z = msgPose->pose.orientation.z;
		m_fTracked_orientation_w = msgPose->pose.orientation.w;

		bResult = true;
		return bResult;
	}

	bool SaveLandMark(LANDMARK_POSE p)
	{
		bool bResult = false;

		string m_strFilePathName;
		m_strFilePathName = "/home/tetra/LANDMARK/" + p.ns + ".txt";    
		fp = fopen(m_strFilePathName.c_str(), "w");
		if(fp == NULL)
		{ 
			printf("[Error]: file is null \n");
			bResult = false;
		}
		else
		{
			if(p.mark_id == m_iAR_tag_id)
			{
				fprintf(fp, "0,%s,%s,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
						p.header_frame_id.c_str(), 
						p.ns.c_str(), 
						p.mark_id,
						p.pose_position_x,
						p.pose_position_y,
						p.pose_position_z,
						p.pose_position_x + (m_fTracked_pose_x - p.pose_position_x), 
						p.pose_position_y + (m_fTracked_pose_y - p.pose_position_y),
						p.pose_position_z, 
						p.pose_orientation_x + p.pose_orientation_y, 
						-1.0 * (p.pose_orientation_z - p.pose_orientation_w)
						);

						
				fclose(fp);

				bResult = true;
			}
			else
				bResult = false;
		}

		return bResult;
	}

	//Server Function ///////////////////////////////////////////////////////////////
	bool SaveMaker_Command(
		const std::shared_ptr<interfaces::srv::SaveMaker::Request> request, 
		const std::shared_ptr<interfaces::srv::SaveMaker::Response> response)
	{
		bool bResult = false;

		//SaveMaker();
		m_bSave_Enable = true;

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
							case 0: //header_frame_id.c_str()
							if(ptr != NULL)
							{
								_pLandMark[ifile_cnt].header_frame_id = ptr;
								printf("header_frame_id: %s \n", _pLandMark[ifile_cnt].header_frame_id.c_str());  
							}
							break;
							case 1: //ns.c_str()
								_pLandMark[ifile_cnt].ns = ptr;
								printf("ns: %s \n", _pLandMark[ifile_cnt].ns.c_str());
							break;
							case 2: //mark_id
								_pLandMark[ifile_cnt].mark_id = atoi(ptr);
								printf("mark_id: %d \n", _pLandMark[ifile_cnt].mark_id);
							break;
							case 3: //pose_position_x
								_pLandMark[ifile_cnt].pose_position_x = atof(ptr);
								printf("pose_position_x: %f \n", _pLandMark[ifile_cnt].pose_position_x);
							break;
							case 4: //pose_position_y
								_pLandMark[ifile_cnt].pose_position_y = atof(ptr);
								printf("pose_position_y: %f \n", _pLandMark[ifile_cnt].pose_position_y);
							break;
							case 5: //pose_position_z
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

	//Map to AR_tag Transform
	void Map2Mark_Callback(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr msg) 
	{
		if (!msg->markers.empty()) 
		{
			//Green circle LandMark//////////////////////////////////////////
			_pLandMark[0].header_frame_id = node.header.frame_id = msg->markers[0].header.frame_id;
			node.header.stamp = rclcpp::Time();
			node.type = visualization_msgs::msg::Marker::SPHERE;
			_pLandMark[0].ns = node.ns = "marker_" + std::to_string(msg->markers[0].id);
			_pLandMark[0].mark_id = node.id = msg->markers[0].id;

			node.action = visualization_msgs::msg::Marker::ADD; 
			_pLandMark[0].pose_position_x = node.pose.position.x = msg->markers[0].pose.pose.position.x;
			_pLandMark[0].pose_position_y = node.pose.position.y = msg->markers[0].pose.pose.position.y;
			_pLandMark[0].pose_position_z = node.pose.position.z = msg->markers[0].pose.pose.position.z;
			_pLandMark[0].pose_orientation_x = node.pose.orientation.x = msg->markers[0].pose.pose.orientation.x;
			_pLandMark[0].pose_orientation_y = node.pose.orientation.y = msg->markers[0].pose.pose.orientation.y;
			_pLandMark[0].pose_orientation_z = node.pose.orientation.z = msg->markers[0].pose.pose.orientation.z;
			_pLandMark[0].pose_orientation_w = node.pose.orientation.w = msg->markers[0].pose.pose.orientation.w;

			// Points are green 
			node.color.a = 0.8; 
			node.color.r = 0.5;
			node.color.g = 1.0; 
			node.color.b = 0.0;  
			node.scale.x = 0.3;
			node.scale.y = 0.3;
			node.scale.z = 0.3;  

			if(m_bSave_Enable)
			{
				//Publish
				landmark_publisher->publish(node);
				//Save Marker data//
				SaveLandMark(_pLandMark[0]);
				m_bSave_Enable = false;
			}
		}
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
            tetra_landmark.node.ns = "marker_" + std::to_string(m_iAR_tag_id);
            tetra_landmark.node.id = _pLandMark[i].mark_id;
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
