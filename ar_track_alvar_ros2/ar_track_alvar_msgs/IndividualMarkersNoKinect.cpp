#include "std_msgs/msg/bool.hpp"
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include "ar_track_alvar_msgs/msg/alvar_marker.hpp"
#include "ar_track_alvar_msgs/msg/alvar_markers.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <ar_track_alvar/ParamsConfig.h>

using namespace alvar;
using namespace std;

bool init=true;
Camera *cam;
std::shared_ptr<rclcpp::Node> n_;
image_transport::Subscriber cam_sub_;
rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr arMarkerPub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rvizMarkerPub_;
ar_track_alvar_msgs::msg::AlvarMarkers arPoseMarkers_;
visualization_msgs::msg::Marker rvizMarker_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
MarkerDetector<MarkerData> marker_detector;

bool enableSwitched = false;
bool enabled = true;
double max_frequency;
double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic;
std::string cam_info_topic;
std::string output_frame;
int marker_resolution = 5; // default marker resolution
int marker_margin = 2; // default marker margin

void getCapCallback (const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
    //If we've already gotten the cam info, then go ahead
	if(cam->getCamInfo_){
		try{
			geometry_msgs::msg::TransformStamped CamToOutput;
        try{
					CamToOutput = tf_buffer_->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp);
        }
        catch (const tf2::TransformException & ex){
          RCLCPP_ERROR(n_->get_logger(), "%s",ex.what());
        }
            //Convert the image
            cv::Mat cv_image = cv_bridge::toCvShare(image_msg, "bgr8");

            marker_detector.Detect(&cv_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);
            arPoseMarkers_.markers.clear ();
			for (size_t i=0; i<marker_detector.markers->size(); i++)
			{
				//Get the pose relative to the camera
        		int id = (*(marker_detector.markers))[i].GetId();
				Pose p = (*(marker_detector.markers))[i].pose;
				double px = p.translation[0]/100.0;
				double py = p.translation[1]/100.0;
				double pz = p.translation[2]/100.0;
				double qx = p.quaternion[1];
				double qy = p.quaternion[2];
				double qz = p.quaternion[3];
				double qw = p.quaternion[0];

        tf2::Quaternion rotation(qx,qy,qz,qw);
        tf2::Vector3 origin(px,py,pz);
        tf2::Transform t (rotation, origin);
        tf2::Vector3 markerOrigin(0, 0, 0);
        tf2::Transform m(tf2::Quaternion::getIdentity (), markerOrigin);
        tf2::Transform markerPose = t * m; // marker pose in the camera frame
        tf2::Vector3 z_axis_cam = tf2::Transform(rotation, tf2::Vector3(0,0,0)) * tf2::Vector3(0,0,1);
//                ROS_INFO("%02i Z in cam frame: %f %f %f",id, z_axis_cam.x(), z_axis_cam.y(), z_axis_cam.z());
        /// as we can't see through markers, this one is false positive detection
        if (z_axis_cam.z() > 0)
        {
            continue;
        }

				//Publish the transform from the camera to the marker
				std::string markerFrame = "ar_marker_";
				std::stringstream out;
				out << id;
				std::string id_string = out.str();
				markerFrame += id_string;
        geometry_msgs::msg::TransformStamped camToMarker;
        camToMarker.header.stamp = image_msg->header.stamp;
        camToMarker.header.frame_id = image_msg->header.frame_id;
        camToMarker.child_frame_id = markerFrame.c_str();
        camToMarker.transform = tf2::toMsg(t);
        tf_broadcaster_->sendTransform(camToMarker);

				//Create the rviz visualization messages
				tf2::toMsg(markerPose, rvizMarker_.pose);
				rvizMarker_.header.frame_id = image_msg->header.frame_id;
				rvizMarker_.header.stamp = image_msg->header.stamp;
				rvizMarker_.id = id;

				rvizMarker_.scale.x = 1.0 * marker_size/100.0;
				rvizMarker_.scale.y = 1.0 * marker_size/100.0;
				rvizMarker_.scale.z = 0.2 * marker_size/100.0;
				rvizMarker_.ns = "basic_shapes";
				rvizMarker_.type = visualization_msgs::msg::Marker::CUBE;
				rvizMarker_.action = visualization_msgs::msg::Marker::ADD;
				switch (id)
				{
				  case 0:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 1.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 1:
				    rvizMarker_.color.r = 1.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 0.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 2:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 1.0f;
				    rvizMarker_.color.b = 0.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 3:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 4:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.0;
				    rvizMarker_.color.a = 1.0;
				    break;
				  default:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				}
				rvizMarker_.lifetime = rclcpp::Duration(1.0);
				rvizMarkerPub_->publish(rvizMarker_);

				//Get the pose of the tag in the camera frame, then the output frame (usually torso)
        tf2::Transform CamToOutputTf;
        tf2::fromMsg(CamToOutput.transform, CamToOutputTf);
				tf2::Transform tagPoseOutput = CamToOutputTf * markerPose;

				//Create the pose marker messages
				ar_track_alvar_msgs::msg::AlvarMarker ar_pose_marker;
        geometry_msgs::msg::Pose marker_pose;
				tf2::toMsg(tagPoseOutput, marker_pose);
        ar_pose_marker.pose.pose = marker_pose;
        ar_pose_marker.header.frame_id = output_frame;
        ar_pose_marker.header.stamp = image_msg->header.stamp;
        ar_pose_marker.id = id;
        arPoseMarkers_.markers.push_back (ar_pose_marker);
			}
			arMarkerPub_->publish (arPoseMarkers_);
		}
        catch (cv_bridge::Exception& e){
      		RCLCPP_ERROR(n_->get_logger(), "Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    	}
	}
}
//TODO get_parameter change
// void configCallback(ar_track_alvar::ParamsConfig &config, uint32_t level)
// {
//   RCLCPP_INFO(n_->get_logger(), "AR tracker reconfigured: %s %.2f %.2f %.2f %.2f", config.enabled ? "ENABLED" : "DISABLED",
//            config.max_frequency, config.marker_size, config.max_new_marker_error, config.max_track_error);

//   enableSwitched = enabled != config.enabled;

//   enabled = config.enabled;
//   max_frequency = config.max_frequency;
//   marker_size = config.marker_size;
//   max_new_marker_error = config.max_new_marker_error;
//   max_track_error = config.max_track_error;
// }

void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    enableSwitched = enabled != msg->data;
    enabled = msg->data;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  n_ = rclcpp::Node::make_shared("marker_detect");

  if(argc > 1) {
    RCLCPP_WARN(n_->get_logger(), "Command line arguments are deprecated. Consider using ROS parameters and remappings.");

    if(argc < 7){
      std::cout << std::endl;
      cout << "Not enough arguments provided." << endl;
      cout << "Usage: ./individualMarkersNoKinect <marker size in cm> <max new marker error> <max track error> "
           << "<cam image topic> <cam info topic> <output frame> [ <max frequency> <marker_resolution> <marker_margin>]";
      std::cout << std::endl;
      return 0;
    }

    // Get params from command line
    marker_size = atof(argv[1]);
    max_new_marker_error = atof(argv[2]);
    max_track_error = atof(argv[3]);
    cam_image_topic = argv[4];
    cam_info_topic = argv[5];
    output_frame = argv[6];

    if (argc > 7) {
      max_frequency = atof(argv[7]);
      n_->declare_parameter("max_frequency", max_frequency);
      max_frequency = n_->get_parameter("max_frequency").as_double();
    }
    if (argc > 8)
      marker_resolution = atoi(argv[8]);
    if (argc > 9)
      marker_margin = atoi(argv[9]);

  } else {
    // Get params from ros param server.
    n_->declare_parameter("marker_size", 10.0);
    marker_size = n_->get_parameter("marker_size").as_double();
    n_->declare_parameter("max_new_marker_error", 0.05);
    max_new_marker_error = n_->get_parameter("max_new_marker_error").as_double();
    n_->declare_parameter("max_track_error", 0.05);
    max_track_error = n_->get_parameter("max_track_error").as_double();
    n_->declare_parameter("max_frequency", 30.0);
    max_frequency = n_->get_parameter("max_frequency").as_double();
    n_->declare_parameter("marker_resolution", 5);
    marker_resolution = n_->get_parameter("marker_resolution").as_int();
    n_->declare_parameter("marker_margin", 2);
    marker_margin = n_->get_parameter("marker_margin").as_int();
    n_->declare_parameter("output_frame", "");
    output_frame = n_->get_parameter("output_frame").as_string();
    if (output_frame == "") {
      RCLCPP_ERROR(n_->get_logger(), "Param 'output_frame' has to be set.");
      exit(EXIT_FAILURE);
    }

    // Camera input topics. Use remapping to map to your camera topics.
    cam_image_topic = "camera/image_raw";
    cam_info_topic = "camera/camera_info";
  }

	marker_detector.SetMarkerSize(marker_size, marker_resolution, marker_margin);

	cam = new Camera(n_, cam_info_topic);
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(n_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*n_);
	arMarkerPub_ = n_->create_publisher<ar_track_alvar_msgs::msg::AlvarMarkers > ("ar_pose_marker", 0);
	rvizMarkerPub_ = n_->create_publisher< visualization_msgs::msg::Marker > ("visualization_marker", 0);

  // Prepare dynamic reconfiguration
  // dynamic_reconfigure::Server < ar_track_alvar::ParamsConfig > server;
  // dynamic_reconfigure::Server<ar_track_alvar::ParamsConfig>::CallbackType f;

  // f = boost::bind(&configCallback, _1, _2);
  // server.setCallback(f);

	//Give tf a chance to catch up before the camera callback starts asking for transforms
  // It will also reconfigure parameters for the first time, setting the default values
	rclcpp::Rate(1.0).sleep();
	rclcpp::spin_some(n_);

	image_transport::ImageTransport it_(n_);

  // Run at the configured rate, discarding pointcloud msgs if necessary
  rclcpp::Rate rate(max_frequency);

  /// Subscriber for enable-topic so that a user can turn off the detection if it is not used without
  /// having to use the reconfigure where he has to know all parameters
  auto enable_sub_ = n_->create_subscription<std_msgs::msg::Bool>("enable_detection", 1, &enableCallback);

  enableSwitched = true;
  while (rclcpp::ok())
  {
    rclcpp::spin_some(n_);
    rate.sleep();

    if (std::abs((rclcpp::Duration(rate.period()) - rclcpp::Duration(1.0 / max_frequency)).seconds()) > 0.001)
    {
      // Change rate dynamically; if must be above 0, as 0 will provoke a segfault on next spinOnce
      RCLCPP_DEBUG(n_->get_logger(), "Changing frequency from %.2f to %.2f", 1.0 / rclcpp::Duration(rate.period()).seconds(), max_frequency);
    }

    if (enableSwitched)
    {
      // Enable/disable switch: subscribe/unsubscribe to make use of pointcloud processing nodelet
      // lazy publishing policy; in CPU-scarce computer as TurtleBot's laptop this is a huge saving
      if (enabled)
        cam_sub_ = it_.subscribe(cam_image_topic, 1, getCapCallback);
      else
        cam_sub_.shutdown();
      enableSwitched = false;
    }
  }

    return 0;
}
