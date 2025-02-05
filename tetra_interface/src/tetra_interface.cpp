#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp" //Ultrasonic option//
//#include "pcl/point_cloud.hpp"
//#include "pcl/point_types.hpp"
#include "pcl_conversions/pcl_conversions.h"
//Custom Message//
#include "interfaces/msg/gpio.hpp"
// //Custom Service//
#include "interfaces/srv/led_control.hpp"
#include "interfaces/srv/led_toggle_control.hpp"
#include "interfaces/srv/toggle_on.hpp"
#include "interfaces/srv/integrallog.hpp"
#include "interfaces/srv/power_set_outport.hpp" 
#include "interfaces/srv/power_get_io_status.hpp"
#include "interfaces/srv/power_set_enable.hpp"
#include "interfaces/srv/power_set_single_outport.hpp"
#include "interfaces/srv/power_set_single_enable.hpp"
#include "interfaces/srv/power_wheel_enable.hpp"
#include "interfaces/srv/power_parameter_read.hpp"
#include "interfaces/srv/power_parameter_write.hpp"
#include "interfaces/srv/power_data_read.hpp"
#include "interfaces/srv/power_adc_read.hpp"
#include "interfaces/srv/power_version_read.hpp"
#include "interfaces/srv/power_sonar_read.hpp" //Ultrasonic option//
#include "interfaces/srv/power_sonar_cmd.hpp"  //Ultrasonic option//
#include "interfaces/srv/conveyor_parameter_read.hpp"  //Conveyor option//
#include "interfaces/srv/conveyor_auto_movement.hpp"   //Conveyor option//
#include "interfaces/srv/conveyor_manual_movement.hpp" //Conveyor option//
#include "interfaces/srv/conveyor_parameter_write.hpp" //Conveyor option//
#include "interfaces/srv/conveyor_data_read.hpp"	   //Conveyor option//


extern "C"
{
	#include "power_module.h"
	#include "dssp_rs232_power_module.h"
}

#include <chrono>
#include <thread> //thread add...
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <atomic>
using namespace std;

#define Ultrasonic_MIN_range	0.04
#define Ultrasonic_MAX_range	0.5
#define BUF_LEN 4096

using namespace std::chrono_literals;

//Power Board Parameter data
int m_iParam0 = 0;
int m_iParam1 = 0;
int m_iParam2 = 0;
int m_iParam3 = 0;
int m_iParam4 = 0;
int m_iParam5 = 0;
int m_iParam6 = 0;
int m_iParam7 = 0;
int m_iParam8 = 0;
int m_iParam9 = 0;
int m_iParam10 = 0;
int m_iParam11 = 0;
int m_iParam12 = 0;
int m_iParam13 = 0;
int m_iParam14 = 0;
int m_iParam15 = 0;
int m_iParam16 = 0;
int m_iParam17 = 0;
int m_iParam18 = 0;
int m_iParam19 = 0;
int m_iParam20 = 0;
int m_iParam21 = 0;
int m_iParam22 = 0;
int m_iParam23 = 0;
int m_iParam24 = 0;
int m_iParam25 = 0;
int m_iParam26 = 0;
int m_iParam27 = 0;
int m_iParam28 = 0;
int m_iParam29 = 0;
int m_iParam30 = 0;
int m_iParam31 = 0;
//serial
int m_iFlag_PowerCheck_cnt = 0;
int com_port = 0;
char port[16] = {0,};
//battery data
int m_ibattery_Level = 0;
double m_dbattery = 0.0;
double m_dVoltage = 0.0;
double m_dCurrent = 0.0;
int m_imode_status = 0;
int m_iPowerCheck = 0;
int m_iPowerCheckCount = 0;
//Ultrasonic data//
bool m_bUltrasonic_option = false; // true or false
double m_dUltrasonic[8] = {0.0, };  //Max Ultrasonic: 8ea (TETRA-DS used 4ea _ Option !)
// sensor_msgs::msg::Range range_msg1; //Ultrasonic_1
// sensor_msgs::msg::Range range_msg2; //Ultrasonic_2
// sensor_msgs::msg::Range range_msg3; //Ultrasonic_3
// sensor_msgs::msg::Range range_msg4; //Ultrasonic_4
// double time_offset_in_seconds = 0.0;
//GPIO data//
interfaces::msg::Gpio gpio_msg;
int m_iOutput[8] = {0,};
int m_iInput[8] = {0,};
//Conveyor  & sensor status _ Option//
bool m_bConveyor_option = false; //true or false
int  m_dConveyor_sensor = 0; 
int  m_iConveyor_movement = 0;
//File read & write
FILE *fp;
int  status;
char Textbuffer[BUF_LEN];
char strPath[BUF_LEN];

std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}

void Error_Log_write(string log_data)
{
	time_t timer; 
	struct tm* t; 
	timer = time(NULL); 
	t = localtime(&timer); 

	string m_strFilePathName;
	string m_colon = ":";
	string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
	string m_temp1 = m_time_stemp + "_error";		

	m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
	fp = fopen(m_strFilePathName.c_str(), "w");
	if(fp == NULL) 
		printf("file is null \n");


	fprintf(fp, "[Error]: %s \n", log_data.c_str());
	fclose(fp);
}


class TETRA_INTERFACE: public rclcpp::Node
{
public:
	TETRA_INTERFACE() : Node("tetra_interface")
	{
		//Publish list////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		power_status_publisher = this->create_publisher<std_msgs::msg::Int32>("power_status", 1);
		servo_publisher = this->create_publisher<std_msgs::msg::Int32>("servo_on", 1);
		tetra_battery_publisher = this->create_publisher<std_msgs::msg::Int32>("tetra_battery", 1);
		battery_voltage_publisher = this->create_publisher<std_msgs::msg::Float64>("battery_voltage", 1);
		battery_current_publisher = this->create_publisher<std_msgs::msg::Float64>("battery_current", 1);
		docking_status_publisher = this->create_publisher<std_msgs::msg::Int32>("docking_status", 1);
		gpio_status_publisher = this->create_publisher<interfaces::msg::Gpio>("gpio_status", 10); //interfaces message_GPIO
		//Ultrasonic_Option//
		// Ultrasonic1_publisher = this->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_D_L", 10);
		// Ultrasonic2_publisher = this->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_R_L", 10);
		// Ultrasonic3_publisher = this->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_R_R", 10);
		// Ultrasonic4_publisher = this->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_D_R", 10);
		//Conveyor_Option//
		conveyor_sensor_publisher = this->create_publisher<std_msgs::msg::Int32>("conveyor_sensor", 1);
		conveyor_movement_publisher = this->create_publisher<std_msgs::msg::Int32>("conveyor_movement", 1);

		//Subscribe list//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		//Service list////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		led_control_srv = create_service<interfaces::srv::LedControl>(
        	"led_cmd", 
		std::bind(&TETRA_INTERFACE::LED_Control_Command, this, std::placeholders::_1, std::placeholders::_2));

		led_toggle_control_srv = create_service<interfaces::srv::LedToggleControl>(
        	"led_toggle_cmd", 
		std::bind(&TETRA_INTERFACE::LED_Toggle_Command, this, std::placeholders::_1, std::placeholders::_2));

		toggle_on_srv = create_service<interfaces::srv::ToggleOn>(
        	"turn_on_cmd", 
		std::bind(&TETRA_INTERFACE::Toggle_On_Command, this, std::placeholders::_1, std::placeholders::_2));

		integrallog_srv = create_service<interfaces::srv::Integrallog>(
        	"log_cmd", 
		std::bind(&TETRA_INTERFACE::Integrallog_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_set_outport_srv = create_service<interfaces::srv::PowerSetOutport>(
        	"Power_outport_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Set_Outport_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_set_single_outport_srv = create_service<interfaces::srv::PowerSetSingleOutport>(
        	"Power_single_outport_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Set_Single_Outport_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_get_io_status_srv = create_service<interfaces::srv::PowerGetIoStatus>(
        	"Power_io_status_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Get_IO_Status_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_set_enable_srv = create_service<interfaces::srv::PowerSetEnable>(
        	"Power_enable_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Set_Enable_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_set_single_enable_srv = create_service<interfaces::srv::PowerSetSingleEnable>(
        	"Power_single_enable_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Set_Single_Enable_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_wheel_enable_srv = create_service<interfaces::srv::PowerWheelEnable>(
        	"Power_wheel_enable_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Wheel_Enable_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_parameter_read_srv = create_service<interfaces::srv::PowerParameterRead>(
        	"Power_parameter_read_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Paramter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_parameter_write_srv = create_service<interfaces::srv::PowerParameterWrite>(
        	"Power_parameter_write_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Paramter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_data_read_srv = create_service<interfaces::srv::PowerDataRead>(
        	"Power_data_read_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Data_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_adc_read_srv = create_service<interfaces::srv::PowerAdcRead>(
        	"Power_adc_read_cmd", 
		std::bind(&TETRA_INTERFACE::Power_ADC_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_version_read_srv = create_service<interfaces::srv::PowerVersionRead>(
        	"Power_version_read_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Version_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		conveyor_auto_movement_srv = create_service<interfaces::srv::ConveyorAutoMovement>(
        	"Conveyor_auto_move_cmd", 
		std::bind(&TETRA_INTERFACE::Conveyor_Auto_Movement_Command, this, std::placeholders::_1, std::placeholders::_2));

		conveyor_manual_movement_srv = create_service<interfaces::srv::ConveyorManualMovement>(
        	"Conveyor_manual_move_cmd", 
		std::bind(&TETRA_INTERFACE::Conveyor_Manual_Movement_Command, this, std::placeholders::_1, std::placeholders::_2));

		conveyor_parameter_read_srv = create_service<interfaces::srv::ConveyorParameterRead>(
        	"Conveyor_parameter_read_cmd", 
		std::bind(&TETRA_INTERFACE::Conveyor_Parameter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		conveyor_parameter_write_srv = create_service<interfaces::srv::ConveyorParameterWrite>(
        	"Conveyor_parameter_write_cmd", 
		std::bind(&TETRA_INTERFACE::Conveyor_Parameter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));

		conveyor_data_read_srv = create_service<interfaces::srv::ConveyorDataRead>(
        	"Conveyor_data_read_cmd", 
		std::bind(&TETRA_INTERFACE::Conveyor_Data_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_sonar_read_srv = create_service<interfaces::srv::PowerSonarRead>(
        	"Power_sonar_read_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Sonar_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_sonar_cmd_srv = create_service<interfaces::srv::PowerSonarCmd>(
        	"Power_sonar_start_cmd", 
		std::bind(&TETRA_INTERFACE::Power_Sonar_Command, this, std::placeholders::_1, std::placeholders::_2));




		//Get ROS PARAM//
		//Conveyor_option_param
		this->declare_parameter("m_bConveyor_option", rclcpp::PARAMETER_BOOL);
		m_bConveyor_option_param = this->get_parameter("m_bConveyor_option");
		m_bConveyor_option = m_bConveyor_option_param.as_bool();
	
		// //Ultrasonic_option_param
		// this->declare_parameter("m_bUltrasonic_option");
		// m_bUltrasonic_option_param = this->get_parameter("m_bUltrasonic_option");
		// m_bUltrasonic_option = m_bUltrasonic_option_param.as_bool();


	}

	////value//////////////////////////////////////////////////////////////////////////////
	rclcpp::Time current_time, last_time;
	rclcpp::Parameter m_bConveyor_option_param;
	// rclcpp::Parameter m_bUltrasonic_option_param;

	//Publisher
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr power_status_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tetra_battery_publisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_publisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_current_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr docking_status_publisher;
	rclcpp::Publisher<interfaces::msg::Gpio>::SharedPtr gpio_status_publisher; //interfaces message_GPIO
	//Publisher _ Ultrasonic_Option//
	// rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic1_publisher;
	// rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic2_publisher;
	// rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic3_publisher;
	// rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic4_publisher;
	//Publisher _ Ultrasonic_Option --> Ultrasonic Range To PointCloud2 data pub//
	//To do...

	//Publisher _ Conveyor_Option//
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr conveyor_sensor_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr conveyor_movement_publisher;
	//Subscription


private:
	////value/////////////////////////////////////////////////////////////////////////////////
	rclcpp::Service<interfaces::srv::LedControl>::SharedPtr led_control_srv;
	rclcpp::Service<interfaces::srv::LedToggleControl>::SharedPtr led_toggle_control_srv;
	rclcpp::Service<interfaces::srv::ToggleOn>::SharedPtr toggle_on_srv;
	rclcpp::Service<interfaces::srv::Integrallog>::SharedPtr integrallog_srv;
	rclcpp::Service<interfaces::srv::PowerSetOutport>::SharedPtr power_set_outport_srv;
	rclcpp::Service<interfaces::srv::PowerSetSingleOutport>::SharedPtr power_set_single_outport_srv;
	rclcpp::Service<interfaces::srv::PowerGetIoStatus>::SharedPtr power_get_io_status_srv;
	rclcpp::Service<interfaces::srv::PowerSetEnable>::SharedPtr power_set_enable_srv;
	rclcpp::Service<interfaces::srv::PowerSetSingleEnable>::SharedPtr power_set_single_enable_srv;
	rclcpp::Service<interfaces::srv::PowerWheelEnable>::SharedPtr power_wheel_enable_srv;
	rclcpp::Service<interfaces::srv::PowerParameterRead>::SharedPtr power_parameter_read_srv;
	rclcpp::Service<interfaces::srv::PowerParameterWrite>::SharedPtr power_parameter_write_srv;
	rclcpp::Service<interfaces::srv::PowerDataRead>::SharedPtr power_data_read_srv;
	rclcpp::Service<interfaces::srv::PowerAdcRead>::SharedPtr power_adc_read_srv;
	rclcpp::Service<interfaces::srv::PowerVersionRead>::SharedPtr power_version_read_srv;
	rclcpp::Service<interfaces::srv::ConveyorAutoMovement>::SharedPtr conveyor_auto_movement_srv;   //Conveyor option//
	rclcpp::Service<interfaces::srv::ConveyorManualMovement>::SharedPtr conveyor_manual_movement_srv; //Conveyor option//
	rclcpp::Service<interfaces::srv::ConveyorParameterRead>::SharedPtr conveyor_parameter_read_srv;  //Conveyor option//
	rclcpp::Service<interfaces::srv::ConveyorParameterWrite>::SharedPtr conveyor_parameter_write_srv; //Conveyor option//
	rclcpp::Service<interfaces::srv::ConveyorDataRead>::SharedPtr conveyor_data_read_srv; //Conveyor option//
	rclcpp::Service<interfaces::srv::PowerSonarRead>::SharedPtr power_sonar_read_srv; //Ultrasonic option//
	rclcpp::Service<interfaces::srv::PowerSonarCmd>::SharedPtr power_sonar_cmd_srv;  //Ultrasonic option//


	//Custom Service function command////////////////////////////////////////////////////////////////
	bool LED_Control_Command(
		const std::shared_ptr<interfaces::srv::LedControl::Request> request, 
		const std::shared_ptr<interfaces::srv::LedControl::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_set_light(request->id, request->led_brightness);

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool LED_Toggle_Command(
		const std::shared_ptr<interfaces::srv::LedToggleControl::Request> request, 
		const std::shared_ptr<interfaces::srv::LedToggleControl::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_set_light_toggle(
			request->de_index, request->light_accel, request->led_high_brightness, request->light_decel, request->led_low_brightness);
		
		bResult = true;
		response->command_result = bResult;
		return true;
	}
	
	bool Toggle_On_Command(
		const std::shared_ptr<interfaces::srv::ToggleOn::Request> request, 
		const std::shared_ptr<interfaces::srv::ToggleOn::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_toggle_on(request->id);

		bResult = true;
		response->command_result = bResult;
		return true;

	}

	bool Integrallog_Command(
		const std::shared_ptr<interfaces::srv::Integrallog::Request> request, 
		const std::shared_ptr<interfaces::srv::Integrallog::Response> response)
	{
		bool bResult = false;

		time_t timer; 
		struct tm* t; 
		timer = time(NULL); 
		t = localtime(&timer); 

		int m_iVlotage_data[500] = {0, };
		int m_iCurrent_data[500] = {0, };
		string m_strFilePathName;
		string m_colon = ":";
		string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
		string m_temp1 = m_time_stemp + "_V_log";
		string m_temp2 = m_time_stemp + "_I_log";
		
		if(request->value_index == 1)
		{
			m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
			dssp_rs232_power_module_read_Voltage(m_iVlotage_data); 

			fp = fopen(m_strFilePathName.c_str(), "w");
			if(fp == NULL) 
				printf("file is null \n");

			for(int i=0; i<500; i++)
			{
				fprintf(fp, "%.2f \n", (double)m_iVlotage_data[i] / 10.0);
			}
			fclose(fp);
		}
		else if(request->value_index == 2)
		{
			m_strFilePathName = "/home/tetra/LOG/" + m_temp2 + ".txt";  
			dssp_rs232_power_module_read_Current(m_iCurrent_data);

			fp = fopen(m_strFilePathName.c_str(), "w");
			if(fp == NULL) 
				printf("file is null \n");

			for(int j=0; j<500; j++)
			{
				fprintf(fp, "%.2f \n", (double)m_iCurrent_data[j] / 10.0);
			}
			fclose(fp);
		}

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Set_Outport_Command(
		const std::shared_ptr<interfaces::srv::PowerSetOutport::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerSetOutport::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_set_OutputPort(
			request->output0, request->output1, request->output2, request->output3,
			request->output4, request->output5, request->output6, request->output7);

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Set_Single_Outport_Command(
		const std::shared_ptr<interfaces::srv::PowerSetSingleOutport::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerSetSingleOutport::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_set_Single_OutputPort(request->id, request->value);
		
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Get_IO_Status_Command(
		const std::shared_ptr<interfaces::srv::PowerGetIoStatus::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerGetIoStatus::Response> response)
	{
		bool bResult = false;
		(void)request;

		//Input data
		response->input0 = gpio_msg.input0;
		response->input1 = gpio_msg.input1;
		response->input2 = gpio_msg.input2;
		response->input3 = gpio_msg.input3;
		response->input4 = gpio_msg.input4;
		response->input5 = gpio_msg.input5;
		response->input6 = gpio_msg.input6;
		response->input7 = gpio_msg.input7;
		//Output data
		response->output0 = gpio_msg.output0;
		response->output1 = gpio_msg.output1;
		response->output2 = gpio_msg.output2;
		response->output3 = gpio_msg.output3;
		response->output4 = gpio_msg.output4;
		response->output5 = gpio_msg.output5;
		response->output6 = gpio_msg.output6;
		response->output7 = gpio_msg.output7;

		bResult = true;
		response->command_result = bResult;
		return true;

	}

	bool Power_Set_Enable_Command(
		const std::shared_ptr<interfaces::srv::PowerSetEnable::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerSetEnable::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_set_Enable(
			request->power0, request->power1, request->power2, request->power3,
			request->power4, request->power5, request->power6, request->power7);

		bResult = true;
		response->command_result = bResult;
		return true;
	}		

	bool Power_Set_Single_Enable_Command(
		const std::shared_ptr<interfaces::srv::PowerSetSingleEnable::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerSetSingleEnable::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_set_Single_Enable(request->id, request->value);

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Wheel_Enable_Command(
		const std::shared_ptr<interfaces::srv::PowerWheelEnable::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerWheelEnable::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_wheel_enable(request->on);

		bResult = true;
		response->command_result = bResult;
		return true;

	}

	bool Power_Paramter_Read_Command(
		const std::shared_ptr<interfaces::srv::PowerParameterRead::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerParameterRead::Response> response)
	{
		bool bResult = false;
		(void)request;

		dssp_rs232_power_module_parameter_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,
			&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20,
			&m_iParam21,&m_iParam22,&m_iParam23,&m_iParam24,&m_iParam25,&m_iParam26,&m_iParam27,&m_iParam28,&m_iParam29,&m_iParam30,&m_iParam31);
		
		response->p0_uart1_baudrate = m_iParam0;
		response->p1_uart2_baudrate = m_iParam1;
		response->p2_sonar_select = m_iParam2;
		response->p3_sonar_distance_offset = m_iParam3;
		response->p4_sonar_quantity = m_iParam4;
		response->p5_sonar_max_distance = m_iParam5;
		response->p6_nc = m_iParam6;
		response->p7_sonar_none_detect_mode = m_iParam7;
		response->p8_battery_charging_offest = m_iParam8;
		response->p9_battery_charging_gain = m_iParam9;
		response->p10_status_led_min = m_iParam10;
		response->p11_status_led_max = m_iParam11;
		response->p12_battery_led_min = m_iParam12;
		response->p13_battery_led_max = m_iParam13;
		response->p14_led_ch1_offest = m_iParam14;
		response->p15_led_ch2_offest = m_iParam15;
		response->p16_led_ch3_offest = m_iParam16;
		response->p17_led_ch4_offest = m_iParam17;
		response->p18_led_ch5_offest = m_iParam18;
		response->p19_led_ch6_offest = m_iParam19;
		response->p20_led_ch7_offest = m_iParam20;
		response->p21_led_ch8_offest = m_iParam21;
		response->p22_power_enable = m_iParam22;
		response->p23_outport = m_iParam23;
		response->p24_battery_recharge_voltage = m_iParam24;
		response->p25_battery_recharge_offset = m_iParam25;
		response->p26_battery_min = m_iParam26;
		response->p27_battery_max = m_iParam27;
		response->p28_battery_sampling_time = m_iParam28;
		response->p29_conveyor_mode = m_iParam29;
		response->p30_conveyor_terminal_base = m_iParam30;
		response->p31_conveyor_terminal_offset = m_iParam31;
	
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Paramter_Write_Command(
		const std::shared_ptr<interfaces::srv::PowerParameterWrite::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerParameterWrite::Response> response)
	{
		bool bResult = false;
		dssp_rs232_conveyor_module_parameter_write(request->num, request->data);
	
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Data_Read_Command(
		const std::shared_ptr<interfaces::srv::PowerDataRead::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerDataRead::Response> response)
	{
		bool bResult = false;
		(void)request;

		dssp_rs232_power_module_data_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11);

		response->d0_battery_voltage = m_iParam0;
		response->d1_system_current = m_iParam1;
		response->d2_charge_current = m_iParam2;
		response->d3_charge_signal = m_iParam3;
		response->d4_inport_status = m_iParam4;
		response->d5_outport_status = m_iParam5;
		response->d6_power_status = m_iParam6;
		response->d7_charge_terminal_status = m_iParam7;
		response->d8_temperature0 = m_iParam8;
		response->d9_temperature1 = m_iParam9;
		response->d10_mobd_inport_status = m_iParam10;
		response->d11_mobd_outport_status = m_iParam11;

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_ADC_Read_Command(
		const std::shared_ptr<interfaces::srv::PowerAdcRead::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerAdcRead::Response> response)
	{
		bool bResult = false;
		(void)request;

		dssp_rs232_power_module_adc_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);

		response->d0_adc0 = m_iParam0;
		response->d1_adc1 = m_iParam1;
		response->d2_adc2 = m_iParam2;
		response->d3_adc3 = m_iParam3;
		response->d4_adc4 = m_iParam4;
		response->d5_adc5 = m_iParam5;
		response->d6_adc6 = m_iParam6;
		response->d7_adc7 = m_iParam7;

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Version_Read_Command(
		const std::shared_ptr<interfaces::srv::PowerVersionRead::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerVersionRead::Response> response)
	{
		bool bResult = false;
		(void)request;

		dssp_rs232_power_module_version_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);

		response->d0_board_type = m_iParam0;
		response->d1_year2 = m_iParam1;
		response->d2_year1 = m_iParam2;
		response->d3_month2 = m_iParam3;
		response->d4_month1 = m_iParam4;
		response->d5_day2 = m_iParam5;
		response->d6_day1 = m_iParam6;
		response->d7_ver = m_iParam7;

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	////[Option]///////////////////////////////////////////////////////////////////////////////////
	//Conveyor//
	bool Conveyor_Auto_Movement_Command(
		const std::shared_ptr<interfaces::srv::ConveyorAutoMovement::Request> request, 
		const std::shared_ptr<interfaces::srv::ConveyorAutoMovement::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_conveyor_movement(request->start);
	
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Conveyor_Manual_Movement_Command(
		const std::shared_ptr<interfaces::srv::ConveyorManualMovement::Request> request, 
		const std::shared_ptr<interfaces::srv::ConveyorManualMovement::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_conveyor_manual_movement(request->mode);
	
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Conveyor_Parameter_Read_Command(
		const std::shared_ptr<interfaces::srv::ConveyorParameterRead::Request> request, 
		const std::shared_ptr<interfaces::srv::ConveyorParameterRead::Response> response)
	{
		bool bResult = false;
		(void)request;

		dssp_rs232_conveyor_module_parameter_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,
			&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20);
		
		response->p0_uart1_baudrate = m_iParam0;
		response->p1_uart2_baudrate = m_iParam1;
		response->p2_mode_select = m_iParam2;
		response->p3_outport = m_iParam3;
		response->p4_nc = m_iParam4;
		response->p5_nc = m_iParam5;
		response->p6_nc = m_iParam6;
		response->p7_conveyor_dir = m_iParam7;
		response->p8_conveyor_timeout = m_iParam8;
		response->p9_fix_loading_start_delay = m_iParam9;
		response->p10_fix_loading_end_delay = m_iParam10;
		response->p11_fix_unloading_start_delay = m_iParam11;
		response->p12_fix_unloading_motor_delay = m_iParam12;
		response->p13_fix_unloading_end_delay = m_iParam13;
		response->p14_loading_start_delay = m_iParam14;
		response->p15_loading_end_delay = m_iParam15;
		response->p16_unloading_start_delay = m_iParam16;
		response->p17_unloading_end_delay = m_iParam17;
		response->p18_express_quantity = m_iParam18;
		response->p19_termianl_loading_base = m_iParam19;
		response->p20_termianl_unloading_base = m_iParam20;
	
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Conveyor_Parameter_Write_Command(
		const std::shared_ptr<interfaces::srv::ConveyorParameterWrite::Request> request, 
		const std::shared_ptr<interfaces::srv::ConveyorParameterWrite::Response> response)
	{
		bool bResult = false;
		dssp_rs232_conveyor_module_parameter_write(request->num, request->data);
	
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Conveyor_Data_Read_Command(
		const std::shared_ptr<interfaces::srv::ConveyorDataRead::Request> request, 
		const std::shared_ptr<interfaces::srv::ConveyorDataRead::Response> response)
	{
		bool bResult = false;
		(void)request;

		dssp_rs232_conveyor_module_data_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,
			&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13);
	
		response->d0_docking_signal0 = m_iParam0;
		response->d1_docking_signal1 = m_iParam1;
		response->d2_charge_current = m_iParam2;
		response->d3_charge_signal = m_iParam3;
		response->d4_charge_voltage = m_iParam4;
		response->d5_nc = m_iParam5;
		response->d6_nc = m_iParam6;
		response->d7_ad_in1 = m_iParam7;
		response->d8_ad_in2 = m_iParam8;
		response->d9_ad_in3 = m_iParam9;
		response->d10_in_status = m_iParam10;
		response->d11_out_status = m_iParam11;
		response->d12_photo_status = m_iParam12;
		response->d13_conveyor_status = m_iParam13;
	
		bResult = true;
		response->command_result = bResult;
		return true;
	}
	
	//Ultrasonic//
	bool Power_Sonar_Read_Command(
		const std::shared_ptr<interfaces::srv::PowerSonarRead::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerSonarRead::Response> response)
	{
		bool bResult = false;
		(void)request;
		
		dssp_rs232_power_module_sonar_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5);
		response->d0_sonar0 = m_iParam0;
		response->d1_sonar1 = m_iParam1;
		response->d2_sonar2 = m_iParam2;
		response->d3_sonar3 = m_iParam3;
		response->d4_sonar4 = m_iParam4;
		response->d5_sonar5 = m_iParam5;
		
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Sonar_Command(
		const std::shared_ptr<interfaces::srv::PowerSonarCmd::Request> request, 
		const std::shared_ptr<interfaces::srv::PowerSonarCmd::Response> response)
	{
		bool bResult = false;
		dssp_rs232_power_module_set_Ultrasonic(request->start);
	
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
	auto node = std::make_shared<TETRA_INTERFACE>();

	//init
	node->current_time = node->now();
    node->last_time = node->now();


	rclcpp::WallRate loop_rate(30); //default: 30HZ
	sprintf(port, "/dev/ttyS1");
	//sprintf(port, "/dev/TETRA");

	//RS232 Connect
	if(dssp_rs232_power_module_create(port, 200) == 0)
	{
		printf("TETRA_Power_rs232 Port Open Success\n");
	}
	else
	{
		printf("TETRA_Power_rs232 Port Open Error!\n");
		return -1;
	}

	//Charge port Enable//
	dssp_rs232_power_module_set_charging_ready(1);

	printf("□■■■■■□□□□□□□■□□■■■■□□□□□□□■■■■■□□□□□□□□□□□□□□□□□□□□□□□□□■□ \n");
	printf("□■□□□□■□□□□□□■□■□□□□■□□□□□□■□□□□■□□□□□□□□□□□□□□□□□□□□□□□□■□ \n");
	printf("□■□□□□■□□□□□■□□■□□□□■□□□□□□■□□□□■□□□□□□□□□□□□□□□□□□□□□□□□■□ \n");
	printf("□■□□□□■□□□□□■□□■□□□□□□□□□□□■□□□□■□□□■■■□□□□■■■□□□■□■□□■■■■□ \n");
	printf("□■□□□□■□□□□■□□□□■■□□□□□□□□□■■■■■□□□■□□□■□□■□□□■□□■■□□■□□□■□ \n");
	printf("□■■■■■□□□□□■□□□□□□■■□□□□□□□■□□□□■□□■□□□■□□□□□□■□□■□□□■□□□■□ \n");
	printf("□■□□□□□□□□■□□□□□□□□□■□□□□□□■□□□□■□□■□□□■□□□■■■■□□■□□□■□□□■□ \n");
	printf("□■□□□□□□□□■□□□□■□□□□■□□□□□□■□□□□■□□■□□□■□□■□□□■□□■□□□■□□□■□ \n");
	printf("□■□□□□□□□■□□□□□■□□□□■□□□□□□■□□□□■□□■□□□■□□■□□■■□□■□□□■□□□■□ \n");
	printf("□■□□□□□□□■□□□□□□■■■■□□□□□□□■■■■■□□□□■■■□□□□■■□■□□■□□□□■■■■□ \n");

	while (rclcpp::ok() && !stop_requested)
	{
		rclcpp::spin_some(node);

		//Power Status Check Loop (error check)
		m_iPowerCheck = dssp_rs232_power_module_read_tetra(&m_dbattery, &m_dVoltage, &m_dCurrent, &m_imode_status, m_iInput, m_iOutput, m_dUltrasonic);
		if(m_iPowerCheck < 0)
		{
			printf("!!!! Power Board data read Error(m_iPowerCheck: %d) !!!! \n", m_iPowerCheck);
			if(m_iFlag_PowerCheck_cnt > 10)
			{
				//Error 
				printf("[Error]: Power Board Disconnect !!! \n");
				Error_Log_write("Power Board Disconnect !!!");
				//servo Off call
				std_msgs::msg::Int32 servo;
				servo.data = 2;
				node->servo_publisher->publish(servo);
			}
			else
			{
				m_iFlag_PowerCheck_cnt++;
			}
			loop_rate.sleep();
			continue;
		}
		else
		{
			m_iFlag_PowerCheck_cnt = 0;
		}

		//Power Board Status Check (Error Check)
		std_msgs::msg::Int32 power_status;
		power_status.data = m_iPowerCheck;
		node->power_status_publisher->publish(power_status);

		//Battery Level Check
		std_msgs::msg::Int32 battery_level;
		m_ibattery_Level = (int)m_dbattery;
		battery_level.data = m_ibattery_Level;
		node->tetra_battery_publisher->publish(battery_level);

		//Battery Voltage Check
		std_msgs::msg::Float64 battery_voltage;
		battery_voltage.data = m_dVoltage;
		node->battery_voltage_publisher->publish(battery_voltage);

		//Battery Current Check
		std_msgs::msg::Float64 battery_current;
		battery_current.data = m_dCurrent;
		node->battery_current_publisher->publish(battery_current);

		//Docking Status Check
		std_msgs::msg::Int32 docking_status;
		docking_status.data = m_imode_status;
		node->docking_status_publisher->publish(docking_status);
		
		//GPIO Check//
		//Input data
		gpio_msg.input0 = m_iInput[0];
		gpio_msg.input1 = m_iInput[1];
		gpio_msg.input2 = m_iInput[2];
		gpio_msg.input3 = m_iInput[3];
		gpio_msg.input4 = m_iInput[4];
		gpio_msg.input5 = m_iInput[5];
		gpio_msg.input6 = m_iInput[6];
		gpio_msg.input7 = m_iInput[7];
		//Output data
		gpio_msg.output0 = m_iOutput[0];
		gpio_msg.output1 = m_iOutput[1];
		gpio_msg.output2 = m_iOutput[2];
		gpio_msg.output3 = m_iOutput[3];
		gpio_msg.output4 = m_iOutput[4];
		gpio_msg.output5 = m_iOutput[5];
		gpio_msg.output6 = m_iOutput[6];
		gpio_msg.output7 = m_iOutput[7];
		node->gpio_status_publisher->publish(gpio_msg);

		//Ultrasonic data Check_Option/////////////////////////////////////////////////////////
		//Ultrasonic Paramter Setting
		// char frameid1[] = "/Ultrasonic_Down_Left";
		// range_msg1.header.frame_id = frameid1;
		// range_msg1.radiation_type = 0; //Ultrasonic
		// range_msg1.field_of_view = (60.0/180.0) * M_PI; //
		// range_msg1.min_range = Ultrasonic_MIN_range; 
		// range_msg1.max_range = Ultrasonic_MAX_range; 

		// char frameid2[] = "/Ultrasonic_Rear_Left";
		// range_msg2.header.frame_id = frameid2;
		// range_msg2.radiation_type = 0; //Ultrasonic
		// range_msg2.field_of_view = (60.0/180.0) * M_PI; //
		// range_msg2.min_range = Ultrasonic_MIN_range;
		// range_msg2.max_range = Ultrasonic_MAX_range;

		// char frameid3[] = "/Ultrasonic_Rear_Right";
		// range_msg3.header.frame_id = frameid3;
		// range_msg3.radiation_type = 0; //Ultrasonic
		// range_msg3.field_of_view = (60.0/180.0) * M_PI; //
		// range_msg3.min_range = Ultrasonic_MIN_range;
		// range_msg3.max_range = Ultrasonic_MAX_range;

		// char frameid4[] = "/Ultrasonic_Down_Right";
		// range_msg4.header.frame_id = frameid4;
		// range_msg4.radiation_type = 0; //Ultrasonic
		// range_msg4.field_of_view = (60.0/180.0) * M_PI; //
		// range_msg4.min_range = Ultrasonic_MIN_range;
		// range_msg4.max_range = Ultrasonic_MAX_range;

		// if(m_dUltrasonic[0] == 0.0)
		// 	range_msg1.range = Ultrasonic_MAX_range;
		// else
		// 	range_msg1.range = m_dUltrasonic[0];

		// range_msg1.header.stamp = node->current_time ;

		// if(m_dUltrasonic[1] == 0.0)
		// 	range_msg2.range = Ultrasonic_MAX_range;
		// else
		// 	range_msg2.range = m_dUltrasonic[1];

		// range_msg2.header.stamp = node->current_time ;

		// if(m_dUltrasonic[2] == 0.0)
		// 	range_msg3.range = Ultrasonic_MAX_range;
		// else
		// 	range_msg3.range = m_dUltrasonic[2];

		// range_msg3.header.stamp = node->current_time ;

		// if(m_dUltrasonic[3] == 0.0)
		// 	range_msg4.range = Ultrasonic_MAX_range;
		// else
		// 	range_msg4.range = m_dUltrasonic[3];
			
		// range_msg4.header.stamp = node->current_time ;

		// //Ultrasonic Publish
		// if(m_bUltrasonic_option) //Option --> default: false
		// {
		// 	node->Ultrasonic1_publisher->publish(range_msg1);
		// 	node->Ultrasonic2_publisher->publish(range_msg2);
		// 	node->Ultrasonic3_publisher->publish(range_msg3);
		// 	node->Ultrasonic4_publisher->publish(range_msg4);
		// }
		///////////////////////////////////////////////////////////////////////////////////////

		//Conveyor data Check_Option///////////////////////////////////////////////////////////
		if(m_bConveyor_option)
		{
			//Conveyor Sensor status
			std_msgs::msg::Int32 conveyor_sensor;
			dssp_rs232_power_module_read_conveyor_sensor(&m_dConveyor_sensor);
			conveyor_sensor.data = m_dConveyor_sensor;
			node->conveyor_sensor_publisher->publish(conveyor_sensor);

			//Conveyor Movement status
			std_msgs::msg::Int32 conveyor_movement;
			dssp_rs232_power_module_read_conveyor_movement(&m_iConveyor_movement);
			conveyor_movement.data = m_iConveyor_movement;
			node->conveyor_movement_publisher->publish(conveyor_movement);
		}
		///////////////////////////////////////////////////////////////////////////////////////

		//Todo...
		
		loop_rate.sleep();
    }

	//Ultrasonic Off//
	dssp_rs232_power_module_set_Ultrasonic(0);
	//RS232 Disconnect
	dssp_rs232_power_module_destroy();

	rclcpp::shutdown();
    return 0;
}
