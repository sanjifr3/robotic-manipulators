#include <ros/ros.h>
#include <std_msgs/String.h>
#include <altitude_sensor/sensor_data.h>
#include <cereal_port/CerealPort.h>

#define REPLY_SIZE 20
#define TIMEOUT 1000
	
	const unsigned int sensor_frequency = 60; /*sensor frequency in hz*/

	double altitude;
	double voltage;
	std::string serial_port;
	unsigned int message_shift = 0;
	unsigned int voltage_message_begin =message_shift+8;
	unsigned int voltage_message_end =message_shift+15;
	unsigned int altitude_message_begin =message_shift;
	unsigned int altitude_message_end =message_shift+6;

int main(int argc, char** argv) {


	ros::init(argc, argv, "alt_sensor_node");
	ros::NodeHandle nh;
	
	/* ros::Publisher alt_pub=nh.advertise<std_msgs::Float64>("old_altitude",5); */
	ros::Publisher value_pub=nh.advertise<altitude_sensor::sensor_data>("altitude", 5);
	
	ros::Rate loop_rate(sensor_frequency); 
	
	cereal::CerealPort device;
	char reply[REPLY_SIZE];
	
	altitude_sensor::sensor_data message;
	

	
	nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	


	
		
	try{ device.open(serial_port.c_str() , 115200); }
        catch(cereal::Exception& e)
        {
        ROS_FATAL("Failed to open the serial port.");
        ROS_BREAK();
        }
        ROS_INFO("The serial port is opened.");

	
	while(ros::ok()) {
		
		std::stringstream ss1;
		std::stringstream ss2;
		

 		try{ device.readBytes(reply, REPLY_SIZE, TIMEOUT); }
    		catch(cereal::TimeoutException& e)
       		{
                ROS_ERROR("Timeout!");
                }

		
		/*getting the altitude */
		for(unsigned int i=altitude_message_begin; i<=altitude_message_end; i++) {
			ss1<<reply[i];
		}		
		ss1 >> altitude;
		
		message.altitude=altitude;

		/*getting the voltage */
		for(unsigned int j=voltage_message_begin; j<=voltage_message_end; j++) {
			ss2<<reply[j];
		}		
		ss2 >>voltage;
		message.voltage=voltage;
		message.header.frame_id="altitude sensor";
		message.header.stamp=ros::Time::now();
		
		
		
				
	
		
		if(isdigit(reply[altitude_message_end])) {
		
		}
		else {
		
		}

		value_pub.publish(message);

		
		
		
		
		/* for synchronisation */
		if(reply[message_shift+7]!='m') { 
			
			message_shift++;
			
			voltage_message_begin =message_shift+10;
			voltage_message_end =message_shift+14;
			altitude_message_begin =message_shift;
			altitude_message_end =message_shift+5;

			if(message_shift>=20) message_shift=message_shift-20;
			if(voltage_message_begin>=20) voltage_message_begin=voltage_message_begin-20;
			if(voltage_message_end>=20) voltage_message_end=voltage_message_end-20;
			if(altitude_message_begin>=20) altitude_message_begin=altitude_message_begin-20;
			if(altitude_message_end>=20) altitude_message_end=altitude_message_end-20;
				
			
		}
		
		loop_rate.sleep(); 
		
	}
	
	ros::spin();

	return 0; 
}
