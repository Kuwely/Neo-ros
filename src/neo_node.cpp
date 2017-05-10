/*The MIT License (MIT)
 *
 * Copyright (c) 2017, micvision, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "ros/ros.h"
#include "neo/neo.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD(x) ((x) * 0.017453293)

NeoDriver *drv = NULL;
status result;

bool checkDeviceInfo(NeoDriver *drv)
{
	status result;
	neo_response_info_device_t info_d;
	neo_response_info_version_t info_v;

	if (drv->getDeviceInfo(&info_d))
	{
		ROS_ERROR_STREAM("Error, couldn't retrieve device info");
	}

	if (drv->getVersionInfo(&info_v))
	{
		ROS_ERROR_STREAM("Error, couldn't retrieve device info");
	}

	if (strncmp((const char *) info_v.model, "NEO01", 5))
	{
		ROS_ERROR_STREAM("Error, unknown Neo Model");
		return false;
	}

	if (info_d.diagnostic != '0') //Check Device health is good
	{
		ROS_ERROR_STREAM("Error, device diagnostic info incorrect");
		return false;
	}

	return true;
}

void publish_scan_directly(ros::Publisher *pub,
				  neo_response_scan_packet_t *nodes,
				  size_t node_count, ros::Time start,
				  double scan_time, std::string frame_id)
{
	/*
	 * output: the ros publish data.
	 */
	sensor_msgs::LaserScan output;
	/*
	 * Using inf or output.range_max+1 for some point cannot obtain.
	 */
	bool use_inf_ = true;

	/*
	 * Fill full the output.header
	 */
	output.header.frame_id = "laser_frame";
	output.header.stamp = start;

	output.angle_min = -3.14;
	output.angle_max = 3.14;
	output.angle_increment = 0.001;
	output.time_increment= 0.0;
	//output.scan_time = 0.1;
	output.range_min = 0.0;
	output.range_max = 40.0;

	/*
	 * ranges_size: size of output.ranges[]
	 */
	uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

	if (use_inf_)
	{
		output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
	} else {
		output.ranges.assign(ranges_size, output.range_max + 1.0);
	}

	/*
	 * Fill full output.ranges[]
	 */
	for (size_t i = 0; i < node_count; i++) {
		/*
		 * range: distance/100, centimeter(CM) to meter(M).
		 */
		float range = (float) nodes[i].distance / 100;
		float angle_f = INT2FLOAT(nodes[i].angle);
		float angle;

		/*
		 * Angle: degree to radian
		 */
		if (angle_f >= 360)
			angle = DEG2RAD(180);
		else if (angle_f >= 180)
			angle = DEG2RAD(angle_f - 360);
		else
			angle = DEG2RAD(angle_f);

		/*
		 * Double check range and angle.
		 */
		if (range < output.range_min) {
			ROS_ERROR_STREAM("Error, range is too samll");
			continue;
		}
		/*
		 *if (angle < output.angle_min || angle > output.angle_max) {
		 *    ROS_ERROR_STREAM("Error, angle is out of range");
		 *    continue;
		 *}
		 */

		int index = (angle - output.angle_min) / output.angle_increment;
		if (range < output.ranges[index])
			output.ranges[index] = range;
	}

	pub->publish(output);
}


int main(int argc, char *argv[])
{
	//Initialize Node and handles
	ros::init(argc, argv, "neo_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	//Get Serial Parameters
	std::string serial_port;
	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
	int serial_baudrate;
	nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);

	//Get Scanner Parameters
	int rotation_speed;
	nh_private.param<int>("rotation_speed", rotation_speed, 5);

	//Get frame id Parameters
	std::string frame_id;
	nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

	//Setup Publisher
	ros::Publisher scan_pub_directly = nh.advertise<sensor_msgs::LaserScan>("scan", 10);

	//Create Neo Driver Object
	drv = new NeoDriver();

	if (!drv)
	{
		fprintf(stderr, "Create Driver fail, exit\n");
		return -2;
	}

	//Connect to Device
	if (drv->connect(serial_port.c_str(), serial_baudrate, 1000))
	{
		fprintf(stderr, "Error, Serial Couldn't Connect\n");
		delete drv;
		return -1;
	}

	//Send Rotation Speed
	if (drv->changeMotorSpeed(rotation_speed))
	{
		ROS_ERROR_STREAM("Error, couldn't set rotation speed");
	}
	else
	{
		ROS_INFO("expected rotation frequency: %d (Hz)", rotation_speed);
	}

	//Verify Device Info
	if (!checkDeviceInfo(drv))
	{
		ROS_ERROR_STREAM("Error, coudln't verify device info");
	}

	//Start Scan
	drv->startScan();

	ros::Time start_scan_time;
	ros::Time end_scan_time;
	double scan_duration;
	while (ros::ok())
	{
		neo_response_scan_packet_t nodes[MAX_SCAN_PACKETS];
		size_t count = countof(nodes);

		//Grab Full Scan
		start_scan_time = ros::Time::now();
		result = drv->getCompleteScan(nodes, count, 3000);
		end_scan_time = ros::Time::now();

		scan_duration = (end_scan_time - start_scan_time).toSec();

		if (result == STATUS_OK)
		{
			publish_scan_directly(&scan_pub_directly, nodes, count, start_scan_time, scan_duration, frame_id);
		}
		else if (result == STATUS_FAIL)
		{
			ROS_ERROR_STREAM("Error, failed scan");
			// The data is invalid, publish anyways
			publish_scan_directly(&scan_pub_directly, nodes, count, start_scan_time, scan_duration, frame_id);
		}
		ros::spinOnce();
	}
	// destroy driver
	delete drv;
	return 0;
}
