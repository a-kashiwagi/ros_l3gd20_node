/******************************************************************************/
/*                                                                            */
/* Title  : STMicro L3GD20 MEMS Gyro Driver for ROS                           */
/* Program Name : l3gd20_node                                                 */
/*                                                                            */
/* Detail :                                                                   */
/* Date   : 2017/05/22                                                        */
/* Author : Akihiro Kashiwagi                                                 */
/* E-mail : kashiwagi@gridsolar.jp                                            */
/*                                                                            */
/* Replace -------------------------------------------------------------------*/
/*                                                                            */
/* Date   :                                                                   */
/* Author :                                                                   */
/* Deteil :                                                                   */
/*                                                                            */
/*-------+---------+---------+---------+---------+---------+---------+--------*/
/*3456789012345678901234567890123456789012345678901234567890123456789012345678*/
/******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
					// For WiringPI
#include <wiringPi.h>
#include <wiringPiI2C.h>
					// For ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

/******************************************************************************/
/*                                                                            */
/* Title  : Class for IMU STMicro L3GD20 MEMS Gyro                            */
/* Class Name : ImuL3GD20                                                     */
/*                                                                            */
/* Detail :                                                                   */
/* Date   : 2017/05/22                                                        */
/* Author : Akihiro Kashiwagi                                                 */
/* E-mail : kashiwagi@gridsolar.jp                                            */
/*                                                                            */
/* Replace -------------------------------------------------------------------*/
/*                                                                            */
/* Date   :                                                                   */
/* Author :                                                                   */
/* Deteil :                                                                   */
/*                                                                            */
/*-------+---------+---------+---------+---------+---------+---------+--------*/
/*3456789012345678901234567890123456789012345678901234567890123456789012345678*/
/******************************************************************************/

					// Class L3GD20
class ImuL3GD20{

private:
					// Publisher
	ros::Publisher imu_data_pub_;
					// Publisher for raw
	ros::Publisher imu_raw_pub_;

	ros::Subscriber update_calibration_sub_;

					// ROS Handler for private NH
	ros::NodeHandle private_nh_;
					// Frame ID
	std::string imu_frame_id_;
					// IMU data
	sensor_msgs::Imu imu_data;
					// IMU data for raw
	sensor_msgs::Imu imu_raw;
					// Delta T
	double dt;
					// Orientation
	double orientation_x;
	double orientation_y;
	double orientation_z;
					// Stored orientation
	double last_orientation_x;
	double last_orientation_y;
	double last_orientation_z;
					// Orientation for raw
	double raw_orientation_x;
	double raw_orientation_y;
	double raw_orientation_z;
					// Caribration
	double calib_x;
	double calib_y;
	double calib_z;
					// Stored number of calibration
	double last_calib_x;
	double last_calib_y;
	double last_calib_z;
					// Counter of calibration
	long calib_cnt;
					// Store current time
	ros::Time last_time;
					// Store x,y,z
	double last_x;
	double last_y;
	double last_z;
					// Summary
	double sum_x;
	double sum_y;
	double sum_z;
					// File descriptor
        int fd;
					// Unit
	double UNIT;
					// Loop rate
	double rate;
					// Calibration time[s]
	double calib_sec;

					// orientation covariances;
	double orientation_covariance_0;
	double orientation_covariance_1;
	double orientation_covariance_2;
	double orientation_covariance_3;
	double orientation_covariance_4;
	double orientation_covariance_5;
	double orientation_covariance_6;
	double orientation_covariance_7;
	double orientation_covariance_8;
					// angular velocity covariances
	double angular_velocity_covariance_0;
	double angular_velocity_covariance_1;
	double angular_velocity_covariance_2;
	double angular_velocity_covariance_3;
	double angular_velocity_covariance_4;
	double angular_velocity_covariance_5;
	double angular_velocity_covariance_6;
	double angular_velocity_covariance_7;
	double angular_velocity_covariance_8;
					// linear acceleration covariances
	double linear_acceleration_covariance_0;
	double linear_acceleration_covariance_1;
	double linear_acceleration_covariance_2;
	double linear_acceleration_covariance_3;
	double linear_acceleration_covariance_4;
	double linear_acceleration_covariance_5;
	double linear_acceleration_covariance_6;
	double linear_acceleration_covariance_7;
	double linear_acceleration_covariance_8;

/******************************************************************************/
/*                                                                            */
/* Title  : Constructor of ImuL3GD20                                          */
/* Method Name : ImuL3GD20                                                    */
/*                                                                            */
/* Detail :                                                                   */
/* Date   : 2017/05/22                                                        */
/* Author : Akihiro Kashiwagi                                                 */
/* E-mail : kashiwagi@gridsolar.jp                                            */
/*                                                                            */
/* Replace -------------------------------------------------------------------*/
/*                                                                            */
/* Date   :                                                                   */
/* Author :                                                                   */
/* Deteil :                                                                   */
/*                                                                            */
/*-------+---------+---------+---------+---------+---------+---------+--------*/
/*3456789012345678901234567890123456789012345678901234567890123456789012345678*/
/******************************************************************************/

public:
	ImuL3GD20(){
					// ID for I2C
		int ID;
					// ROS Handler
		ros::NodeHandle node;
					// Publisher
		imu_data_pub_
			= node.advertise<sensor_msgs::Imu>("imu/data", 10);
		imu_raw_pub_
			= node.advertise<sensor_msgs::Imu>("imu/raw", 10);

					// Subscriber
		update_calibration_sub_
			= node.subscribe(
				"imu/update_calibration",
				1000,
				&ImuL3GD20::update_calibration,
				this
			);
					// Init
		dt = 0.0;

		orientation_x =0.0;
		orientation_y =0.0;
		orientation_z =0.0;

		calib_x = 0.0;
		calib_y = 0.0;
		calib_z = 0.0;

		last_calib_x = 0.0;
		last_calib_y = 0.0;
		last_calib_z = 0.0;

		last_x = 999;
		last_y = 999;
		last_z = 999;

		sum_x = 0;
		sum_y = 0;
		sum_z = 0;

					// Parameters
		node.param("rate", rate, 100.0);
		node.param("calibration_second", calib_sec, 10.0);
		node.param("unit", UNIT, 0.0175);

		node.param(
			"orientation_covariance_0",
			 orientation_covariance_0,
			1e6
		);
		node.param(
			"orientation_covariance_1",
			 orientation_covariance_1,
			0.0
		);
		node.param(
			"orientation_covariance_2",
			 orientation_covariance_2,
			0.0
		);
		node.param(
			"orientation_covariance_3",
			 orientation_covariance_3,
			0.0
		);
		node.param(
			"orientation_covariance_4",
			 orientation_covariance_4,
			1e6
		);
		node.param(
			"orientation_covariance_5",
			 orientation_covariance_5,
			0.0
		);
		node.param(
			"orientation_covariance_6",
			 orientation_covariance_6,
			0.0
		);
		node.param(
			"orientation_covariance_7",
			 orientation_covariance_7,
			0.0
		);
		node.param(
			"orientation_covariance_8",
			 orientation_covariance_8,
			1e-6
		);

		node.param(
			"angular_velocity_covariance_0",
			 angular_velocity_covariance_0,
			1e6
		);
		node.param(
			"angular_velocity_covariance_1",
			 angular_velocity_covariance_1,
			0.0
		);
		node.param(
			"angular_velocity_covariance_2",
			 angular_velocity_covariance_2,
			0.0
		);
		node.param(
			"angular_velocity_covariance_3",
			 angular_velocity_covariance_3,
			0.0
		);
		node.param(
			"angular_velocity_covariance_4",
			 angular_velocity_covariance_4,
			1e6
		);
		node.param(
			"angular_velocity_covariance_5",
			 angular_velocity_covariance_5,
			0.0
		);
		node.param(
			"angular_velocity_covariance_6",
			 angular_velocity_covariance_6,
			0.0
		);
		node.param(
			"angular_velocity_covariance_7",
			 angular_velocity_covariance_7,
			0.0
		);
		node.param(
			"angular_velocity_covariance_8",
			 angular_velocity_covariance_8,
			1e-6
		);

		node.param(
			"linear_acceleration_covariance_0",
			 linear_acceleration_covariance_0,
			-1.0
		);
		node.param(
			"linear_acceleration_covariance_1",
			 linear_acceleration_covariance_1,
			0.0
		);
		node.param(
			"linear_acceleration_covariance_2",
			 linear_acceleration_covariance_2,
			0.0
		);
		node.param(
			"linear_acceleration_covariance_3",
			 linear_acceleration_covariance_3,
			0.0
		);
		node.param(
			"linear_acceleration_covariance_4",
			 linear_acceleration_covariance_4,
			0.0
		);
		node.param(
			"linear_acceleration_covariance_5",
			 linear_acceleration_covariance_5,
			0.0
		);
		node.param(
			"linear_acceleration_covariance_6",
			 linear_acceleration_covariance_6,
			0.0
		);
		node.param(
			"linear_acceleration_covariance_7",
			 linear_acceleration_covariance_7,
			0.0
		);
		node.param(
			"linear_acceleration_covariance_8",
			 linear_acceleration_covariance_8,
			0.0
		);
					// Set frame ID
		private_nh_.param<std::string>(
			"frame_id", imu_frame_id_, "imu_link"
		);
					// Covariences
		imu_data.orientation_covariance = {
			orientation_covariance_0,
			orientation_covariance_1,
			orientation_covariance_2,
			orientation_covariance_3,
			orientation_covariance_4,
			orientation_covariance_5,
			orientation_covariance_6,
			orientation_covariance_7,
			orientation_covariance_8
		};

		imu_data.angular_velocity_covariance = {
			angular_velocity_covariance_0,
			angular_velocity_covariance_1,
			angular_velocity_covariance_2,
			angular_velocity_covariance_3,
			angular_velocity_covariance_4,
			angular_velocity_covariance_5,
			angular_velocity_covariance_6,
			angular_velocity_covariance_7,
			angular_velocity_covariance_8
		};

		imu_data.linear_acceleration_covariance = {
			linear_acceleration_covariance_0,
			linear_acceleration_covariance_1,
			linear_acceleration_covariance_2,
			linear_acceleration_covariance_3,
			linear_acceleration_covariance_4,
			linear_acceleration_covariance_5,
			linear_acceleration_covariance_6,
			linear_acceleration_covariance_7,
			linear_acceleration_covariance_8
		};
					// For raw data
		imu_raw.orientation_covariance
			= imu_data.orientation_covariance;

		imu_raw.angular_velocity_covariance
			= imu_data.angular_velocity_covariance;

		imu_raw.linear_acceleration_covariance
			= imu_data.linear_acceleration_covariance;

					// Number of calibration
		calib_cnt = rate * calib_sec;
					// ID of I2C
		ID = 0x6b;
					// Unit
		//UNIT = 0.0175;
					// Who am I for I2C
		fd = wiringPiI2CSetup(ID);
		ROS_INFO("I2C ID: %#x",ID);
		ROS_INFO("setup return : %d",fd);

					// Start senser

		if((wiringPiI2CWriteReg8(fd,0x20,0x0F)) < 0){
			ROS_INFO("write error register 0x20");
		}
		ROS_INFO("write register:0x20 = 0x0F");

					// Set range

		if((wiringPiI2CWriteReg8(fd,0x23,0x00)) < 0){
			ROS_INFO("write error register 0x23");
		}
		ROS_INFO("write register:0x23 = 0x00");
	}

/******************************************************************************/
/*                                                                            */
/* Title  : Get number of axsises from gyro                                   */
/* Method Name : getAxsis                                                     */
/*                                                                            */
/* Detail :                                                                   */
/* Date   : 2017/05/22                                                        */
/* Author : Akihiro Kashiwagi                                                 */
/* E-mail : kashiwagi@gridsolar.jp                                            */
/*                                                                            */
/* Replace -------------------------------------------------------------------*/
/*                                                                            */
/* Date   :                                                                   */
/* Author :                                                                   */
/* Deteil :                                                                   */
/*                                                                            */
/*-------+---------+---------+---------+---------+---------+---------+--------*/
/*3456789012345678901234567890123456789012345678901234567890123456789012345678*/
/******************************************************************************/

	int getAxsis( double *x, double *y, double *z ){

		int h;
		int l;
		int hl;
		double dps;
					// read OUT_X_L
		l = wiringPiI2CReadReg8(fd,0x28);
		h = wiringPiI2CReadReg8(fd,0x29);

					// Int is 4[byte]
		hl = h;
					// H contain 8[bit] data.
		hl = hl << 8;
					// L contain 8[bit] data.
		hl = hl | l;
					// H+L is 16[bit] data.
		hl = hl << 16;
					// Input data is
					//     number of 2's complement.
		dps = (double)(hl / 0x1FFFF) * UNIT;

		if( last_x == 999 ){
					// Initial setting
			last_x = dps;
		}
					// Output data
		*x = dps - last_x;

		//printf("%07.2f, %07.2f, ",dps-last_x,sum_x);
		//sum_x += (dps - last_x) * 0.03;

					// read OUT_Y_L
		l = wiringPiI2CReadReg8(fd,0x2A);
		h = wiringPiI2CReadReg8(fd,0x2B);

		hl = h;
		hl = hl << 8;
		hl = hl | l;
		hl = hl << 16;
		dps = (double)(hl / 0x1FFFF) * UNIT;

		if( last_y == 999 ){

			last_y = dps;
		}

		*y = dps - last_y;

		//printf("%07.2f, %07.2f, ",dps-last_y,sum_y);
		//sum_y += (dps - last_y) * 0.03;

					// read OUT_Z_L
		l = wiringPiI2CReadReg8(fd,0x2C);
		h = wiringPiI2CReadReg8(fd,0x2D);

		hl = h;
		hl = hl << 8;
		hl = hl | l;
		hl = hl << 16;
		dps = (double)(hl / 0x1FFFF) * UNIT;

		if( last_z == 999 ){

			last_z = dps;
		}

		*z = dps - last_z;

		//printf("%07.2f, %07.2f\n",dps-last_z,sum_z);
		//sum_z += (dps - last_z) * 0.03;

		return 0;
	}

	void update_calibration(const std_msgs::Int32::ConstPtr& msg){

		ROS_INFO("Calibration start...");
		calib_cnt = 100 * 10;
	}

/******************************************************************************/
/*                                                                            */
/* Title  : Publisher for imu/data and imu/raw                                */
/* Method Name : pub                                                          */
/*                                                                            */
/* Detail :                                                                   */
/* Date   : 2017/05/22                                                        */
/* Author : Akihiro Kashiwagi                                                 */
/* E-mail : kashiwagi@gridsolar.jp                                            */
/*                                                                            */
/* Replace -------------------------------------------------------------------*/
/*                                                                            */
/* Date   :                                                                   */
/* Author :                                                                   */
/* Deteil :                                                                   */
/*                                                                            */
/*-------+---------+---------+---------+---------+---------+---------+--------*/
/*3456789012345678901234567890123456789012345678901234567890123456789012345678*/
/******************************************************************************/

	void pub(){

		double x;
		double y;
		double z;
					// Get current time
		ros::Time current_time = ros::Time::now();

					// Get number of axsises
		getAxsis( &x, &y, &z );

		if( calib_cnt >= 0 ){
					// Procedure of calibration
			sum_x += x - last_calib_x;
			sum_y += y - last_calib_y;
			sum_z += z - last_calib_z;

			calib_cnt--;

			if( calib_cnt == 0 ){
				calib_x /= rate * calib_sec;
				calib_y /= rate * calib_sec;
				calib_z /= rate * calib_sec;

				ROS_INFO("Calibration done.");
			}
		}
					// Set timestamp
		imu_data.header.stamp = current_time;

					// Set frame ID
		imu_data.header.frame_id = imu_frame_id_;

					// Set angular velocitys xyz
		imu_data.angular_velocity.x
			= (x - calib_x) * M_PI / 180 * (-1.0);

		imu_data.angular_velocity.y
			= (y - calib_y) * M_PI / 180;

		imu_data.angular_velocity.z
			= (z - calib_z) * M_PI / 180 * (-1.0);

		if( dt == 0 ){
					// Set delta T
			dt = 1 / rate;
		}else{
			dt = (current_time - last_time).toSec();
		}
					// Summary orientations
		orientation_x += imu_data.angular_velocity.x * dt;
		orientation_y += imu_data.angular_velocity.y * dt;
		orientation_z += imu_data.angular_velocity.z * dt;

					// Transfer to quaternion
		tf::Quaternion q = tf::createQuaternionFromRPY (
			orientation_x,
			orientation_y,
			orientation_z
		); 
					// Set imu_data
		imu_data.orientation.x = q.x();
		imu_data.orientation.y = q.y();
		imu_data.orientation.z = q.z();
		imu_data.orientation.w = q.w();

					// Publish imu_data
		imu_data_pub_.publish(imu_data);

					// Set timestamp
		imu_raw.header.stamp = current_time;

					// Set frame ID
		imu_raw.header.frame_id = imu_frame_id_;

					// Set angular velocitys xyz
		imu_raw.angular_velocity.x = imu_data.angular_velocity.x;
		imu_raw.angular_velocity.y = imu_data.angular_velocity.y;
		imu_raw.angular_velocity.z = imu_data.angular_velocity.z;

					// Summary orientations
		raw_orientation_x
			+= last_orientation_x
			+ imu_data.angular_velocity.x * dt;

		raw_orientation_y
			+= last_orientation_y
			+ imu_data.angular_velocity.y * dt;

		raw_orientation_z
			+= last_orientation_z
			+ imu_data.angular_velocity.z * dt;

					// Transfer to quaternion
		q = tf::createQuaternionFromRPY (
			raw_orientation_x,
			raw_orientation_y,
			raw_orientation_z
		); 
					// Set imu_data
		imu_raw.orientation.x = q.x();
		imu_raw.orientation.y = q.y();
		imu_raw.orientation.z = q.z();
		imu_raw.orientation.w = q.w();

					// Publish imu_raw
		imu_raw_pub_.publish(imu_raw);

					// Store current time
		last_time = current_time;

					// Store orientations
		last_orientation_x = orientation_x;
		last_orientation_y = orientation_y;
		last_orientation_z = orientation_z;
	}
};

/******************************************************************************/
/*                                                                            */
/* Title  : Main function                                                     */
/* Method Name : main                                                         */
/*                                                                            */
/* Detail :                                                                   */
/* Date   : 2017/05/22                                                        */
/* Author : Akihiro Kashiwagi                                                 */
/* E-mail : kashiwagi@gridsolar.jp                                            */
/*                                                                            */
/* Replace -------------------------------------------------------------------*/
/*                                                                            */
/* Date   :                                                                   */
/* Author :                                                                   */
/* Deteil :                                                                   */
/*                                                                            */
/*-------+---------+---------+---------+---------+---------+---------+--------*/
/*3456789012345678901234567890123456789012345678901234567890123456789012345678*/
/******************************************************************************/

int main(int argc, char **argv){

	double rate;
					// ROS init
	ros::init(argc, argv, "l3gd20");
					// Time init
	ros::Time::init();

	ros::NodeHandle node;

	node.param("rate", rate, 100.0);

					// Set rate[Hz]
	ros::Rate loop_rate(rate);
					// Messages
	ROS_INFO("Start IMU STMicro L3GD20 MEMS Gyro.");
	ROS_INFO("Sampling rate: %f[Hz]", rate);

					// IMU instance
	ImuL3GD20 l3gd20;

	while(ros::ok()){
					// Publish
		l3gd20.pub();
					// Spin for wait
		ros::spinOnce();
		loop_rate.sleep();
	}
}
