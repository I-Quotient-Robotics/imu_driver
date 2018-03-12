/*
 * ros driver node for JY901 imu
 * guolindong@gmail.com
 * 2018.01.23
 */

#define Pi 3.14159265359

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>

#include "JY901.h"

struct STime    stcTime;
struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;
struct SMag     stcMag;
struct SDStatus stcDStatus;
struct SPress   stcPress;
struct SLonLat  stcLonLat;
struct SGPSV    stcGPSV;
struct SOrien   stcOrien;

// convert serial data to jy901 data
void CopeSerialData(std::string str_in) {
  unsigned int str_length = str_in.size();
  static unsigned char chrTemp[2000];
  static unsigned char ucRxCnt = 0;
  static unsigned int usRxLength = 0;

  memcpy(chrTemp+usRxLength, str_in.data(), str_length);
  usRxLength += str_length;
  while (usRxLength >= 11) {
    if (chrTemp[0] != 0x55) {
      usRxLength--;
      memcpy(&chrTemp[0], &chrTemp[1], usRxLength);
      continue;
    }
    switch(chrTemp[1]) {
      case 0x50: memcpy(&stcTime, &chrTemp[2], 8); break;
      case 0x51: memcpy(&stcAcc, &chrTemp[2], 8); break;
      case 0x52: memcpy(&stcGyro, &chrTemp[2], 8); break;
      case 0x53: memcpy(&stcAngle, &chrTemp[2], 8); break;
      case 0x54: memcpy(&stcMag, &chrTemp[2], 8);break;
      case 0x55: memcpy(&stcDStatus, &chrTemp[2], 8); break;
      case 0x56: memcpy(&stcPress, &chrTemp[2], 8); break;
      case 0x57: memcpy(&stcLonLat, &chrTemp[2], 8); break;
      case 0x58: memcpy(&stcGPSV,&chrTemp[2], 8); break;
      case 0x59: memcpy(&stcOrien, &chrTemp[2], 8); break;
    }
    usRxLength -= 11;
    memcpy(&chrTemp[0], &chrTemp[11], usRxLength);
  }
}

int main (int argc, char** argv) {
  // param
  serial::Serial serial_port;
  std::string port;
  int baudrate;
  int looprate;

  // ros init
  ros::init(argc, argv, "jy901_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get param from launch file
  pnh.param<int>("baudrate", baudrate, 115200);
  pnh.param<std::string>("port", port, "/dev/ttyUSB0");
  pnh.param<int>("looprate", looprate, 100);

  pnh.getParam("baudrate", baudrate);
  pnh.getParam("port", port);
  pnh.getParam("looprate", looprate);

  ROS_INFO_STREAM(port);
  ROS_INFO_STREAM(baudrate);
  ROS_INFO_STREAM(looprate);

  // ros pub and sub
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 50);

  try {
    serial_port.setPort(port);
    serial_port.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_port.setTimeout(to);
    serial_port.open();
    serial_port.setRTS(false);
    serial_port.setDTR(false);
    // serial_port.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open serial port ");
    return -1;
  }

  // check if serial port is open
  if(serial_port.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }

  // set looprate
  ros::Rate loop_rate(looprate);
  while(ros::ok()) {
    if(serial_port.available()>=11){
      // convert serial string to JY901 data
      CopeSerialData(serial_port.read(serial_port.available()));

      // imu sensor msg pub
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "imu_link";
      imu_msg.orientation.w = stcOrien.q[0] / 32768.0;
      imu_msg.orientation.x = stcOrien.q[1] / 32768.0;
      imu_msg.orientation.y = stcOrien.q[2] / 32768.0;
      imu_msg.orientation.z = stcOrien.q[3] / 32768.0;
      imu_msg.orientation_covariance[0] = -1;
      imu_msg.linear_acceleration.x = (float)stcAcc.a[0] / 32768.0 * 16.0 * 9.8;
      imu_msg.linear_acceleration.y = (float)stcAcc.a[1] / 32768.0 * 16.0 * 9.8;
      imu_msg.linear_acceleration.z = (float)stcAcc.a[2] / 32768.0 * 16.0 * 9.8;
      imu_msg.linear_acceleration_covariance[0] = -1;
      imu_msg.angular_velocity.x = ((float)stcGyro.w[0] / 32768.0 * 2000) / 180.0 * Pi;
      imu_msg.angular_velocity.y = ((float)stcGyro.w[1] / 32768.0 * 2000) / 180.0 * Pi;
      imu_msg.angular_velocity.z = ((float)stcGyro.w[2] / 32768.0 * 2000) / 180.0 * Pi;
      imu_msg.angular_velocity_covariance[0] = -1;
      imu_pub.publish(imu_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
