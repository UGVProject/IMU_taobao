/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "../imu.h"

serial::Serial ser;
static char rev_buf[64];
static imu_data data;
uint8_t fcs = 0;

// void write_callback(const std_msgs::String::ConstPtr& msg){
//     ROS_INFO_STREAM("Writing to serial port" << msg->data);
//     ser.write(msg->data);
// }

int main (int argc, char** argv){
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;
    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("magneticField/data_raw", 1000);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(38400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(80);

    while(ros::ok()){

      sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
      sensor_msgs::MagneticField mag_msg = sensor_msgs::MagneticField();

      imu_msg.header.stamp = ros::Time::now();
      mag_msg.header.stamp = imu_msg.header.stamp;
      imu_msg.header.frame_id = "imu";
      mag_msg.header.frame_id = "magneticField";

      if(ser.available()){
          // ROS_INFO_STREAM("Reading from serial port");
          std_msgs::String result;
          std::size_t find0,find1;
          // imu_get_data(&imu);
          // printf("P/R/Y:%2.2f %2.2f %2.2f\r\n", ((float)imu.pitch)/100, ((float)imu.roll)/100, ((float)imu.yaw)/10);

          result.data = ser.read(ser.available());
          // strcpy(rev_buf, result.data.c_str());
          find0 = result.data.find((char)0x88);
          find1 = result.data.find((char)0xAF);
          if( (find0!=std::string::npos) && (find1!=std::string::npos) && (find0 +1 == find1) )
          {
            std::size_t length1 = result.data.copy(rev_buf,32,find0);

            // ROS_INFO_STREAM("length is: " << (int)rev_buf[2] );
            data.accl[0] = (rev_buf[3]<<8) + rev_buf[4];
            data.accl[1] = (rev_buf[5]<<8) + rev_buf[6];
            data.accl[2] = (rev_buf[7]<<8) + rev_buf[8];

            data.gyro[0] = (rev_buf[9]<<8) + rev_buf[10];
            data.gyro[1] = (rev_buf[11]<<8) + rev_buf[12];
            data.gyro[2] = (rev_buf[13]<<8) + rev_buf[14];

            data.mag[0] = (rev_buf[15]<<8) + rev_buf[16];
            data.mag[1] = (rev_buf[17]<<8) + rev_buf[18];
            data.mag[2] = (rev_buf[19]<<8) + rev_buf[20];

            data.roll = (rev_buf[21]<<8) + rev_buf[22];
            data.pitch = (rev_buf[23]<<8) + rev_buf[24];
            data.yaw = (rev_buf[25]<<8) + rev_buf[26];

            fcs = 0x88;
            fcs += 0xAF;
            fcs += 28;
            for(int i=3; i<30; i++)
            {
                fcs += rev_buf[i];
            }
            if(fcs == (uint8_t)rev_buf[31])
            {
              // ROS_INFO_STREAM("haha!" );
            }
            else{
              ROS_INFO_STREAM("a: " << (int)fcs << " b: " << (int)rev_buf[31] );
            }
            //
            // ROS_INFO_STREAM("Read: pitch: " << (float)data.pitch/100 << " roll: " << (float)data.roll/100 << " yaw: " << (float)data.yaw/10);

          }
          else{
            ROS_INFO_STREAM("Header wrong!");
          }

          imu_msg.orientation.x = (float)data.pitch*pi/18000;
          imu_msg.orientation.y = (float)data.roll*pi/18000;
          imu_msg.orientation.z = (float)data.yaw*pi/1800;
          imu_msg.orientation.w = 0.0;
          //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
          imu_msg.angular_velocity.x = (float)data.gyro[0]*0.061035*pi/180;
          imu_msg.angular_velocity.y = (float)data.gyro[1]*0.061035*pi/180;
          imu_msg.angular_velocity.z = (float)data.gyro[2]*0.061035*pi/180;
          //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
          imu_msg.linear_acceleration.x = (float)data.accl[0]*0.002385305;
          imu_msg.linear_acceleration.y = (float)data.accl[1]*0.002385305;
          imu_msg.linear_acceleration.z = (float)data.accl[2]*0.002385305;
          //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
          mag_msg.magnetic_field.x = (float)data.mag[0]*6/10000;
          mag_msg.magnetic_field.y = (float)data.mag[1]*6/10000;
          mag_msg.magnetic_field.z = (float)data.mag[2]*6/10000;
          imu_pub.publish(imu_msg);
          mag_pub.publish(mag_msg);
      }

      ros::spinOnce();
      loop_rate.sleep();

    }
}
