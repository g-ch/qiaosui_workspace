//*******receiver.cpp**********

#include "receiver.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include "../../../devel/include/mavros/State.h"
#include "../../../devel/include/mavros/Mavlink.h"
#include "../../../devel/include/mavros/BatteryStatus.h"
#include "../../../devel/include/mavros/RadioStatus.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/TimeReference.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TwistStamped.h"
#include "../../../devel/include/mavros_extras/OpticalFlowRad.h"
#include "ros/time.h"
#include <mavros_extras/SonarDistance.h>
#include <mavros_extras/LaserDistance.h>

extern MavrosMessage message;

void chatterCallback_Mode(const mavros::State &msg);
void chatterCallback_Mavlink(const mavros::Mavlink &msg);
void chatterCallback_GlobalPosition_RelAlt(const std_msgs::Float64 &msg);
void chatterCallback_LocalPosition_Local(const geometry_msgs::PoseStamped &msg);
void chatterCallback_WindEstimation(const geometry_msgs::TwistStamped &msg);
void chatterCallback_Battery(const mavros::BatteryStatus &msg);
void chatterCallback_RadioStatus(const mavros::RadioStatus &msg);
void chatterCallback_GPS_Fix(const sensor_msgs::NavSatFix &msg);
void chatterCallback_Imu_Temperature(const sensor_msgs::Temperature &msg);
void chatterCallback_Sonar(const mavros_extras::SonarDistance &msg);
void chatterCallback_Laser(const mavros_extras::LaserDistance &msg);
void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg);

void chatterCallback_Mode(const mavros::State &msg)//模式
{  
  //cout<<msg.mode;
  if(msg.mode=="MANUAL")message.mode="手动模式";
  else if(msg.mode=="ALTCTRL")message.mode="高度控制";
  else if(msg.mode=="POSCTRL")message.mode="位置控制";
  else if(msg.mode=="AUTOCTRL")message.mode="自动模式";
  else if(msg.mode=="LOITER")message.mode="悬停模式";
  else message.mode=msg.mode;
  message.msg_Send_State();
}  
void chatterCallback_Mavlink(const mavros::Mavlink &msg)//mavlink原始信息
{
    if(msg.msgid==24)
       {
        //cout<<msg<<endl;
        message.global_position.gps.satellites=((msg.payload64[3]>>40)&0xFF);
       // cout<<message.global_position.gps.satellites<<endl;
       }
    //if(msg.msgid==105)cout<<msg<<endl;
    if(msg.msgid==212||msg.msgid==213)cout<<msg<<endl;
    message.msg_Send_GPS_Satellites();
}
void chatterCallback_Imu_Data(const sensor_msgs::Imu &msg)//传感器信息
{

}
void chatterCallback_Imu_Temperature(const sensor_msgs::Temperature &msg)//飞控温度
{
    message.temperature=msg.temperature;
    message.msg_Send_Temperature();
}
void chatterCallback_GlobalPosition_RelAlt(const std_msgs::Float64 &msg)//相对高度
{
    //cout<<"A"<<msg.data<<endl;
    message.global_position.rel_altitude=msg.data;
    message.msg_Send_Rel_Alt();
}

void chatterCallback_GlobalPosition_GpVel(const geometry_msgs::TwistStamped &msg)//速度
{
    //cout<<"V"<<msg.vector<<endl;
    message.global_position.vel.x =msg.twist.linear.x;
    message.global_position.vel.y =msg.twist.linear.y;
    message.global_position.vel.z =msg.twist.linear.z;
    message.msg_Send_Velocity();
}
void chatterCallback_LocalPosition_Local(const geometry_msgs::PoseStamped &msg)//当地信息
{
    double q2=msg.pose.orientation.x;
    double q1=msg.pose.orientation.y;
    double q0=msg.pose.orientation.z;
    double q3=msg.pose.orientation.w;
    //cout<<"Speed "<<msg.pose.position<<endl;
    //cout<<" ** "<<q0<<endl<<q1<<endl<<q2<<endl<<q3<<endl;
    message.local_position.orientation.pitch = asin(2*q0*q2-2*q1*q3 );//弧度制
    message.local_position.orientation.roll  = atan2(2*q2*q3 + 2*q0*q1, 1-2*q1*q1-2*q2*q2);
    message.local_position.orientation.yaw = -atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1);
    message.local_position.orientation.pitchd=message.local_position.orientation.pitch*57.3;//换为角度制
    message.local_position.orientation.rolld=message.local_position.orientation.roll*57.3;
    message.local_position.orientation.yawd=message.local_position.orientation.yaw*57.3;
    //cout<<"pitch "<<message.local_position.orientation.pitchd<<endl;
    //cout<<"roll "<<message.local_position.orientation.rolld<<endl;
    //cout<<"yaw "<<message.local_position.orientation.yawd<<endl;
    //message.local_position.speed.x=msg.pose.position.x;
    //message.local_position.speed.y=msg.pose.position.y;
    //message.local_position.speed.z=msg.pose.position.z;
    message.global_position.rel_altitude=msg.pose.position.z;
    message.msg_Send_Rel_Alt();
    message.msg_Send_Orientation();
}
void chatterCallback_WindEstimation(const geometry_msgs::TwistStamped &msg)//风速信息
{
    //cout<<"WIND"<<msg.twist<<endl;
    message.wind_speed.linear.x=msg.twist.linear.x;
    message.wind_speed.linear.y=msg.twist.linear.y;
    message.wind_speed.linear.z=msg.twist.linear.z;
    message.wind_speed.angular.x=msg.twist.angular.x;
    message.wind_speed.angular.y=msg.twist.angular.y;
    message.wind_speed.angular.z=msg.twist.angular.z;
}
void chatterCallback_Battery(const mavros::BatteryStatus &msg)//电量信息
{
    //cout<<"Battery"<<msg.voltage<<endl;
    message.battery_voltage=msg.voltage;
    message.msg_Send_Battery();
}
void chatterCallback_RadioStatus(const mavros::RadioStatus &msg)//信号质量
{
    //cout<<"Signal Strength"<<msg.rssi_dbm<<endl;
    message.radio_rssi=msg.rssi_dbm;
    message.msg_Send_Radio();
}
void chatterCallback_GPS_Fix(const sensor_msgs::NavSatFix &msg)//GPS
{
    //cout<<msg<<endl;
    message.global_position.gps.status=msg.status.status;
    message.global_position.gps.x=msg.latitude;
    message.global_position.gps.y=msg.longitude;
    message.global_position.gps.z=msg.altitude;
    message.msg_Send_GPS();
}
void chatterCallback_Optical_Flow(const mavros_extras::OpticalFlowRad &msg)//光流
{
    message.optical_flow.quality=msg.quality;
    message.optical_flow.distance=msg.distance;
    message.optical_flow.temperature=msg.temperature;
    //cout<<msg;
    message.msg_Send_Optical_Flow();
}
void chatterCallback_Time(const sensor_msgs::TimeReference &msg)//飞行时间
{
    //待改动
    message.time_fromboost=0.0;
    //cout<<"source:"<<msg.source<<endl;
    //cout<<"reference:"<<msg.time_ref<<endl;
    message.msg_Send_Time();
}

void chatterCallback_Sonar(const mavros_extras::SonarDistance &msg)
{
    message.optical_flow.distance=msg.sonar_up/100.0; //changed to sonar msg, clarence, 2015.12.2
    message.optical_flow.temperature=0;
    //cout<<msg;
    message.msg_Send_Optical_Flow();
}

void chatterCallback_Laser(const mavros_extras::LaserDistance &msg)
{
    message.optical_flow.quality=msg.min_distance/100.0;
    message.msg_Send_Optical_Flow();
}

void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg)
{
    message.local_position.speed.x=msg.x;
    message.local_position.speed.y=msg.y;
    message.local_position.speed.z=msg.z;
    message.msg_Send_Orientation();
}

void MavrosMessage::run()
{
    //接收以下话题
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/mavros/state", 1000,chatterCallback_Mode);
    ros::Subscriber sub2 = n.subscribe("/mavlink/from", 1000,chatterCallback_Mavlink);
    ros::Subscriber sub3 = n.subscribe("/mavros/imu/data", 1000,chatterCallback_Imu_Data);
    ros::Subscriber sub4 = n.subscribe("/mavros/global_position/rel_alt", 1000,chatterCallback_GlobalPosition_RelAlt);
    ros::Subscriber sub5 = n.subscribe("/mavros/global_position/raw/gps_vel", 1000,chatterCallback_GlobalPosition_GpVel);
    ros::Subscriber sub6 = n.subscribe("/mavros/local_position/local", 1000,chatterCallback_LocalPosition_Local);
    ros::Subscriber sub7 = n.subscribe("/mavros/wind_estimation",1000,chatterCallback_WindEstimation);
    ros::Subscriber sub8 = n.subscribe("/mavros/battery",1000,chatterCallback_Battery);
    ros::Subscriber sub9 = n.subscribe("/mavros/radio_status",200,chatterCallback_RadioStatus);
    ros::Subscriber sub10 = n.subscribe("/mavros/global_position/raw/fix",100,chatterCallback_GPS_Fix);
    ros::Subscriber sub11 =n.subscribe("/mavros/px4flow/raw/optical_flow_rad",1000,chatterCallback_Optical_Flow);
    ros::Subscriber sub12 =n.subscribe("/mavros/imu/temperature",100,chatterCallback_Imu_Temperature);
    ros::Subscriber sub13 =n.subscribe("/mavros/time_reference",500,chatterCallback_Time);
    ros::Subscriber sub14 =n.subscribe("/mavros/sonar_receiver/sonar_receiver",500,chatterCallback_Sonar);
    ros::Subscriber sub15 =n.subscribe("/mavros/laser_receiver/laser_receiver",500,chatterCallback_Laser);
    ros::Subscriber sub16 =n.subscribe("/mavros/local_position/local_velocity",500,chatterCallback_local_velocity);
    //循环执行
    ros::spin();
}



