/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/btn/qnode.hpp"

#include "../include/btn/inc2/mavlink_helpers.h"
#include "../include/btn/inc2/common/mavlink.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace btn {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"btn");
	if ( ! ros::master::check() ) {
		return false;
	}
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  chatter_subscriber = n.subscribe("/mavros/mocap/pose", 10, &QNode::Callback, this);  //add
  start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
  ros::init(remappings,"btn");
	if ( ! ros::master::check() ) {
		return false;
	}
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  chatter_subscriber = n.subscribe("/mavros/mocap/pose", 100, &QNode::Callback, this);  //add
  start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(120);
	int count = 0;
	while ( ros::ok() ) {
    /*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
    */
		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::close()
{
  ros::shutdown();
  //Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}



//add
void QNode::Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::PoseStamped submsg=*msg;
    geometry_msgs::Quaternion quat_opti = msg->pose.orientation;
    // convert quat 2 yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quat_opti, quat);

    double roll, pitch, yaw;//????????????r\p\y?????????
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//????????????
    //cout<<"find position"<<endl;
    //ROS_INFO("pos.x:%lf",position_opti.x); //???????????????????????????
    trackPose = msg->pose;

    //PoseXYZRPY2buffer();
    Q_EMIT poseUpdated(); // used to readjust the scrollbar

}

int QNode::PoseXYZRPY2buffer(uint8_t* buf,float* pose)
{//24+5
  std::stringstream ss;
  std::string c_msg;
  int header_len = 3;
  int checksum_len = 2;
  int no_payload_len = header_len + checksum_len;
  int num_payload = 6;
  size_t sz_per_payload = sizeof(float);
  int len = num_payload*static_cast<int>(sz_per_payload)+no_payload_len;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(trackPose.orientation, quat);

  double roll_tmp,pitch_tmp,yaw_tmp;
  float x,y,z,roll, pitch, yaw;//????????????r\p\y?????????
  tf::Matrix3x3(quat).getRPY(roll_tmp, pitch_tmp, yaw_tmp);//????????????

  pose[0] = x = static_cast<float>(trackPose.position.x);
  pose[1] =y = static_cast<float>(trackPose.position.y);
  pose[2] =z = static_cast<float>(trackPose.position.z);
  pose[3] =roll = static_cast<float>(roll_tmp);
  pose[4] =pitch = static_cast<float>(pitch_tmp);
  pose[5] =yaw = static_cast<float>(yaw_tmp);

  pose[3] *= 57.29578f;
  pose[4] *= 57.29578f;
  pose[5] *= 57.29578f;

  buf[0] = 0xff;
  buf[1] = 0xfe;
  buf[2] = 0x00;
  memcpy(buf+header_len,&(x),sz_per_payload);
  memcpy(buf+header_len+sz_per_payload,&(y),sz_per_payload);
  memcpy(buf+header_len+2*sz_per_payload,&(z),sz_per_payload);
  memcpy(buf+header_len+3*sz_per_payload,&(roll),sz_per_payload);
  memcpy(buf+header_len+4*sz_per_payload,&(pitch),sz_per_payload);
  memcpy(buf+header_len+5*sz_per_payload,&(yaw),sz_per_payload);
  buf[len-2] = 0x0d;
  buf[len-1] = 0x0a;

  ss<<"[Info] [" << ros::Time::now() << "]: pos x:"<<mocap_pos_send.x<<" y:"<<mocap_pos_send.y<<" z:"<<mocap_pos_send.z<<
      " r:"<<mocap_pos_send.roll<<" p:"<<mocap_pos_send.pitch<<" :y:"<<mocap_pos_send.yaw;
  ROS_INFO_STREAM(ss.str());

  return len;
}

int QNode::PoseXYZRPY2bufferMavlink(uint8_t* buf, float* pose)
{
  std::stringstream ss;
  std::string c_msg;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(trackPose.orientation, quat);
  double roll, pitch, yaw;//????????????r\p\y?????????
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//????????????

  pose[0] = mocap_pos_send.x = static_cast<float>(trackPose.position.x);
  pose[1] = mocap_pos_send.y = static_cast<float>(trackPose.position.y);
  pose[2] = mocap_pos_send.z = static_cast<float>(trackPose.position.z);


  pose[3] = mocap_pos_send.roll = static_cast<float>(roll);
  pose[4] = mocap_pos_send.pitch = static_cast<float>(pitch);
  pose[5] = mocap_pos_send.yaw =  static_cast<float>(yaw);

  pose[3] *= 57.29578f;
  pose[4] *= 57.29578f;
  pose[5] *= 57.29578f;

  mavlink_msg_vicon_position_estimate_encode(9,201,&mav_msg,&mocap_pos_send);
  int len = mavlink_msg_to_send_buffer(buf, &mav_msg);

  ss<<"[Info] [" << ros::Time::now() << "]: pos x:"<<mocap_pos_send.x<<" y:"<<mocap_pos_send.y<<" z:"<<mocap_pos_send.z<<
      " r:"<<mocap_pos_send.roll<<" p:"<<mocap_pos_send.pitch<<" :y:"<<mocap_pos_send.yaw;
  ROS_INFO_STREAM(ss.str());
  return len;
}

//add
void QNode::sent_cmd()
{
  if(ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "clicked the button";
    msg.data = ss.str();
    //chatter_publisher.publish(msg);
    //log(Info, std::string("I sent:"+msg.data));
    ros::spinOnce();
  }
}

void QNode::log_info(std::string &msg)
{
  ROS_INFO_STREAM(msg);
}

}  // namespace btn
