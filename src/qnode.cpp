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

    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    //cout<<"find position"<<endl;
    //ROS_INFO("pos.x:%lf",position_opti.x); //打印接受到的字符串
    trackPose = msg->pose;

    //PoseXYZRPY2buffer();
    Q_EMIT poseUpdated(); // used to readjust the scrollbar

}

int QNode::PoseXYZRPY2buffer(char* buf,std::string &msg)
{
  std::stringstream ss;
  std::string c_msg;
  int header_len = 3;
  int checksum_len = 2;
  int no_payload_len = header_len + checksum_len;
  int num_payload = 6;
  size_t sz_per_payload = sizeof(double);
  int len = num_payload*sz_per_payload+no_payload_len;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(trackPose.orientation, quat);

  double roll, pitch, yaw,roll_tmp,pitch_tmp,yaw_tmp;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll_tmp, pitch_tmp, yaw_tmp);//进行转换
  roll = (double)roll_tmp;
  pitch = (double)pitch_tmp;
  yaw = (double)yaw_tmp;



  buf[0] = 0xff;
  buf[1] = 0xfe;
  buf[2] = 0x00;
  memcpy(buf+header_len,&(trackPose.position.x),sz_per_payload);
  memcpy(buf+header_len+sz_per_payload,&(trackPose.position.y),sz_per_payload);
  memcpy(buf+header_len+2*sz_per_payload,&(trackPose.position.z),sz_per_payload);
  memcpy(buf+header_len+3*sz_per_payload,&(roll),sz_per_payload);
  memcpy(buf+header_len+4*sz_per_payload,&(pitch),sz_per_payload);
  memcpy(buf+header_len+5*sz_per_payload,&(yaw),sz_per_payload);
  buf[len-2] = 0x0d;
  buf[len-1] = 0x0a;

  ss<<"[Info] [" << ros::Time::now() << "]: pos x:"<<trackPose.position.x<<" y:"<<trackPose.position.y<<" z:"<<trackPose.position.z<<
      " r:"<<roll<<" p:"<<pitch<<" :y:"<<yaw;
  msg = ss.str();

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

}  // namespace btn
