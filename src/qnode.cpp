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
  chatter_subscriber = n.subscribe("/mavros/vision_pose/pose", 100, &QNode::Callback, this);  //add
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
  chatter_subscriber = n.subscribe("/mavros/vision_pose/pose", 100, &QNode::Callback, this);  //add
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


void QNode::log( const LogLevel &level, const std::string &msg) {
  if(logging_model.rowCount()>=num_log_rec)
    logging_model.removeRows(0,1);
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

//add
void QNode::log_sub( const LogLevel &level, const std::string &msg) {
  if(logging_model_sub.rowCount()>=num_log_rec)
    logging_model_sub.removeRows(0,1);
  logging_model_sub.insertRows(logging_model_sub.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_sub.setData(logging_model_sub.index(logging_model_sub.rowCount()-1),new_row);
  Q_EMIT loggingUpdated_sub(); // used to readjust the scrollbar
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

    //record and view
    log_sub(Info, std::string("Sub Pose: x:")+std::to_string(trackPose.position.x) + std::string(" y:")
            +std::to_string(trackPose.position.y)+std::string(" z:")+std::to_string(trackPose.position.z));
    PoseXYZRPY2buffer();
    Q_EMIT poseUpdated(); // used to readjust the scrollbar

}

void QNode::PoseXYZRPY2buffer()
{
  int header_len = 3;
  int checksum_len = 2;
  int no_payload_len = header_len + checksum_len;
  int num_payload = 6;
  size_t sz_per_payload = sizeof(double);
  int len = num_payload*sz_per_payload+no_payload_len;
  send_buf.clear();
  send_buf.resize(len);

  tf::Quaternion quat;
  tf::quaternionMsgToTF(trackPose.orientation, quat);

  double roll, pitch, yaw;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换


  send_buf[0] = 0xff;
  send_buf[1] = 0xfe;
  send_buf[3] = 0x00;
  memcpy(send_buf.data()+header_len,&(trackPose.position.x),sz_per_payload);
  memcpy(send_buf.data()+header_len+sz_per_payload,&(trackPose.position.y),sz_per_payload);
  memcpy(send_buf.data()+header_len+2*sz_per_payload,&(trackPose.position.z),sz_per_payload);
  memcpy(send_buf.data()+header_len+3*sz_per_payload,&(roll),sz_per_payload);
  memcpy(send_buf.data()+header_len+4*sz_per_payload,&(pitch),sz_per_payload);
  memcpy(send_buf.data()+header_len+5*sz_per_payload,&(yaw),sz_per_payload);
  send_buf[len-2] = 0x0d;
  send_buf[len-1] = 0x0a;
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
    log(Info, std::string("I sent:"+msg.data));
    ros::spinOnce();
  }
}

}  // namespace btn
