/**
 * @file /include/btn/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef btn_QNODE_HPP_
#define btn_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>  //add
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"//转换函数头文件
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace btn {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  void close();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

  //QByteArray sendBuf() {return send_buf;}
	QStringListModel* loggingModel() { return &logging_model; }
  QStringListModel* loggingModel_sub() { return &logging_model_sub; } //add
  void Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);  //add
  void sent_cmd();  //add
  int PoseXYZRPY2buffer(char* buf,std::string &msg);


Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void loggingUpdated_sub();  //add
  void poseUpdated();

private:
	int init_argc;
  const int num_log_rec = 20;
	char** init_argv;
  QStringListModel logging_model;
  ros::Subscriber chatter_subscriber; //add
  QStringListModel logging_model_sub; //add
  //QByteArray send_buf;
  geometry_msgs::Pose trackPose;

};

}  // namespace btn

#endif /* btn_QNODE_HPP_ */
