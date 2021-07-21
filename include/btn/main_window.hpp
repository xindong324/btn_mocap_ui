/**
 * @file /include/btn/main_window.hpp
 *
 * @brief Qt based gui for btn.
 *
 * @date November 2010
 **/
#ifndef btn_MAIN_WINDOW_H
#define btn_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include <QMainWindow>
#include <QTranslator>
#include <string>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>//不是QTime
#include <QTime>
#include <QTextCodec>
#include <QSettings>
#include <thread>
#include <QLabel>
#include <QMessageBox>
#include <QDir>
#include <QSpinBox>
#include <QPalette>



#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace btn {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  void getComboBoxValue();
  void iniPort();


	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    //void updateLoggingView(); // no idea why this can't connect automatically
    //void updateLoggingView_sub(); //add
    void serialSendMocapData();
    void pub_cmd(); //add

private slots:
    void on_btn_choosefile_clicked();

    void on_btn_loadparam_clicked();

    void on_cb_boadrate_currentIndexChanged(int index);

    void on_send_gain_clicked();

    void on_tb_pub_textChanged();

    void SerialRead();
    void SerialWrite();

    void on_btn_stopuav_clicked();

    void on_ck_sendmocap_stateChanged(int arg1);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

  float fGain[30];  // gain of uav

  QString m_port;
  QString m_baudrate;
  QString m_databit;
  QString m_check;
  QString m_stopbit;
  QString m_FlowControl ;

  QString gain_file_name;
  bool is_file_name_set=false;


  //ini类
  QSettings* configini;
  //串口类
  QSerialPort* SPort;

  bool runonce;
  //定时器
  //QTimer * timer;
  //显示当前那时间
  QTime * currenttime;
  bool DisplayTimeStatus;
  //时间间隔time_cycle ms发送一次
 /*QSpinBox * gettime;*/
  int time_cycle;
  //ini路径
  QString PATH;
  QDir * currentPath;
  QPalette palette;

private:
  bool setPortConfig();
  void setcurrentPath();

  // config
  void config();
  void configiniPortRead();
  void configiniPortWrite();

  void configiniGainRead();
  void configiniGainWrite();


  // send data to uav
  void send_gain();
  int send_stop(char *buf,std::string &msg);

};

}  // namespace btn

#endif // btn_MAIN_WINDOW_H
