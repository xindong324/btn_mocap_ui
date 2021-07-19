/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QThread>
#include <QtSerialPort>

#include <QFileDialog>


#include "../include/btn/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace btn {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
  //ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    ui.view_logging_sub->setModel(qnode.loggingModel_sub());  //add
      QObject::connect(&qnode, SIGNAL(loggingUpdated_sub()), this, SLOT(updateLoggingView_sub()));  //add
    //QObject::connect(ui.sent_cmd, SIGNAL(clicked()), this, SLOT(pub_cmd()));
      QObject::connect(&qnode,SIGNAL(poseUpdated()),this,SLOT(serialSendMocapData()));


      /*********************
      ** Port
      **********************/
      SPort = new QSerialPort(this);
      iniPort();
    /*********************
    ** Auto Start
    **********************/

    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow()
{
  delete SPort;
}


//初始化
void MainWindow::iniPort()
{

    QList<QSerialPortInfo>  infos = QSerialPortInfo::availablePorts();
    if(infos.isEmpty())
    {
        ui.cb_port->addItem(tr("Empty"));
        return;
    }
    foreach (QSerialPortInfo info, infos) {
        ui.cb_port->addItem(info.portName());
    }
    //ini配置文件
    //config();
    //configiniRead();
    //默认选中
    //ui->radio_accept_hex->setChecked(true);
    //ui->radio_send_ascii->setChecked(true);
    //ui->check_auto_line->setChecked(true);
    //ui->check_repsend->setChecked(true);
    //设置只读
    //ui->text_accept->setReadOnly(true);
}
void MainWindow::setcurrentPath()
{
    currentPath = new QDir;
    PATH = currentPath->currentPath() + "/SerialPort.ini";
}
void MainWindow::getComboBoxValue()
{
    m_port = ui.cb_port->currentText();
    m_baudrate = ui.cb_boadrate->currentText();
    m_databit = ui.cb_data->currentText();
    m_check = ui.cb_check->currentText();
    m_stopbit = ui.cb_stop->currentText();

}

//设置串口参数
void MainWindow::setPortConfig()
{


    //设置串口号
    SPort->setPortName(m_port);
    if(SPort->open(QIODevice::ReadWrite))
    {
        //设置波特率
        SPort->setBaudRate(m_baudrate.toInt());
        //设置数据位
        switch(m_databit.toInt())
        {
            case 5:
                 SPort->setDataBits(QSerialPort::Data5);break;
            case 6:
                 SPort->setDataBits(QSerialPort::Data6);break;
            case 7:
                 SPort->setDataBits(QSerialPort::Data7);break;
            case 8:
                 SPort->setDataBits(QSerialPort::Data8);break;
            default: break;
        }
        //设置校验位
        switch(ui.cb_check->currentIndex())
        {
            case 0:
                SPort->setParity(QSerialPort::NoParity);break;
            case 1:
                SPort->setParity(QSerialPort::EvenParity);break;
            case 2:
                SPort->setParity(QSerialPort::OddParity);break;
            case 3:
                SPort->setParity(QSerialPort::SpaceParity);break;
            case 4:
                SPort->setParity(QSerialPort::MarkParity);break;
            default: break;
        }
        //设置流控制

        SPort->setFlowControl(QSerialPort::NoFlowControl);


        //设置停止位
        switch(m_stopbit.toInt())
        {
            case 1:
                SPort->setStopBits(QSerialPort::OneStop);
            case 2:
                SPort->setStopBits(QSerialPort::TwoStop);
            default: break;
        }

        //message("config 成功\r\n");
    }
    else{
        QMessageBox::warning(this,tr("warning"),tr("initialization config failed!"));
       // message("config 失败\r\n");
    }


}

void MainWindow::serialSendMocapData()
{
  QByteArray arr = qnode.sendBuf();
  SPort->write(arr);
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
  /*********************
  ** Launch ros
  **********************/
  if ( ui.checkbox_use_environment->isChecked() ) { // use pc env
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
  }
  else {// user defined env
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}

  /*********************
  ** Open port
  **********************/
  //写配置信息
  //configiniWrite();

  getComboBoxValue();
  setPortConfig();
  ui.cb_port->setEnabled(false);
  ui.cb_data->setEnabled(false);
  ui.cb_stop->setEnabled(false);
  ui.cb_check->setEnabled(false);
  ui.cb_boadrate->setEnabled(false);
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        //ui.view_logging->scrollToBottom();
}
//add
void MainWindow::updateLoggingView_sub() {
        ui.view_logging_sub->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "btn");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "btn");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::pub_cmd()
{
  qnode.sent_cmd();
}

}  // namespace btn


void btn::MainWindow::on_btn_choosefile_clicked()
{
  ui.edt_filepath->setReadOnly(false);
  gain_file_name = QFileDialog::getOpenFileName(this,tr("文件"),"",tr("text(*.txt)"));
  is_file_name_set = true;
  ui.edt_filepath->setText(gain_file_name);
  ui.edt_filepath->setReadOnly(true); // set read only
}

void btn::MainWindow::on_btn_loadparam_clicked()
{

}

void btn::MainWindow::on_cb_boadrate_currentIndexChanged(int index)
{

}
