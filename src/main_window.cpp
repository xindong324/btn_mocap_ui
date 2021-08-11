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
#include <qvalidator.h>


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
    //QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  ui.tb_pub->document()->setMaximumBlockCount(200);
  //ui.tb_msg_rcv->document()->setMaximumBlockCount(200); // max row of rcv msg is 200
  /*********************
	** Logging
	**********************/
  //ui.view_logging->setModel(qnode.loggingModel());
    //QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    //ui.view_logging_sub->setModel(qnode.loggingModel_sub());  //add
      //QObject::connect(&qnode, SIGNAL(loggingUpdated_sub()), this, SLOT(updateLoggingView_sub()));  //add
    //QObject::connect(ui.sent_cmd, SIGNAL(clicked()), this, SLOT(pub_cmd()));

  /*********************
  ** send pose
  **********************/
      QObject::connect(&qnode,SIGNAL(poseUpdated()),this,SLOT(serialSendMocapData()));


      /*********************
      ** Port
      **********************/
      SPort = new QSerialPort(this);
      iniPort();
      runonce = true;
    /*********************
    ** Auto Start
    **********************/

    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    /*********************
    ** set text editor
    **********************/
    ui.edt_k01->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k02->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k03->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k04->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k05->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k06->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k07->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k08->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k09->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k10->setValidator(new QDoubleValidator(-180.0,180.0,4,this));

    ui.edt_k11->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k12->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k13->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k14->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k15->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k16->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k17->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k18->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k19->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k20->setValidator(new QDoubleValidator(-180.0,180.0,4,this));

    ui.edt_k21->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k22->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k23->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k24->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k25->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k26->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k27->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k28->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k29->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
    ui.edt_k30->setValidator(new QDoubleValidator(-180.0,180.0,4,this));
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
    config();
    configiniPortRead();
    configiniGainRead();
    //默认选中
    ui.send_gain->setEnabled(false);
    ui.btn_stopuav->setEnabled(false);

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
bool MainWindow::setPortConfig()
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
                 break;
            case 2:
                SPort->setStopBits(QSerialPort::TwoStop);
                break;
            default: break;
        }

        //message("config 成功\r\n");
    }
    else{
        QMessageBox::warning(this,tr("warning"),tr("initialization Port failed! Maybe used by others"));
        return false;
    }
    return true;


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

  if(tr("Connect")==ui.button_connect->text())
  { // click connect
    /*********************
    ** Open port
    **********************/
    //写配置信息
    configiniPortWrite();

    getComboBoxValue();
    if(!setPortConfig())
    { // port err

    } // end port err
    else { // port open
      /*********************
      ** Launch ros
      **********************/
      if ( ui.checkbox_use_environment->isChecked() ) { // use pc env
        if ( !qnode.init() ) {
          showNoMasterMessage();
        } else {
          //ui.button_connect->setEnabled(false);
        }
      }//end pc env
      else {// user defined env
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
               ui.line_edit_host->text().toStdString()) ) {
          showNoMasterMessage();
        } else {
          //ui.button_connect->setEnabled(false);
          ui.line_edit_master->setReadOnly(true);
          ui.line_edit_host->setReadOnly(true);
        }
      }// end user def env


      //收到数据运行槽函数
      if(runonce)//只允许运行一次
      {
         connect(SPort,SIGNAL(readyRead()),this,SLOT(SerialRead()));
      }

      ui.cb_port->setEnabled(false);
      ui.cb_data->setEnabled(false);
      ui.cb_stop->setEnabled(false);
      ui.cb_check->setEnabled(false);
      ui.cb_boadrate->setEnabled(false);

      ui.send_gain->setEnabled(true);
      ui.btn_stopuav->setEnabled(true);

      ui.quit_button->setEnabled(false);
      ui.button_connect->setText(tr("Disconnect"));
    }// end port open

  }// end click connect
  else
  {// disconnect clicked
    std::cout<<"bf"<<std::endl;
    ui.button_connect->setText(tr("Connect"));
    //qnode.rosShutdown();
    runonce = false;
    qnode.close();
    SPort->close();
    std::cout<<"aft"<<std::endl;
    ui.cb_port->setEnabled(true);
    ui.cb_data->setEnabled(true);
    ui.cb_stop->setEnabled(true);
    ui.cb_check->setEnabled(true);
    ui.cb_boadrate->setEnabled(true);
    ui.send_gain->setEnabled(false);
    ui.btn_stopuav->setEnabled(false);

    ui.quit_button->setEnabled(true);
  }

}

void MainWindow::SerialRead()
{

}

void MainWindow::SerialWrite()
{

}

//获取当前路径并创建ini对象
void MainWindow::config()
{
    setcurrentPath();
    configini = new QSettings(PATH, QSettings::IniFormat);
}
//从ini文件读取并设置为上次配置
void MainWindow::configiniPortRead()
{
    configini->beginGroup("SETUP");
    int i_m_port = configini->value("COM").toInt();
    int i_m_baudrate = configini->value("baudrate").toInt();
    int i_m_databit = configini->value("databit").toInt();
    int i_m_check = configini->value("check").toInt();
    int i_m_stopbit = configini->value("stopbit").toInt();
    //int i_m_FlowControl = configini->value("flow").toInt();
    configini->endGroup();

    ui.cb_port->setCurrentIndex(i_m_port);
    ui.cb_boadrate->setCurrentIndex(i_m_baudrate);
    ui.cb_data->setCurrentIndex(i_m_databit);
    ui.cb_check->setCurrentIndex(i_m_check);
    ui.cb_stop->setCurrentIndex(i_m_stopbit);
    //ui->cb_flow->setCurrentIndex(i_m_FlowControl);
}
//将配置信息写入ini文件
void MainWindow::configiniPortWrite()
{
    configini->beginGroup("SETUP");
    configini->setValue("COM",ui.cb_port->currentIndex());
    configini->setValue("baudrate",ui.cb_boadrate->currentIndex());
    configini->setValue("databit",ui.cb_data->currentIndex());
    configini->setValue("check",ui.cb_check->currentIndex());
    configini->setValue("stopbit",ui.cb_stop->currentIndex());
    //configini->setValue("flow",ui->CB_flow->currentIndex());
    configini->endGroup();
}

//从ini文件读取并设置为上次配置
void MainWindow::configiniGainRead()
{
    configini->beginGroup("GAIN");
    // TODO: use for to set string K%s value to set the cofig value into float array
    float k1 = configini->value("K1").toFloat();
    float k2 = configini->value("K2").toFloat();
    float k3 = configini->value("K3").toFloat();
    float k4 = configini->value("K4").toFloat();
    float k5 = configini->value("K5").toFloat();
    float k6 = configini->value("K6").toFloat();
    float k7 = configini->value("K7").toFloat();
    float k8 = configini->value("K8").toFloat();
    float k9 = configini->value("K9").toFloat();
    float k10 = configini->value("K10").toFloat();

    float k11 = configini->value("K11").toFloat();
    float k12 = configini->value("K12").toFloat();
    float k13 = configini->value("K13").toFloat();
    float k14 = configini->value("K14").toFloat();
    float k15 = configini->value("K15").toFloat();
    float k16 = configini->value("K16").toFloat();
    float k17 = configini->value("K17").toFloat();
    float k18 = configini->value("K18").toFloat();
    float k19 = configini->value("K19").toFloat();
    float k20 = configini->value("K20").toFloat();

    float k21 = configini->value("K21").toFloat();
    float k22 = configini->value("K22").toFloat();
    float k23 = configini->value("K23").toFloat();
    float k24 = configini->value("K24").toFloat();
    float k25 = configini->value("K25").toFloat();
    float k26 = configini->value("K26").toFloat();
    float k27 = configini->value("K27").toFloat();
    float k28 = configini->value("K28").toFloat();
    float k29 = configini->value("K29").toFloat();
    float k30 = configini->value("K30").toFloat();

    configini->endGroup();

    ui.edt_k01->setText(QString("%1").arg(k1));
    ui.edt_k02->setText(QString("%1").arg(k2));
    ui.edt_k03->setText(QString("%1").arg(k3));
    ui.edt_k04->setText(QString("%1").arg(k4));
    ui.edt_k05->setText(QString("%1").arg(k5));
    ui.edt_k06->setText(QString("%1").arg(k6));
    ui.edt_k07->setText(QString("%1").arg(k7));
    ui.edt_k08->setText(QString("%1").arg(k8));
    ui.edt_k09->setText(QString("%1").arg(k9));
    ui.edt_k10->setText(QString("%1").arg(k10));

    ui.edt_k11->setText(QString("%1").arg(k11));
    ui.edt_k12->setText(QString("%1").arg(k12));
    ui.edt_k13->setText(QString("%1").arg(k13));
    ui.edt_k14->setText(QString("%1").arg(k14));
    ui.edt_k15->setText(QString("%1").arg(k15));
    ui.edt_k16->setText(QString("%1").arg(k16));
    ui.edt_k17->setText(QString("%1").arg(k17));
    ui.edt_k18->setText(QString("%1").arg(k18));
    ui.edt_k19->setText(QString("%1").arg(k19));
    ui.edt_k20->setText(QString("%1").arg(k20));

    ui.edt_k21->setText(QString("%1").arg(k21));
    ui.edt_k22->setText(QString("%1").arg(k22));
    ui.edt_k23->setText(QString("%1").arg(k23));
    ui.edt_k24->setText(QString("%1").arg(k24));
    ui.edt_k25->setText(QString("%1").arg(k25));
    ui.edt_k26->setText(QString("%1").arg(k26));
    ui.edt_k27->setText(QString("%1").arg(k27));
    ui.edt_k28->setText(QString("%1").arg(k28));
    ui.edt_k29->setText(QString("%1").arg(k29));
    ui.edt_k30->setText(QString("%1").arg(k30));
}
//将配置信息写入ini文件
void MainWindow::configiniGainWrite()
{
    configini->beginGroup("GAIN");
    configini->setValue("K1",ui.edt_k01->text());
    configini->setValue("K2",ui.edt_k02->text());
    configini->setValue("K3",ui.edt_k03->text());
    configini->setValue("K4",ui.edt_k04->text());
    configini->setValue("K5",ui.edt_k05->text());
    configini->setValue("K6",ui.edt_k06->text());
    configini->setValue("K7",ui.edt_k07->text());
    configini->setValue("K8",ui.edt_k08->text());
    configini->setValue("K9",ui.edt_k09->text());
    configini->setValue("K10",ui.edt_k10->text());

    configini->setValue("K11",ui.edt_k11->text());
    configini->setValue("K12",ui.edt_k12->text());
    configini->setValue("K13",ui.edt_k13->text());
    configini->setValue("K14",ui.edt_k14->text());
    configini->setValue("K15",ui.edt_k15->text());
    configini->setValue("K16",ui.edt_k16->text());
    configini->setValue("K17",ui.edt_k17->text());
    configini->setValue("K18",ui.edt_k18->text());
    configini->setValue("K19",ui.edt_k19->text());
    configini->setValue("K20",ui.edt_k20->text());

    configini->setValue("K21",ui.edt_k21->text());
    configini->setValue("K22",ui.edt_k22->text());
    configini->setValue("K23",ui.edt_k23->text());
    configini->setValue("K24",ui.edt_k24->text());
    configini->setValue("K25",ui.edt_k25->text());
    configini->setValue("K26",ui.edt_k26->text());
    configini->setValue("K27",ui.edt_k27->text());
    configini->setValue("K28",ui.edt_k28->text());
    configini->setValue("K29",ui.edt_k29->text());
    configini->setValue("K30",ui.edt_k30->text());
    configini->endGroup();
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
//void MainWindow::updateLoggingView() {
//        //ui.view_logging->scrollToBottom();
//}
////add
//void MainWindow::updateLoggingView_sub() {
//        ui.view_logging_sub->scrollToBottom();
//}

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
    //restoreState(settings.value("windowState").toByteArray());
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


/*****************************************************************************
** Implementation [Send Data]
*****************************************************************************/

/************************************************
 * ** float 转 qstring 保留制定有效数字
 *     QString str = QString::number(f, ‘f’, 6);
 *    QString str = QString("%1").arg(f, 0, ‘f’, 6);
 * ********************************************/
void MainWindow::serialSendMocapData()
{
  uint8_t buf[1000];
  int len;
  float msg[6];

  if(ui.ck_sendmocap->isChecked()==true)
  {

    if(ui.ck_mavlink->isChecked() == true)
    {
      len = qnode.PoseXYZRPY2bufferMavlink(buf,msg);
    }
    else
    {
      len = qnode.PoseXYZRPY2buffer(buf,msg);
    }
    SPort->write(reinterpret_cast<char*>(buf),len);
    qApp->processEvents();

    ui.edt_x->setText(QString("%1").arg(msg[0]));
    ui.edt_y->setText(QString("%1").arg(msg[1]));
    ui.edt_z->setText(QString("%1").arg(msg[2]));

    ui.edt_roll->setText(QString("%1").arg(msg[3]));
    ui.edt_pitch->setText(QString("%1").arg(msg[4]));
    ui.edt_yaw->setText(QString("%1").arg(msg[5]));
    //ui.tb_msg_rcv->append(qmsg);
  }

}

int MainWindow::send_gain(uint8_t *buf,std::string &msg)
{
  int header_len = 3;
  int checksum_len = 2;
  int no_payload_len = header_len + checksum_len;
  int num_gain = 30;
  int sz_float = sizeof(float);
  int len = no_payload_len + num_gain * sz_float;
  //QByteArray send_arr;

  msg = "gain success";
  //send_arr.resize(len);

  float k[30]; // 4Byte
  k[0] = ui.edt_k01->text().toFloat();
  k[1] = ui.edt_k02->text().toFloat();
  k[2] = ui.edt_k03->text().toFloat();
  k[3] = ui.edt_k04->text().toFloat();
  k[4] = ui.edt_k05->text().toFloat();
  k[5] = ui.edt_k06->text().toFloat();
  k[6] = ui.edt_k07->text().toFloat();
  k[7] = ui.edt_k08->text().toFloat();
  k[8] = ui.edt_k09->text().toFloat();
  k[9] = ui.edt_k10->text().toFloat();

  k[10] = ui.edt_k11->text().toFloat();
  k[11] = ui.edt_k12->text().toFloat();
  k[12] = ui.edt_k13->text().toFloat();
  k[13] = ui.edt_k14->text().toFloat();
  k[14] = ui.edt_k15->text().toFloat();
  k[15] = ui.edt_k16->text().toFloat();
  k[16] = ui.edt_k17->text().toFloat();
  k[17] = ui.edt_k18->text().toFloat();
  k[18] = ui.edt_k19->text().toFloat();
  k[19] = ui.edt_k20->text().toFloat();

  k[20] = ui.edt_k21->text().toFloat();
  k[21] = ui.edt_k22->text().toFloat();
  k[22] = ui.edt_k23->text().toFloat();
  k[23] = ui.edt_k24->text().toFloat();
  k[24] = ui.edt_k25->text().toFloat();
  k[25] = ui.edt_k26->text().toFloat();
  k[26] = ui.edt_k27->text().toFloat();
  k[27] = ui.edt_k28->text().toFloat();
  k[28] = ui.edt_k29->text().toFloat();
  k[29] = ui.edt_k30->text().toFloat();

  buf[0] = 0xff;
  buf[1] = 0xfe;
  buf[2] = 0x01;

  for(int i = 0; i<num_gain;i++)
  {
    memcpy(buf+header_len+i*sz_float,&(k[i]),sizeof(float));
  }

  buf[len-2] = 0x0d;
  buf[len-1] = 0x0a;

  return len;

}

int MainWindow::send_gain_mavlink(uint8_t *buf,std::string &msg)
{
  msg = "gain success mavlink";
  //send_arr.resize(len);

  mavlink_mav_set_gain_t mav_gain;

  mav_gain.gain_num = 30;

   // 4Byte
  mav_gain.gain[0] = ui.edt_k01->text().toFloat();
  mav_gain.gain[1] = ui.edt_k02->text().toFloat();
  mav_gain.gain[2] = ui.edt_k03->text().toFloat();
  mav_gain.gain[3] = ui.edt_k04->text().toFloat();
  mav_gain.gain[4] = ui.edt_k05->text().toFloat();
  mav_gain.gain[5] = ui.edt_k06->text().toFloat();
  mav_gain.gain[6] = ui.edt_k07->text().toFloat();
  mav_gain.gain[7] = ui.edt_k08->text().toFloat();
  mav_gain.gain[8] = ui.edt_k09->text().toFloat();
  mav_gain.gain[9] = ui.edt_k10->text().toFloat();

  mav_gain.gain[10] = ui.edt_k11->text().toFloat();
  mav_gain.gain[11] = ui.edt_k12->text().toFloat();
  mav_gain.gain[12] = ui.edt_k13->text().toFloat();
  mav_gain.gain[13] = ui.edt_k14->text().toFloat();
  mav_gain.gain[14] = ui.edt_k15->text().toFloat();
  mav_gain.gain[15] = ui.edt_k16->text().toFloat();
  mav_gain.gain[16] = ui.edt_k17->text().toFloat();
  mav_gain.gain[17] = ui.edt_k18->text().toFloat();
  mav_gain.gain[18] = ui.edt_k19->text().toFloat();
  mav_gain.gain[19] = ui.edt_k20->text().toFloat();

  mav_gain.gain[20] = ui.edt_k21->text().toFloat();
  mav_gain.gain[21] = ui.edt_k22->text().toFloat();
  mav_gain.gain[22] = ui.edt_k23->text().toFloat();
  mav_gain.gain[23] = ui.edt_k24->text().toFloat();
  mav_gain.gain[24] = ui.edt_k25->text().toFloat();
  mav_gain.gain[25] = ui.edt_k26->text().toFloat();
  mav_gain.gain[26] = ui.edt_k27->text().toFloat();
  mav_gain.gain[27] = ui.edt_k28->text().toFloat();
  mav_gain.gain[28] = ui.edt_k29->text().toFloat();
  mav_gain.gain[29] = ui.edt_k30->text().toFloat();

  mavlink_msg_mav_set_gain_encode(sys_id,comp_id,&mav_msg_ui,&mav_gain);
  return mavlink_msg_to_send_buffer(buf,&mav_msg_ui);
}

int MainWindow::send_stop(uint8_t *buf,std::string &msg)
{
  int header_len = 3;
  int checksum_len = 2;
  int no_payload_len = header_len + checksum_len;
  int num_payload = 1;
  int sz_per_payload = 1; // char
  int len = no_payload_len + num_payload * sz_per_payload;

  uint8_t msg_id = 0x02;

  msg = "stop uav";

  buf[0] = 0xff;
  buf[1] = 0xfe;
  buf[2] = msg_id;
  buf[3] = 0x01;
  buf[len-2] = 0x0d;
  buf[len-1] = 0x0a;
  msg = "Uav Stop Send";
  return len;
}

int MainWindow::send_stop_mavlink(uint8_t *buf,std::string &msg)
{
//mavlink set base mode to auto disarm
  msg = "stop uav mavlink";
  mavlink_set_mode_t set_mode;
  set_mode.target_system = 0x01;
  set_mode.base_mode =MAV_MODE_AUTO_DISARMED; // atuo mission disarmed force to land
  set_mode.custom_mode = 0;
  mavlink_msg_set_mode_encode(sys_id,comp_id,&mav_msg_ui,&set_mode);
  return mavlink_msg_to_send_buffer(buf,&mav_msg_ui);

}


}  // namespace btn



void btn::MainWindow::on_btn_loadparam_clicked()
{

}

void btn::MainWindow::on_cb_boadrate_currentIndexChanged(int index)
{

}

void btn::MainWindow::on_send_gain_clicked()
{
  uint8_t buf[256];
  std::string msg;
  configiniGainWrite();
  int len;

  if(ui.ck_mavlink->isChecked())
  {
    len = send_gain_mavlink(buf,msg);
  }
  else
  {
    len = send_gain(buf,msg);
  }

  qnode.log_info(msg);

  SPort->write(reinterpret_cast<char*>(buf),len);
  qApp->processEvents();
  QString qmsg =  QString::fromStdString(msg);
  ui.tb_pub->append(qmsg);
}

void btn::MainWindow::on_tb_pub_textChanged()
{
    ui.tb_pub->moveCursor(QTextCursor::End);
}

void btn::MainWindow::on_btn_stopuav_clicked()
{
  std::string msg;
  uint8_t buf[128];

  int len;

  if(ui.ck_mavlink->isChecked())
  {
    len = send_stop_mavlink(buf,msg);
  }
  else
  {
    len = send_stop(buf,msg);
  }

  qnode.log_info(msg);

  SPort->write(reinterpret_cast<char*>(buf),len);
  qApp->processEvents();

  QString qmsg =  QString::fromStdString(msg);
  ui.tb_pub->append(qmsg);

}

void btn::MainWindow::on_ck_sendmocap_stateChanged(int arg1)
{

}
