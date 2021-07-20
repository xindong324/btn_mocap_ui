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
    ui.edt_k01->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k02->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k03->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k04->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k05->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k06->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k07->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k08->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k09->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k10->setValidator(new QDoubleValidator(0.0,180.0,4,this));

    ui.edt_k11->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k12->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k13->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k14->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k15->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k16->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k17->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k18->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k19->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k20->setValidator(new QDoubleValidator(0.0,180.0,4,this));

    ui.edt_k21->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k22->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k23->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k24->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k25->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k26->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k27->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k28->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k29->setValidator(new QDoubleValidator(0.0,180.0,4,this));
    ui.edt_k30->setValidator(new QDoubleValidator(0.0,180.0,4,this));
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

void MainWindow::serialSendMocapData()
{
  char buf[1000];
  int len;
  len = qnode.PoseXYZRPY2buffer(buf);
  SPort->write(buf,len);
  qApp->processEvents();
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
          ui.line_edit_topic->setReadOnly(true);
        }
      }// end user def env


      //收到数据运行槽函数
      if(runonce)//只允许运行一次
      {
         //connect(SPort,SIGNAL(readyRead()),this,SLOT(SerialRead()));
      }

      ui.cb_port->setEnabled(false);
      ui.cb_data->setEnabled(false);
      ui.cb_stop->setEnabled(false);
      ui.cb_check->setEnabled(false);
      ui.cb_boadrate->setEnabled(false);

      ui.send_gain->setEnabled(true);
      ui.button_connect->setText(tr("Disconnect"));
    }// end port open

  }// end click connect
  else
  {
    ui.button_connect->setText(tr("Connect"));
    //qnode.rosShutdown();
    runonce = false;
    qnode.close();
    SPort->close();
    ui.cb_port->setEnabled(true);
    ui.cb_data->setEnabled(true);
    ui.cb_stop->setEnabled(true);
    ui.cb_check->setEnabled(true);
    ui.cb_boadrate->setEnabled(true);
    ui.send_gain->setEnabled(false);
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

void MainWindow::send_gain()
{
  int header_len = 3;
  int checksum_len = 2;
  int no_payload_len = header_len + checksum_len;
  int num_gain = 30;
  int sz_float = sizeof(float);
  int len = no_payload_len + num_gain * sz_float;
  //QByteArray send_arr;

  QString s_gain = QString("gain success: ");
  char buf[len];
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

  SPort->write(buf,len);
  qApp->processEvents();
  ui.tb_pub->append(s_gain);

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

void btn::MainWindow::on_send_gain_clicked()
{

  configiniGainWrite();
  send_gain();
}

void btn::MainWindow::on_tb_pub_textChanged()
{
    ui.tb_pub->moveCursor(QTextCursor::End);
}
