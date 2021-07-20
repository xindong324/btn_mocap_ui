
```
void MainWindow::on_button_connect_clicked(bool check ) {

  if(tr("connect") == ui.button_connect->text())
  {
    /*********************
    ** Open port
    **********************/
    //写配置信息
    configiniPortWrite();

    getComboBoxValue();
    if(!setPortConfig()) //port err
    {

    }//end port err

    else {// port open
      /*********************
      ** Launch ros
      **********************/
      if ( ui.checkbox_use_environment->isChecked() ) { // use pc env
        if ( !qnode.init() ) {
          showNoMasterMessage();
        } else {
          ui.button_connect->setEnabled(false);
        }
      }//end pc env
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
      }// end user def env

      /*********************
      ** port read signal
      **********************/
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
    } // end port open

    // set button to disconnect
    ui.button_connect->setText(tr("disconnect"));
  }
  else
  {
    runonce = false;
    ui.button_connect->setText(tr("connect"));
    SPort->close();
    qnode.rosShutdown();

  }


}
```
