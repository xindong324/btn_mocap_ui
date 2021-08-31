# ros qt5 界面
## 运行环境
* ubuntu18.04
* ros melodic
* Qt 5.12.11

## Notice
### Qt 闪退 0720
1. Qt中由于串口高速收发引起的内存占用不断扩大，在SPort->write 后加入 qApp->processEvents(); 内存不再增加
```
if (port.waitForReadyRead(10))
{
qApp->processEvents();

port->readAll();
}
```
2. Qt由于QbyteArray引用clean和resize()方法导致内存释放报错 double free or corruption，解决方案是可以考虑不使用clean()

3. 使用memcpy复制数据到Qbytearray会使程序崩溃，可以使用memcpy将数据复制到char数组，然后直接发送或者将char赋值给QbyteArray再发送；


## Qt 闪退0720更新
1. 发现上一个标题中的 2和3 都是在胡扯，程序崩溃不是QbyteArray异常释放引起的，而是ui界面中logiew中的logmodel在qnode中更新为了限制其长度增加了一个delet旧行函数使得内存释放报错
2. 将logview改为textbrowser，程序不再崩溃，可以考虑将被改的面目全非的Pose2xyzrpy再改回原来的赋值qbytearray，如果哪天有心情
3. textbrowser一直记录数据，会使程序占用内存缓慢增加但是不是数个小时持续使用应该不用担心，为了程序稳定，还是在界面构造函数中增加了textbrowser限制显示最大行数：ui.tb_msg_rcv->document()->setMaximumBlockCount(200); // max row of rcv msg is 200
## QChart
find_package(Qt5 REQUIRED Core Widgets SerialPort Charts)
set(QT_LIBRARIES Qt5::Widgets Qt5::SerialPort Qt5::Charts)

##Qt tcp client
