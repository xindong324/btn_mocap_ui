# ros qt5 界面
## 运行环境
* ubuntu18.04
* ros melodic
* Qt 5.12.11

## Notice
### Qt 闪退
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
