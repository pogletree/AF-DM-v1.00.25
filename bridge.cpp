#include "bridge.h"
#include <QThread>

Bridge::Bridge(QObject *parent)
    : QObject{parent}
{

}

void Bridge::registerSendMethod(void (*callback) (QByteArray))
{
    sMsg = callback;
}

//Sends the message bytes via this bridge thread to the same Cm110a instance where the soundblock queue
//is being emptied by the modemAudioOut thread.

void Bridge::msgToModem(QByteArray msg)
{
    //qDebug()<<Q_FUNC_INFO<<"Send message to modem"<<msg<<"ThreadId = "<<QThread::currentThreadId();
    sMsg(msg);
}


