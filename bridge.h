#ifndef BRIDGE_H
#define BRIDGE_H

#include <QObject>
//#include "modemAudioOut.h"    // modemAudioOut.h already includes bridge.h

class Bridge : public QObject
{
    Q_OBJECT
public:
    explicit Bridge(QObject *parent = nullptr);

    void registerSendMethod(void (* callback) (QByteArray));
    void (* sMsg) (QByteArray);


public slots:
    void msgToModem(QByteArray);


};

#endif // BRIDGE_H
