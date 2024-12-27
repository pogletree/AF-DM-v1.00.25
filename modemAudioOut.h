#ifndef MODEMAUDIOOUT_H
#define MODEMAUDIOOUT_H

#include <QObject>

//#include "modemAudioOut.h"    // Huh! Why include the module in THE module...
#include "globals.h"
#include <QAudioFormat>
#include <QAudioSink>
#include <QAudioDevice>
#include <QMutex>
#include <QMessageBox>
#include <QCoreApplication>
#include <QtEndian>
#include "dint.h"     // Not referenced but needed for interpolation.
#include "bridge.h"     // Not referenced but needed.


class ModemAudioOut :public QObject
{
    Q_OBJECT

public:
    explicit ModemAudioOut(QObject *parent = 0);

    float floatSampleFactor{.75f};      //Multiplier for raw float samples from modem soundBlock to set level.
    void setFloatSampleFactor(float);
    float getFloatSampleFactor();
    static void status(ModemStatus status, void* param);
    CodecState getOutputCodecState(); 
    void setAudioOutputDevice(QAudioDevice);
    QThread * bridgeThread;

signals:

    void startOutThread();
    void valueChanged(const QString &value);
    void finished();
    void cleanup();
    void newOutputModemStatus(int, void *);
    void messageReady(QByteArray);


public slots:

    void abort();
    void handleAudioOutStateChanged(QAudio::State newState);
    static void sendMessage(QByteArray);
    void newMessage(QByteArray);
    void setAudioOutVolume(qreal value);
    void handleTxAudioSamples();
    void initializeModem();
    void initializeAudioOut();
    void setModemSpeedInterleave(Mode);

private:

    void newStatusCallback(int status, void *param);
    void prepareSoundBlock(float *tx_samples);
    void writeMoreSamples();                                                //Holds all converted little endian samples from message processing.

    unsigned char soundBlock[19200];                                        //9600 2-byte 16-bit little endian samples from modem for audio output.
    QMutex outBufferMutex;
    QByteArray sampleBufferOut;
    QIODevice * audioOutDevice;                                             //Pointer to internal QtAudioOutput device
    QAudioSink * audioOut;                                                //Internal QAudioOut output device tied to CODEC.
    QAudioDevice outputDevice;
    CodecState outputCodecState;
    QByteArray msgBytes;
    qreal outputLinearVolume;
    qreal cumBytesAdded {0};
    qreal cumBytesRemoved;
    qreal fractionComplete;



};

#endif // MODEMAUDIOOUT_H
