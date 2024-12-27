#include "modemAudioOut.h"
#include <QObject>
#include <QMutex>
#include <QDebug>
#include <QThread>
#include <QTimer>



Cm110s * mm = new Cm110s;         //TX modem core.  Needs a status function callback.
static ModemAudioOut *ma;
//Bridge * bridge = new Bridge;     //Sends mesage to modem core in separate thread.

//This class uses a single instance of Cm110s (the modem core) for transmit only.  Call Cm110s with message bytes, and it builds
//audio samples from the message, packages them into soundblocks and adds them to a queue. Cm110s does not return until the queue
//has been emptied.  Accordingly, we use two threads with a single instance of Cm110s: one to send the message bytes to Cm110s
//(the bridge thread) and another to process the soundblocks and release them to empty the queue (the modemaudioout thread).
//Transmission begins as soon as a soundblock is available.

ModemAudioOut::ModemAudioOut(QObject *parent) :
    QObject(parent)
{
    ma = this;
    Bridge * bridge = new Bridge;
    outputCodecState = CODEC_CLOSED;
    connect (this, SIGNAL(messageReady(QByteArray)), bridge, SLOT(msgToModem(QByteArray)));
    bridge->registerSendMethod(* sendMessage);
    bridgeThread = new QThread();
    bridge->moveToThread(bridgeThread);
    connect(this, SIGNAL(finished()), bridgeThread, SLOT(quit()), Qt::DirectConnection);    //stops thread
}

void ModemAudioOut::status(ModemStatus status, void* param)
{
    //qDebug()<<Q_FUNC_INFO<<"started."<<QThread::currentThreadId();
    int intStatus=static_cast<int>(status);
    ma->newStatusCallback(intStatus, param);  //Jump from static to member function.
    return;
}

void ModemAudioOut::newStatusCallback(int status, void *param)  //Passes TX status to the GUI
{
    emit newOutputModemStatus (status, param);
}

void ModemAudioOut::initializeModem()
{
    //qDebug()<<Q_FUNC_INFO<<"Entered"<<QThread::currentThreadId();
    mm->tx_set_soundblock_size( TX_MODEM_SAMPLE_BLOCK_SIZE/5 );
    mm->register_status( *status );
    mm->tx_enable();
    mm->rx_enable();
    mm->set_psk_carrier(1800);
    mm->set_preamble_hunt_squelch(8);  //None
    mm->set_p_mode( 1 );
    mm->set_e_mode( 0 );
    mm->set_b_mode( 0 );
    mm->m_eomreset=0;
    mm->eom_rx_reset();
    mm->tx_set_mode(M600L);

}

void ModemAudioOut::abort()   //Invoked by the GUI Kill TX button.
{
        //qDebug()<<Q_FUNC_INFO<<"Aborting transmit in thread ."<<thread()->currentThreadId();
        audioOut->stop();
        audioOut->reset();
        mm->m_eomreset=0;
        mm->eom_rx_reset();
        emit finished();
        audioOutDevice = audioOut->start();
}

void ModemAudioOut::newMessage(QByteArray newMsg)  //Passes msg to be run in its own thread.
{
        //qDebug()<<Q_FUNC_INFO<<newMsg<<"THREAD: "<<QThread::currentThreadId();
        //A QTimer is used instead of a while(true) loop. Each time the timer times out,
        //a soundblock from the modem is processed and written to the audioSink. In the intervals,
        //events can be processed in MainWindow.
        //Timer timeouts are spaced such that the AudioSink sample buffer is never empty until EOM.

        cumBytesAdded = 0;
        sampleBufferOut.clear();
        bridgeThread->start();
        QTimer * txTimer = new QTimer(this);
        txTimer->setInterval(25);                                    //Short enough to keep audioSink buffer from empty.  100 ms. is too long.
        connect(txTimer,SIGNAL(timeout()), this, SLOT(handleTxAudioSamples()));        //Timeouts process a modemsoundblock.
        txTimer->start();
        emit messageReady(newMsg);  //Sending of message to be initiated from bridge thread.
}

void ModemAudioOut::sendMessage(QByteArray msgBytes)       //Called from the bridge thread to use same instance of Cm110s.
{
    //qDebug()<<Q_FUNC_INFO<<msgBytes<<"THREAD: "<<QThread::currentThreadId();
    mm->tx_enable();
    mm->tx_sync_frame_eom((unsigned char * )msgBytes.data(),msgBytes.length()); //Send msgBytes bytes to the modem to make into samples.
    //The modem core will start generating soundblocks of audio samples.  We have to process them and send the samples to the audio sink.
}

void ModemAudioOut::handleTxAudioSamples()
{
    //qDebug()<<Q_FUNC_INFO<<"started. THREAD: "<<QThread::currentThreadId();

    //For output, the modem builds 1920-sample soundblocks from transmit text.  The samples are interpolated 5x to 9600 samples,
    //scaled by a level factor, converted to 16-bit signed ints, little endian, and added to sampleBufferOut for sending
    //to the CODEC.  Simultaneously, audioOut begins draining the samplesBufferOut and sending the samples to the CODEC, even as it
    //is being filled with new samples.  Mutexes serialize access to sampleBufferOut.

    //(Unlike the MFC MS-DMT, audio input is not handled here.  QAudioInput runs in the GUI thread and issues a signal when a buffer of samples are
    //ready for processing.  We process the samples and return until the next signal wakes us up.)

    //The output (transmit) thread is started by sendMesssage() before bytes are sent to the modem and ends when the sampleBuffer is empty.

    if( outputCodecState == CODEC_OPEN )            //Set open when the audio output device has been started.
    {
        //-----------------------------------Route trasmit audio to audio device --------------------------------------------------------
        //Fill the sample buffer.
        static float *tx_samples=0;                                 //Pointer to soundblock in modem's pending soundblock queue.
        tx_samples = mm->tx_get_soundblock();                       //Get a soundblock from the queue.
        //qDebug()<<Q_FUNC_INFO<<"tx_samples = "<<tx_samples;

        if( tx_samples != 0 )                                       //A non-zero pointer means we have a soundblock.
        {
            //qDebug()<<Q_FUNC_INFO<<"tx_samples = "<<tx_samples;
            prepareSoundBlock(tx_samples);                          //Interpolate, apply mod factor, convert to 16-bit, little endian, append to sampleBuffer.                                                     //Break it into chunks and enqueue them for output to soundcard.
            mm->tx_release_soundblock( tx_samples );                //Release the soundblock from the queue. Cm110s returns whenthe queue isempty.
            tx_samples = 0;                                         //Reset the pointer to zero, waiting for the next soundblock.
        }
        if(sampleBufferOut.size()>0) writeMoreSamples();            //Keep emptying the buffer until it is empty, which kills the thread().

    }//End of codec open actions
}

void ModemAudioOut::prepareSoundBlock(float *tx_samples)        //1920 modem samples -> 9600 interpolated -> 19200 bytes of 16-bit little endian samples.
{
    static float output[TX_MODEM_SAMPLE_BLOCK_SIZE];                                        //Float array to receive the interpolated samples.
    interpolate_9600( tx_samples, output, TX_MODEM_SAMPLE_BLOCK_SIZE/5 );                   //Interpolate to 48000 sample rate.  1 sample becomes 5 in output[].
    //Now, convert the float samples in output to signed int and convert that to little endian.
    unsigned char *ptr = soundBlock;                                                        //ptr points to first byte of the soundBlock array.
    for(int i=0;i<TX_MODEM_SAMPLE_BLOCK_SIZE;i++)                                           //For each float sample in output[],
    {
        qint16 value = static_cast<qint16>((floatSampleFactor*output[i]) * (65535/2));     //convert it to signedInt, 0 - 65535
        qToLittleEndian<qint16>(value, ptr);
        ptr+=2;                                                                             //Since it used 2 bytes for the quint16, bump the pointer by 2.
    }
    QByteArray qSoundBlock((const char*) soundBlock, 19200);                                //Convert the unsigned char array to a QByteArray for QAudioOutput.
    outBufferMutex.lock();
    sampleBufferOut.append(qSoundBlock);                                                    //Add soundblock to buffer for QAudioOutput.
    cumBytesAdded += qSoundBlock.size();
    outBufferMutex.unlock();
}

void ModemAudioOut::writeMoreSamples()               //Writes samples from sampleBufferOut to audio out whenever it has room (bytesFree) in its buffer.
{
    //qDebug()<<Q_FUNC_INFO<<"Entered. THREAD: "<<QThread::currentThreadId();
    outBufferMutex.lock();                                                                   //Ensure serial access to the sampleBuffer.
    int chunkSize = std::min(audioOut->bytesFree(), sampleBufferOut.size());
    if(audioOut->bytesFree()>=chunkSize && chunkSize > 0 )                                  //Only write if there are enough bytes free.
    {
        //qDebug()<<Q_FUNC_INFO<<"Before write to audiodevice";
        audioOutDevice->write(sampleBufferOut.data(),chunkSize);                            //Send samples to the CODEC.
        //qDebug()<<Q_FUNC_INFO<<"After write to audiodevice";
        sampleBufferOut.remove(0,chunkSize);                                                //Pop front what was written from buffer.

        //We can't use the sink's IDLE signal to finish because it goes IDLE before all samples have been sent to the CODEC.
        //Instead, we will wait for sampleBufferOut to be empty.
        //Because we want to begin transmission immediately, we start removing samples as soon as the modem core starts filling sampleBufferOut.
        //So when the transmission starts and QAudioSink's buffer is empty, bytes added may = bytes removed, leaving the sampleBufferOut empty too soon.
        //The modem core adds samples to sampleBufferOut much faster than QAudioSink can take them, so adter the first 50K bytes,
        //the modem is far enough ahead of the audio sink that we can finish when the buffer is empty.
        if(cumBytesAdded > 50000)
        {  if(sampleBufferOut.size() == 0)
            {
                outputCodecState = CODEC_CLOSED;
                emit finished();
            }
        }
    }
    outBufferMutex.unlock();
    QCoreApplication::processEvents();
}

void ModemAudioOut::initializeAudioOut()
{
    //qDebug()<<Q_FUNC_INFO<<"Entered.  outputDevice = "<<outputDevice.description()<<"THREAD: "<<QThread::currentThreadId();
    if(outputCodecState == CODEC_OPEN)                                             //If a device was open,
    {
        audioOut->stop();                                                          //stop the current device,
        audioOut->disconnect(this);                                                //disconnect from slots,
        audioOut->deleteLater();
    }
    outputCodecState = CODEC_CLOSED;

    QAudioFormat format;
    format.setSampleRate(48000);
    format.setChannelCount(1);
    format.setSampleFormat(QAudioFormat::Int16);

    if (!outputDevice.isFormatSupported(format)) {
        //emit resetAudioOutComboBox();
        QMessageBox::information(nullptr,"AudioOut","Unsupported format.  Select a valid audio output device.");
        return;
    }

    audioOut = new QAudioSink(outputDevice,format,nullptr);
    audioOut->setBufferSize(38400);    //When buffer is 96000 bytes, (48000 samples, or 1 second) periodSize is 19200 bytes, 9600 samples, 200 ms.
    audioOut->setVolume(outputLinearVolume);            //  1 is max.
    connect(audioOut, SIGNAL(stateChanged(QAudio::State)), this, SLOT(handleAudioOutStateChanged(QAudio::State)));
    audioOutDevice=audioOut->start();
    outputCodecState=CODEC_OPEN;
    //qDebug()<<Q_FUNC_INFO<<"OutputCODECState = "<<outputCodecState;
}

void ModemAudioOut::handleAudioOutStateChanged(QAudio::State newState)    //Terminates the thread when output (message) is transferred to audio out.
{
    //qDebug()<<Q_FUNC_INFO<<"Entered."<<newState;
    switch (newState)
    {
        case QAudio::IdleState:                                                     // Finished playing (no more data)
        {
            //emit finished();
            break;
        }
        case QAudio::StoppedState:                                                  // Stopped for other reasons
            //qDebug()<<Q_FUNC_INFO<<"AudioOut is in stopped state.";

            switch(audioOut->error())
            {
            case QAudio::NoError:
                break;
            case QAudio::OpenError:
                QMessageBox::information(nullptr,"QAudioError","Unable to open soundcard.");
                break;
            case QAudio::IOError:
                QMessageBox::information(nullptr,"QAudioError","Soundcard read/write Error.");
                break;
            case QAudio::UnderrunError:
                QMessageBox::information(nullptr,"QAudioError","Data not flowing to soundcard fast enough.");
                break;
            case QAudio::FatalError:
                QMessageBox::information(nullptr,"QAudioError","Fatal error. Soundcard not usable.");
                break;
            }
        break;
        default:
            break;
    }
}

void ModemAudioOut::setFloatSampleFactor(float value)  { floatSampleFactor = value;  }

float ModemAudioOut::getFloatSampleFactor() { return floatSampleFactor;  }

CodecState ModemAudioOut::getOutputCodecState() { return outputCodecState;  }

void ModemAudioOut::setModemSpeedInterleave(Mode mode) {  mm->tx_set_mode(mode); }

void ModemAudioOut::setAudioOutputDevice(QAudioDevice info) { outputDevice = info; }

void ModemAudioOut::setAudioOutVolume(qreal level)
{
    //qDebug()<<Q_FUNC_INFO<<"----------------------------"<<audioOut->volume();
    outputLinearVolume = level;  //Used by initializer when CODEC is closed.
    if(outputCodecState == CODEC_OPEN) audioOut->setVolume(outputLinearVolume);
}

