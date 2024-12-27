#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "bridge.h"
#include <QApplication>
#include <QTimer>
#ifdef __linux__
#include <unistd.h>
#endif

Cm110s * sm; // = new Cm110s();                                             //Instantiate the modem class.
ModemAudioOut * modemAudioOut = new ModemAudioOut;
QMediaDevices * m_devices = new QMediaDevices;

static MainWindow *mw;                                 //Establish a pointer to this MainWindow class for the static data callback.
static MainWindow *pmw;                                //A separate pointer was needed for the static status callback

QSettings settings(QString("AF-DM.ini"), QSettings::IniFormat); //Replacement for db.

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle(title);

    sm = new Cm110s();                                             //Instantiate the modem class.

    QFile f("./ErrorLog.txt");  //If session logs are being used,
    if(f.exists())f.remove();   //  remove the log for the previous session.

    #if defined(_WIN32) || defined(_WIN64)
        //qDebug()<<"ifdef statements executed WIN32 or WIN64.";
        //If this is a Windows operating system, refuse to operate on anything less than Windows 10.
        auto OSInfo = QOperatingSystemVersion::current();
        auto OSType= OSInfo.type();
        if(OSType == 1 )
        {
            if(OSInfo < QOperatingSystemVersion::Windows10)
            {
                QMessageBox::information(nullptr, "Information","AF-DM is restricted to Windows 10 or above.");
                exit(0);
        }
    }
    #endif

    connect(ui->tabsModem, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()));

   //Run a separate transmit modem in a thread.  Testing the concept.
    modemAudioOutThread = new QThread();
    modemAudioOut->moveToThread(modemAudioOutThread);
    connect(modemAudioOut, SIGNAL(finished()), modemAudioOutThread, SLOT(quit()), Qt::DirectConnection);    //stops thread
    connect(modemAudioOut, SIGNAL(finished()), this, SLOT(cleanupAfterSend()));
    connect(modemAudioOutThread, SIGNAL(started()), modemAudioOut, SLOT(initializeAudioOut()));
    connect(modemAudioOutThread, SIGNAL(started()), modemAudioOut, SLOT(initializeModem()));
    connect(this, SIGNAL(sendMsg(QByteArray)), modemAudioOut, SLOT(newMessage(QByteArray)));
    //The send method starts the thread, then starts the modem loop.
    connect (this, SIGNAL(txModeChanged(Mode)), modemAudioOut, SLOT(setModemSpeedInterleave(Mode)));
    connect(this,SIGNAL(killTx()), modemAudioOut, SLOT(abort()));

    inputCodecState = CODEC_CLOSED;

    connect (this, SIGNAL(audioOutVolumeChanged(qreal)), modemAudioOut, SLOT(setAudioOutVolume(qreal)));

    mw=this;            //Sets the pointer (mw) to the current class (MainWindow) address so that the static output_data_octet() callback function
                        //can send the octet to a member function in the MainWindow class.
    pmw=this;           //Sets the pointer (pmw) to the current class (MainWindow) address so that the static status() callback function
                        //can send the status to a member function in the MainWindow class.
                        //Could not use one pointer for both callbacks.  Tried a second one and it worked.

    radioCommandWindowCursor=ui->plainTextEditRadioCommands->textCursor();

    ui->labelTransmitState->setText("IDLE");
    ui->labelReceiveMode->setText("HUNTING");
    ui->labelReceiveState->setText("IDLE");

    // Decorations would be better in the Stylesheet
    ui->labelTransmitState->setStyleSheet("background-color: lightyellow");
    ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");
    ui->labelReceiveState->setStyleSheet("background-color: lightyellow");
    ui->labelBEC->setStyleSheet("background-color: lightyellow");
    ui->labelBER->setStyleSheet("background-color: lightyellow");
    ui->labelFER->setStyleSheet("background-color: lightyellow");
    ui->labelSNR->setStyleSheet("background-color: lightyellow");
    ui->spinBoxBECThreshold->setStyleSheet("background-color: lightyellow");
    ui->spinBoxIPAddress_P1->setStyleSheet("background-color: lightyellow");
    ui->spinBoxIPAddress_P2->setStyleSheet("background-color: lightyellow");
    ui->spinBoxIPAddress_P3->setStyleSheet("background-color: lightyellow");
    ui->spinBoxIPAddress_P4->setStyleSheet("background-color: lightyellow");
    ui->spinBoxCustomDataPort->setStyleSheet("background-color: lightyellow");
    ui->pushButtonResetRx->setStyleSheet("background-color: tomato");
    // End decorations

    ui->labelDBFS->setText("");

    radioControlPort = new QSerialPort(this);                             //Serial port for radio control (PTT).
    ui->comboBoxRadioControlPort->clear();
    ui->comboBoxRadioControlPort->addItem("NONE");
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
       { ui->comboBoxRadioControlPort->addItem(serialPortInfo.portName()); }

    //Fill audio input device comboBox device selections.
    for (auto &deviceInfo : m_devices->audioInputs())
        ui->comboBoxInputDevice->addItem(deviceInfo.description(), QVariant::fromValue(deviceInfo));

    //Fill audio output device comboBox device selections.
    for (auto &deviceInfo : m_devices->audioOutputs())
        ui->comboBoxOutputDevice->addItem(deviceInfo.description(), QVariant::fromValue(deviceInfo));

    ui->comboBoxSpeedInterleave->setCurrentIndex(7);
    emit txModeChanged(static_cast<Mode>(ui->comboBoxSpeedInterleave->currentIndex()));
    initializeModem();
    readSettings();                                                                     //Restore user selections.

    modemSyncMode=SYNC_EOM_MODE;                                                        //Fixed for now.  Other modes available.  Will add to GUI later.
    modemSyncMode=static_cast<SyncMode>(0);

    static const char eomBuf[] = "\x4b\x65\xa5\xb2";                                    //Initialize eomBytes, the EOM search string.
    eomBytes = QByteArray::fromRawData(eomBuf, sizeof(eomBuf)-1);
    payload = "";                                                                       //Holds all octets of the incoming message...for scanning.
    msgBuffer.clear();                                                                  //The incoming message buffer.

    tcpConnectionTimer.start(2000);

    //For AudioIn, a separate thread is not needed.  We use signals and slots.  AudioIn issues a signal that starts "handleAudioInputSamples"
    //when it has a buffer of new audio samples available. The function processes them when it ends, it issues another signal to send the bytes
    //on to the terminal interface.

    bool success;

    success = connect(this, SIGNAL(newAudioInSamples()), this, SLOT(processAudioInSamples()));
    Q_ASSERT(success);

    success = connect(this, SIGNAL(newMessageBytes(QByteArray)), this, SLOT(bytesReceived(QByteArray)));
    Q_ASSERT(success);

    success = connect(this, SIGNAL(newModemStatus(int,void *)), SLOT(handleModemStatusChanges(int, void *)));
    Q_ASSERT(success);

    dataServer = new QTcpServer(this);
    QHostAddress host;
    host.setAddress(ipv4Address);
    if (!dataServer->listen(host, iDataPort)) {
        QString tcpError = "TCP/IP.  Data Port:    " + dataServer->errorString() + ".";
        writeLog(tcpError);
    }
    connect(dataServer, SIGNAL(newConnection()), this, SLOT(onNewDataConnection()));

    controlServer = new QTcpServer(this);
    iControlPort = (iDataPort + 1);
    if (iControlPort > 65534)
    {
        QString tcpError = "TCP/IP.  Control Port: " +QString::number(iControlPort) + " is out of range.";
        writeLog(tcpError);
        QMessageBox::information(this, "Control Port Address", "The Dataport is set too high causing the Control port to be out of range. Maximum Dataport is 65534.");
    }
    if (!controlServer->listen(host, iControlPort)) {
        QString tcpError = "TCP/IP.  Control Port: " + controlServer->errorString() + ".";
        writeLog(tcpError);
        //qDebug("controlServer failed to listen: %s", controlServer->errorString().toLatin1().constData());
    }
    connect(controlServer, SIGNAL(newConnection()), this, SLOT(onNewControlConnection()));


    dataSocketIsConnected=false;
    controlSocketIsConnected=false;

    connect(&eomTimer, &QTimer::timeout,  [&] {payload.clear(); eomTimer.stop();sm->m_eomreset=0;
        sm->eom_rx_reset();});

    //This statement eliminates extraneous leading nulls.  Leading nulls that are actually part of the message are not eliminated.
    //A varying number of nulls (depends on the data rate) continue to come in from the modem core after EOM.
    //The timer is set to 100 ms. at detection of EOM on a received message.
    //The delay allows any flush nulls to finish arriving, then fires the payload clearing function and stops the timer.
    //The next octet from the modem core will begin the next message.

    success = connect(modemAudioOut, SIGNAL(newOutputModemStatus(int,void *)), SLOT(handleModemStatusChanges(int, void *)), Qt::QueuedConnection);
    Q_ASSERT(success);

    ui->progressBarRxLevel->setRange(0,100);
    ui->progressBarRxLevel->setValue(0);
    ui->progressBarRxLevel->setTextVisible(false);

    qApp->installEventFilter(this);

    connect(&audioInCodecResetTimer, SIGNAL(timeout()), this, SLOT(handleAudioInCodecDisconnect()));

    if(autoResetAudioInCodec) audioInCodecResetTimer.start(3000);
    statusBar()->showMessage("Modem ready. ");

    // All setup - make sure radio is in RX
    if (ui->comboBoxRadioControlPort->currentText()=="NONE")
    {
        // Using EXT VOX PTT
    }
    else
    {
        if (radioControlPort->open(QIODeviceBase::ReadWrite))
        {
            setRadioReceiveState();
            radioControlPort->close();
        }
    }
    ui->tabModem->setFocus();
    writeLog("Started" );
}

MainWindow::~MainWindow()
{
   writeSettings();
    if(dataSocketIsConnected) dataSocket->disconnect();
    if(controlSocketIsConnected) controlSocket->disconnect();

    if (radioControlPort->isOpen()) radioControlPort->close();
    modemAudioOutThread->deleteLater();
    delete sm;
    delete radioControlPort ;
    delete m_devices;
    writeLog("Exited.");
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)                                     //Shuts things down...
{
    event->accept();
    QApplication::quit();
}

bool MainWindow::event(QEvent *ev)
{
    return (QMainWindow::event(ev));
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    //Handle tool tips.
    if (event->type() == QEvent::ToolTip) return !toolTipsVisible;
    return QWidget::eventFilter(obj, event);

}  //end eventFilter

//*******************************************************************************************************************************************
//*********************************?*********** Delay code that doesn't freeze the GUI ************************************************
//*******************************************************************************************************************************************

void MainWindow::delay (quint16 iDelay)
{
    QEventLoop loop;
    QTimer::singleShot(iDelay, &loop, &QEventLoop::quit);
    loop.exec();
}

//============================================================================================================================
//*************************************  GUI Code and Entry Points for Messages In and Out   *********************************
//============================================================================================================================

//Messages flow into and out of the modem via the next two functions.

void MainWindow::bytesReceived(QByteArray newBytes)  //Received message bytes from the modem appear in this function. Pass to terminal via TCP.
{
    //Here, we will send the newBytes (buffer) to the TCP Data Socket.
    if(dataSocketIsConnected) writeDataSocket(newBytes);                        //Send them to NOSS via TCP.
}

bool MainWindow:: modemReady()                                                         //See if the user has configured everything.
{
    if(!settingsLoaded) return true;                                                    //Allow testing without all connections.
    QString modemSetupProblems;
    modemSetupProblems.clear();
    if(inputCodecState == CODEC_CLOSED) modemSetupProblems.append("  No valid audio input CODEC selected\n");
    if(!validOutputDevice) modemSetupProblems.append("  No valid audio output CODEC selected\n");
    if (!ui->comboBoxRadioControlPort->currentText().contains("NONE"))
    {
        if(!radioControlPort->isOpen())modemSetupProblems.append("  Radio control port is not connected.\n");
    }

    if(modemSetupProblems.length()>0)
    {
        for (QString erStr:modemSetupProblems)
        {
            statusBar()->showMessage(erStr);
            delay(500);
        }
        statusBar()->showMessage("Modem initilzation failed.");
        return false;
    }
    return true;
}

void MainWindow::send(QByteArray( msgBytes))        //New message byte arrays enter the modem via this function -- from TCP and test send functions below.
{
    emit txModeChanged(static_cast<Mode>(ui->comboBoxSpeedInterleave->currentIndex()));
    ui->labelTransmitState->setText("TRANSMIT");
    ui->labelTransmitState->setStyleSheet("background-color: tomato");

    ui->labelReceiveMode->setText("IN TX");
    ui->labelReceiveMode->setStyleSheet("background-color: tomato");

    setRadioSendState();    //Sends commands for data mode, modulation source and PTT.
    //Receive state is restored when audioOut times out (is finished).
    showCmdMessage("Sending message ...");

    if(!modemReady()) {cleanupAfterSend(); return;}
    delay(waitBeforeTones);             // Delay tone generation to insure the radio is in TX
    modemAudioOutThread->start();
    sm->rx_disable();
    transmitting = true;       //Block the receive modem.  Causes it to reject receive audio samples while transmitting.
    QCoreApplication::processEvents();
    emit sendMsg(msgBytes);
}

void MainWindow::cleanupAfterSend(){                                               //Executed when AudioOut goes to Idle state.
    //qDebug()<<Q_FUNC_INFO<<"Set transmitting = false.";
    sm->rx_enable();
    setRadioReceiveState();

    writeControlSocket("IDLE\n");
    transmitting=false;                                  //Unblocks the receive modem.
    ui->labelTransmitState->setText("IDLE");
    ui->labelTransmitState->setStyleSheet("background-color: lightyellow");

    ui->labelReceiveMode->setText("HUNTING");
    ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");


}

QAudioDevice MainWindow::getOutputAudioDevice(QString &name)                   //Finds user-selected device name in list.
{
    QAudioDevice device;
    QList<QAudioDevice> devices = m_devices->audioOutputs();
    for(int i = 0; i < devices.size(); ++i) {

        if(devices.at(i).description() == name) {
            device = devices.at(i);
            break;
        }
    }
    return device;
}

QAudioDevice MainWindow::getInputAudioDevice(QString &name)   //Link user-selected device name to deviceInfo so we can set it up.
{
    QAudioDevice device;
    QList<QAudioDevice> devices = m_devices->audioInputs();
    for(int i = 0; i < devices.size(); ++i) {
        if(devices.at(i).description() == name) {
            device = devices.at(i);
            break;
        }
    }
    return device;
}

void MainWindow::tabSelected()
{

    if(ui->tabsModem->currentIndex() == 1)                              //Audio tab
    {
        enableRxLevelMeter = true;
        ui->groupBoxRxLevel->show();
    }
    else
    {
        enableRxLevelMeter = false;
        ui->groupBoxRxLevel->hide();
    }

    if(ui->tabsModem->currentIndex() == 2)      //TCP/IP tab
    {
        if(dataSocketIsConnected) statusBar()->showMessage( "TCP dataSocket connected.");
        else statusBar()->showMessage( "TCP dataSocket disconnected.");
    }

    if(ui->tabsModem->currentIndex() == 3)      //RadioCtrl tab
    {
        if(radioControlPort->isOpen()) {
            QString msg = "Radio Port on %1 : %2, %3, %4, %5, %6";
            statusBar()->showMessage( msg.arg(ui->comboBoxRadioControlPort->currentText()).arg(ui->comboBoxRadioControlPortBaudRate->currentText()).arg(dataBits)
                                     .arg(parity).arg(stopBits).arg("None"));
        }
        else if (ui->comboBoxRadioControlPort->currentText() == "NONE")
        {
            statusBar()->showMessage( "No CAT control selected.");
        }
        else statusBar()->showMessage( "Radio port disconnected.");
    }
}

//*****************************************************************************************************************************
//***********************************   Receive Side.  Audio In CODEC samples -> modem -> Message Bytes   *********************
//*****************************************************************************************************************************

void MainWindow::initializeAudioIn(QAudioDevice inputDevice)            //Initialises Audio In
{
    if(autoResetAudioInCodec) audioInCodecResetTimer.stop();
    if(inputCodecState == CODEC_OPEN)                                             //If a device was open,
    {
        audioIn->stop();                                                          //stop the current device,
        audioIn->disconnect(this);                                                //disconnect from slots,
        audioIn->deleteLater();
        audioIn->setVolume(1);
    }
    inputCodecState = CODEC_CLOSED;
    format.setSampleRate(48000);
    format.setChannelCount(1);
    format.setSampleFormat(QAudioFormat::Int16);

    if (!inputDevice.isFormatSupported(format)) {
        //ui->comboBoxInputDevice->setCurrentIndex(0);   //Reset to empty entry.
        QMessageBox::information(nullptr,"QAudioError","Raw audio format not supported.  Cannot hear audio.");
        return;
    }
    audioIn = new QAudioSource(inputDevice, format, this);
    audioIn->setBufferSize(480000);   //When buffer is 96000 bytes, (48000 samples, or 1 second) periodSize is 19200 bytes, 9600 samples, 200 ms.
    qreal linearVolume = QAudio::convertVolume(ui->horizontalSliderAudioIn->value() / qreal(10000), QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
    audioIn->setVolume(linearVolume);
    connect(audioIn, SIGNAL(stateChanged(QAudio::State)), this, SLOT(handleAudioInStateChanged(QAudio::State)));
    audioInDevice = audioIn->start();
    bool success = connect(audioInDevice, &QIODevice::readyRead, this, &MainWindow::handleAudioInData);   //Process samples when available.
    Q_ASSERT(success);
    inputCodecState=CODEC_OPEN;
    if(autoResetAudioInCodec) audioInCodecResetTimer.start(3000);
}

void MainWindow::handleAudioInStateChanged(QAudio::State newState)              //Handles changes to AudioIn.  Just waynings, now.
{
    switch (newState)
    {
        case QAudio::ActiveState:
            ui->labelReceiveState->setText("Receive");
//            ui->labelReceiveState->setStyleSheet("background-color: mediumseagreen");
//            ui->labelReceiveMode->setStyleSheet("background-color: mediumseagreen");
            break;
        case QAudio::IdleState:
            ui->labelReceiveState->setText("Idle");
            ui->labelReceiveState->setStyleSheet("background-color: lightyellow");
            ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");
            break;
        case QAudio::StoppedState:                                                   // Stopped for other reasons
        {
            ui->labelReceiveState->setText("Error!");
            switch(audioIn->error())
            {
                case QAudio::NoError:
                    break;
                case QAudio::OpenError:
                    QMessageBox::information(nullptr,"QAudioError","Unable to open soundcard.");
                    break;
                case QAudio::IOError:
                    writeLog("QAudioError:  Soundcard read/write Error. Reinitializing audioIn.");
                    break;
                case QAudio::UnderrunError:
                    QMessageBox::information(nullptr,"QAudioError","Data not flowing to soundcard fast enough.");
                    break;
                case QAudio::FatalError:
                    QMessageBox::information(nullptr,"QAudioError","Fatal error. Soundcard not usable.");
                    break;
             }
        }
        break;
        default:
            break;
    }
}

void MainWindow::handleAudioInCodecDisconnect()
{
    writeLog("AudioIn Error: No samples from AudioIn Codec.  Reinitializing AudioIn and Modem." );
    initializeAudioIn(inputDevice);
    initializeModem();
    audioInCodecResetTimer.stop();
    audioInCodecResetTimer.start(3000);
}

void MainWindow::handleAudioInData()                                            //Receives incoming audio samples and passes to modem to demodulate.
{
    //qDebug()<<Q_FUNC_INFO<<"Entered.";

    int bufferSize = 19200;
    int len = audioIn->bytesAvailable();
    if(len > 0)
    {
        QByteArray buffer(bufferSize, 0);
        int bytesRead = audioInDevice->read(buffer.data(), bufferSize);
        if(bytesRead>0)
        {
            //inBufferMutex.lock();
            sampleBufferIn.append(buffer.mid(0,bytesRead));
            //inBufferMutex.unlock();
            if(sampleBufferIn.size()>=19200) processAudioInSamples();
        }

        if(bytesRead < 0)     //We have an I/O error.
        {
            writeLog("QAudioSource:  Error reading from the audioIn CODEC.");
            writeLog(audioInDevice->errorString());
            initializeAudioIn(inputDevice);
            sampleBufferIn.clear();
            payload.clear();
            sm->m_eomreset = 0;
            sm->eom_rx_reset();
            sm->rx_reset();
            writeLog("Reinitialized audioIn.");
        }
    }
    if(autoResetAudioInCodec)   //Timer will reset audioIn if no samples ready in 3 seconds.
    {
        audioInCodecResetTimer.stop();
        audioInCodecResetTimer.start(3000);
    }
}

void MainWindow::processAudioInSamples()                                        //Pre-processes samples and sends them to the modem for demod.
{                                                                               //The samples represent one second of audio.
    //inBufferMutex.lock();                                                       //Ensure serial access to the sampleBufferIn.
    if(transmitting) { sampleBufferIn.clear(); }                                //Do not process incoming audio while transmitting.
    int chunkSize = 19200;                                                      //Number of bytes in Size of 9600 2-byte int16's (shorts).
    while (sampleBufferIn.size()>=chunkSize)
    {
        dint_decimate((short *) sampleBufferIn.data(), chunkSize/2,             //Dint takes 9600 shorts,
                      decimatedAudioBlock, lenDecimated);                       //  reduces to every 5th sample, 1920 shorts.

        sampleBufferIn.remove(0,chunkSize);                                     //Pop front (what was processed) from buffer.
        sm->rx_process_block((signed short *) decimatedAudioBlock,lenDecimated);                //Converts shorts to float and sends through dmodulator.

        if(enableRxLevelMeter)                       //Update the peak audio level monitor.
        {
            double dbfs;
            double peak = 0;
            double sample = 0;
            //for(int k=1; k<lenDecimated; k++)
            for(int k=1; k<100; k++)  //Works just as well for peaks, and 1/10 of the calculations.
            {
                sample = decimatedAudioBlock[k];
                sample = sample*0.000030517578125;
                peak = std::max(peak, abs(sample));
            }

            ui->progressBarRxLevel->setValue(   (1+ std::log10(peak)/2.4)*100    );
            dbfs = 20*log10(peak);
            if( (std::isinf(dbfs)) | (dbfs < -50) )
            {
                ui->progressBarRxLevel->setValue(0);
                dbfs = -48;
            }

            ui->lineEditClipping->setStyleSheet("background-color: mediumseagreen");
            if(dbfs > -6)ui->lineEditClipping->setStyleSheet("background-color: yellow");
            if(dbfs > -3)ui->lineEditClipping->setStyleSheet("background-color: red");
            ui->labelDBFS->setText(QStringLiteral("%1").arg((int)dbfs, 3, 10, QLatin1Char('0')));
        }
    
    QCoreApplication::processEvents();
    //inBufferMutex.unlock();
    }
}

void MainWindow::output_data_octet( U8 octet )                                  //Modem sends demodulated octets from incoming messages to here (callback).
 {
    //qDebug()<<Q_FUNC_INFO;
    mw->newDataOctet(octet);
 }

void MainWindow::newDataOctet( U8 octet )                                           //Bridge function out of the static callback into the member function world.
{
    lookForSyncEomSequence(octet);
}

void MainWindow::lookForSyncEomSequence(U8 octet)                                   //Search the raw, bits-reversed bytes for the EOM sequence.
{
    //qDebug()<<Q_FUNC_INFO<<octet;

    payload.append(octet);
    if(payload.length()>4 && sm->m_eomreset!=1)                                     //Look for he 4-byte EOM sequence.
    {
        addToMessageBuffer(payload[payload.length()-5]);                            //After a 4-byte delay, add the byte to the message buffer.

        if(payload.contains(eomBytes))                                              //(The 4-byte EOM is not added to the message buffer).
        {
            //qDebug()<<Q_FUNC_INFO<<"  found EOM";                                 //Found the SYNC_EOM sequence.
            flushMessageBuffer();                                                   //Send any bytes remaining in the buffer at EOM to NOSS.
            payload.clear();
            sm->m_eomreset = 0;
            sm->eom_rx_reset();
            return;
        }
        if(sm->m_eomreset == 1)                                                      //We did not find the EOM sequence, but the modem did and signaled EOM.  Quit anyway.
        {
            //qDebug()<<Q_FUNC_INFO<<"  sm->m_eomreset = 1";
            flushMessageBuffer();                                                    //Send any bytes remaining in the buffer to NOSS.
            payload.clear();
            sm->m_eomreset = 0;
            sm->eom_rx_reset();
            return;
        }
        if(sm->m_eomreset == 2)                                                       //The modem signaled the EOM was corrupt,  Quit.
        {
            //qDebug()<<Q_FUNC_INFO<<"  sm->m_eomreset = 2";
            flushMessageBuffer();                                                     //Send any bytes remaining in the buffer to NOSS.
            payload.clear();
            sm->m_eomreset = 0;
            sm->eom_rx_reset();
            return;
        }
    }
    //After this, may have to try to detect and deal with other corrupt EOMs that lead to modem run-on.  Might be more frequent with ASYNC or SYNC modes.
}

void MainWindow::addToMessageBuffer(unsigned char octet)                        //Buffers demodulated bytes, outputs 50-byte packets to TCP port.
{
    if(modemSyncMode != ASYNC_MODE)                                             //Reverse the bits of the octet.
    {
        octet = ((octet>>1)&0x55)|((octet<<1)&0xAA);
        octet = ((octet>>2)&0x33)|((octet<<2)&0xCC);
        octet = (octet>>4) | (octet<<4) ;
    }
    msgBuffer.append(octet);                                                    //Add the octet to a buffer to accumulate msgBufferSize octets before sending.
    if (msgBuffer.size()>msgBufferLimit)                                        //When buffer is full...
    {
        emit newMessageBytes(msgBuffer);                                        //Send message segment to terminal interface.
        msgBuffer.clear();                                                      //Reset the buffer to empty for next octet.												  //  Partial buffer will be emptied when EOM is detected.  See that code.
    }
}

void MainWindow::flushMessageBuffer() {                                         //Outputs remaining bytes in buffer.

    if (msgBuffer.size() > 0) {
        //qDebug()<<Q_FUNC_INFO<<"  message buffer = "<<msgBuffer;
        emit newMessageBytes(msgBuffer);                                        //Send msg segment to terminal interface.
        msgBuffer.clear();                                                      //Reset the buffer to empty for next octet.
        if(ui->checkBoxEnableMATCHDR->isChecked()){matchDataRate();}            //Match the TX data rate to the RX data rate.
        eomTimer.start(100);
    }
}

//*****************************************************************************************************************************
//***************************************************  Receive Modem Interface ************************************************
//*****************************************************************************************************************************

void MainWindow::initializeModem()
{
    //qDebug()<<Q_FUNC_INFO<<"  begin initmodem"<<sm;                                 //Found the SYNC_EOM sequence.

    sm->set_noise_reduction_state(0);
    sm->set_agc_state(0);
    sm->tx_set_soundblock_size( TX_MODEM_SAMPLE_BLOCK_SIZE/5 );
    sm->register_receive_octet_callback_function(*output_data_octet );
    sm->register_status( *status );
    sm->tx_enable();
    sm->rx_enable();
    sm->set_psk_carrier(1800);
    sm->set_preamble_hunt_squelch(8);  //None
    sm->set_p_mode( 1 );
    sm->set_e_mode( 0 );
    sm->set_b_mode( 0 );
    sm->m_eomreset=0;
    sm->eom_rx_reset();
}

void MainWindow::matchDataRate()                                       //Sets TX data rate to the last RX data rate.
{
    ui->comboBoxSpeedInterleave->setCurrentIndex(static_cast<int>(sm->rx_get_mode()));
    sm->tx_set_mode(sm->rx_get_mode());
}

void MainWindow::status(ModemStatus status, void* param){

    //if(status == FREQ_ERROR_STATUS) qDebug()<<Q_FUNC_INFO<<"SNR_STATUS, param = "<<*((float*)param);
    int intStatus=static_cast<int>(status);
    pmw->newStatusCallback(intStatus, param);  //Jump from static to member function.
}

void MainWindow::newStatusCallback(int status, void *param)
{
    emit newModemStatus(status, param);        //Emits signal and returns to modem thread.  Slot takes over in MainWindow thread.
}

void MainWindow::handleModemStatusChanges(int iStatus, void * param)
{
    ModemStatus status;
    status = static_cast<ModemStatus>(iStatus);
    switch (status)
    {
        case TEXT_STATUS:
        {
            float val = *(float*)param;
            if(val == 7777.0)
            {
                //Digital voice stuff goes here, when implemented
            }
            break;

        }
        case DCD_FALSE_STATUS:
        {
            ui->labelReceiveState->clear();
            ui->labelReceiveState->setText(sm->get_state_text());
            ui->labelReceiveMode->setText("HUNTING");
            ui->labelReceiveState->setStyleSheet("background-color: lightyellow");
            ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");

            if(!transmitting) writeControlSocket("ERROR\n");
            break;
        }
        case DCD_TRUE_STATUS:
        {
            ui->labelReceiveState->clear();
            ui->labelReceiveState->setText(sm->get_state_text());
            ui->labelReceiveState->setStyleSheet("background-color: mediumseagreen");
            ui->labelReceiveMode->setStyleSheet("background-color: mediumseagreen");
            QString mode = sm->rx_get_mode_string();
            int bps = mode.indexOf("BPS");                  //Fit mode string from modem into smaller window.
            mode = mode.left(bps) + mode.mid(bps+4);
            mode.replace("LONG", "L");
            mode.replace("SHORT","S");
            ui->labelReceiveMode->setText(mode);
            QByteArray modeString =  "IND RX ";
            modeString.append(sm->rx_get_mode_string());
            writeControlSocket ("DCD " + modeString+"\n");
            break;
        }

        case TX_TRUE_STATUS:
        {                                                        
            ui->labelTransmitState->setText("TRANSMIT");
            ui->labelTransmitState->setStyleSheet("background-color: tomato");
            ui->labelReceiveMode->setText("IN TX");
            ui->labelReceiveMode->setStyleSheet("background-color: tomato");

            writeControlSocket("BUSY\n");
            break;
        }

        case TX_FALSE_STATUS:
        {
            //ui->lineEditTransmitState->setText("IDLE");
            //writeControlSocket("IDLE\n");  //Sent too soon if sent here (where it was in old MS-DMT).  Move to cleanupAfterSend.
            break;
        }

        case FREQ_ERROR_STATUS:
        {   if (sm->m_eomreset == 1 || sm->m_eomreset == 2) { return; }
            ui->labelReceiveState->clear();
            ui->labelReceiveState->setText(sm->get_state_text());
//            ui->labelReceiveState->setStyleSheet("background-color: mediumseagreen");
//            ui->labelReceiveMode->setStyleSheet("background-color: mediumseagreen");
            float newFER = *(float*)param;
            if (std::isnan(newFER) || std::isinf(newFER))return;
            float oldFER = ui->labelFER->text().toFloat();
            if(newFER != oldFER)
            {
                newFER=(newFER+oldFER)/2;
                ui->labelFER->setText(QString::number(newFER,'f',0)+" Hz");
            }
            break;
        }

        case SNR_STATUS:
        {      
            if(ui->labelTransmitState->text().contains("IDLE"))  //Clear BER and BEC lineEdits.
            {
                ui->labelBER->clear();
                ui->labelBEC->clear();
            }
            if (sm->m_eomreset == 1 || sm->m_eomreset == 2) { return; }
            float snr = *(float*)param;
            if (std::isnan(snr) || std::isinf(snr))return;
            int smooth = 4;
            snrsIndex++;
            snrsIndex %= smooth;
            snrs[snrsIndex] = snr;
            float meanSnr = 0;
            for (int i=0; i<smooth; i++) meanSnr+= snrs[i];
            meanSnr = meanSnr/smooth;
            getSNRAve = (getSNRAve + meanSnr)/2;
            QString snrText;
            snrText = QString::number(meanSnr, 'f', 2)+" dB";
            ui->labelSNR->setText(snrText);
            break;
        }
        case PREAM_SNR_STATUS:
        {
            if(ui->labelTransmitState->text().contains("IDLE"))     //Clear BER and BEC lineEdits.
            {
                ui->labelBER->clear();
                ui->labelBEC->clear();
            }
            if (sm->m_eomreset == 1 || sm->m_eomreset == 2) { return; }
            float val = *(float*)param;
            if (std::isnan(val) || std::isinf(val))return;
            getSNRAve = (getSNRAve + val)/2;
            QString snrText;
            snrText = QString::number(val,'f',0 )+" dB";
            ui->labelSNR->setText(snrText);
            break;
        }
        case VITERBI_STATUS:
        {
            if (sm->m_eomreset == 1 || sm->m_eomreset == 2) { return; }
            ui->labelReceiveState->clear();
            ui->labelReceiveState->setText(sm->get_state_text());
            ui->labelReceiveState->setStyleSheet("background-color: mediumseagreen");
            ui->labelReceiveMode->setStyleSheet("background-color: mediumseagreen");
            float val = *((float*)param);
            if (std::isnan(val) || std::isinf(val))return;
            break;
        }

        case TRAIN_STATUS:
        {
            //Not implemented in MFC version.
            break;
        }

        case BER_STATUS:
        {
            if (sm->m_eomreset == 1 || sm->m_eomreset == 2) { return; }
            float val = *((float*)param);
            if (std::isnan(val) || std::isinf(val))return;
            QString ber = QString::number(val);
            ui->labelBER->setText(ber.right(2));
            break;
        }
        case BEC_STATUS:
        {
            if (sm->m_eomreset == 1 || sm->m_eomreset == 2) { return; }
            float val = *((float*)param);
            ui->labelBEC->setText(QString::number(val,'f',0 ));

            if(ui->checkBoxBECAutoReset->isChecked())
            {
                if(val>=ui->spinBoxBECThreshold->value())
                {
                    sm->rx_reset();
                    payload = "";
                }
            }
            break;
        }
        QCoreApplication::processEvents();
    }
}

//***************************************************************************************************************************************
//**********************************************************  NOSS Command Handler  *******************************************************
//***************************************************************************************************************************************


void MainWindow::handleCommands(QByteArray command)      //Processes speed change, reset commands, etc. received on TCP control port from NOSS..
{
    command = command.toUpper();
    int nIndex=99;

        if(command.contains("RADIO"))
        {
            processTcpRadioCommand(command);

        }

    //********************************************************************************************************************************
    //*********************************** Not a radio command. Modem control string? ******************************
    //********************************************************************************************************************************

    // NOSS compatible commands - PWO

        else if (command.contains("75S"))
        {
            nIndex = 0;
            changeDataRate(nIndex);
            writeControlSocket("75S\n");
            chkMDState();
        }
        else if (command.contains("75L"))
        {
            nIndex = 1;
            changeDataRate(nIndex);
            writeControlSocket("75L\n");
            chkMDState();
        }
        else if (command.contains("150S"))
        {
            nIndex = 2;
            changeDataRate(nIndex);
            writeControlSocket("150S\n");
            chkMDState();
        }
        else if (command.contains("150L"))
        {
            nIndex = 3;
            changeDataRate(nIndex);
            writeControlSocket("150L\n");
            chkMDState();
        }
        else if (command.contains("300S"))
        {
            nIndex = 4;
            changeDataRate(nIndex);
            writeControlSocket("300S\n");
            chkMDState();
        }
        else if (command.contains("300L"))
        {
            nIndex = 5;
            changeDataRate(nIndex);
            writeControlSocket("300L\n");
            chkMDState();
        }
        else if (command.contains("600S"))
        {
            nIndex = 6;
            changeDataRate(nIndex);
            writeControlSocket("600S\n");
            chkMDState();
        }
        else if (command.contains("600L"))
        {
            nIndex = 7;
            changeDataRate(nIndex);
            writeControlSocket("600L\n");
            chkMDState();
        }
        else if (command.contains("1200S"))
        {
            nIndex = 8;
            changeDataRate(nIndex);
            writeControlSocket("1200S\n");
            chkMDState();
        }
        else if (command.contains("1200L"))
        {
            nIndex = 9;
            changeDataRate(nIndex);
            writeControlSocket("1200L\n");
            chkMDState();
        }
        else if (command.contains("2400S"))
        {
            nIndex = 10;
            changeDataRate(nIndex);
            writeControlSocket("2400S\n");
            chkMDState();
        }
        else if (command.contains("2400L"))
        {
            nIndex = 11;
            changeDataRate(nIndex);
            writeControlSocket("2400L\n");
            chkMDState();
        }
/*
            // Exclude 600V, 0000N, 1200V, 00001N, 2400V - Not implemented.

            nIndex = 12;
            nIndex = 13;
            nIndex = 14;
            nIndex = 15;
            nIndex = 16;
*/
        else if (command.contains("4800U"))
        {
            nIndex = 17;
            changeDataRate(nIndex);
            writeControlSocket("4800U\n");
            chkMDState();
        }
/*
           // Exclude QUERYS and QUERYL
        nIndex = 18;
        nIndex = 19;
*/

           // Start of modem commands.
        else if (command.contains("SEND"))
        {
            writeControlSocket("SENDING\n");
            if(tcpMessageBuffer.length()>0) send(tcpMessageBuffer);
            writeControlSocket("IDLE\n");
        }
/*
        else if (command.contains("KILL"))
        {
            emit killTx();
            writeControlSocket("KILLED\n");

        }
*/
        else if (command.contains("RESET"))
        {
            if (transmitting) emit killTx();
            payload="";
            sm->rx_reset();
            ui->labelTransmitState->setText("IDLE");
            ui->labelTransmitState->setStyleSheet("background-color: lightyellow");
            ui->labelReceiveMode->setText("HUNTING");
            ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");

           writeControlSocket("RESET\n");

        }

        else if (command.contains("MD ON"))
        {
            ui->checkBoxEnableMATCHDR->setChecked(true);
            writeControlSocket("MD ON\n");
        }

        else if (command.contains("MD OFF"))
        {
            ui->checkBoxEnableMATCHDR->setChecked(false);
            writeControlSocket("MD OFF\n");
        }

        else if (command.contains("MD?"))
        {
            if(ui->checkBoxEnableMATCHDR->isChecked())
            {
                writeControlSocket("MD ON\n");
            }
            else
            {
                writeControlSocket("MD OFF\n");
            }
        }

        else if (command.contains("DR?"))
        {
            if(ui->checkBoxEnableMATCHDR->isChecked())
            {
                writeControlSocket("MD ON\n");
                writeControlSocket("DR " + ui->comboBoxSpeedInterleave->currentText().toLatin1() + "\n");
            }
            else
            {
                writeControlSocket("DR " + ui->comboBoxSpeedInterleave->currentText().toLatin1() + "\n");
            }
        }

        else if (command.contains("VR?"))
        {
            writeControlSocket("Version: ");
            writeControlSocket(APPVERSION);
            writeControlSocket(MINOR);
            writeControlSocket(" \n");
        }
}

void MainWindow::chkMDState()
{
    if(ui->checkBoxEnableMATCHDR->isChecked())
    {
        writeControlSocket("MD ON\n");
    }
}

void MainWindow::changeDataRate(int iIndex)
{
    if (!ui->checkBoxEnableMATCHDR->isChecked())                    // If MATCHDR is not active...
    {                                                                                                   //  change the speed.
        ui->labelBEC->setText(QString::number(iIndex));
        Mode val = static_cast<Mode>(iIndex);
        ui->labelBER->setText((QString::number(val)));
        sm->tx_set_mode(val);
        emit txModeChanged(static_cast<Mode>(ui->comboBoxSpeedInterleave->currentIndex()));
        ui->comboBoxSpeedInterleave->setCurrentIndex(iIndex);          // Update the GUI.
        if(!ui->labelTransmitState->text().contains("TRANSMIT"))
        {
            ui->labelTransmitState->setText("IDLE");
            ui->labelTransmitState->setStyleSheet("background-color: lightyellow");
            ui->labelReceiveMode->setText("HUNTING");
            ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");


            writeControlSocket("IDLE\n");                                           // Send an ACK to terminal
        }
        else
        {
            writeControlSocket("BUSY\n");
        }
    }
    else
    {
        writeControlSocket("MD ON\n");
    }
    return;
}



//============================================================================================================================
//**************************************************************************  TCP Functions  ********************************************************************************************
//============================================================================================================================

void MainWindow::onNewDataConnection()
{
    dataSocketIsConnected=false;
    dataSocket = dataServer->nextPendingConnection();
    //qDebug()<<Q_FUNC_INFO<<"new data server connection";
    bool success = connect(dataSocket, SIGNAL(readyRead()), this, SLOT(onDataSocketReadyRead()));
    if(success)
    {
        dataSocketIsConnected=true;
        statusBar()->showMessage( "TCP data socket connected.  ");
        connect(dataSocket, &QTcpSocket::disconnected, this, &MainWindow::slotDataSocketDisconnected);
    }
}

void MainWindow::onDataSocketReadyRead()
{
    tcpMessageBuffer = dataSocket->readAll();
}

void MainWindow::onNewControlConnection()
{
    controlSocketIsConnected=false;
    controlSocket = controlServer->nextPendingConnection();
    //qDebug()<<Q_FUNC_INFO<<"new control server connection";
    bool success = connect(controlSocket, SIGNAL(readyRead()), this, SLOT(onControlSocketReadyRead()));
    if (success) controlSocketIsConnected=true;
    if (modemReady()) writeControlSocket("IDLE\n");
    else
    {
        //qDebug()<<Q_FUNC_INFO<<"Modem Not Ready";
        writeControlSocket("BUSY\n");
    }
}

void MainWindow::onControlSocketReadyRead()
{
    QByteArray command = controlSocket->readAll();
    handleCommands(command);
}

void MainWindow::writeDataSocket(QByteArray messageBytes)
{
    //Proved data socket was working.  In operation the modem does not write to the data socket.  Incoming data from NOSS only.
    //qDebug()<<Q_FUNC_INFO<<"newBytesArray = "<<messageBytes.toHex()<<"\narray size = "<<messageBytes.size();
    //qDebug()<<Q_FUNC_INFO<<messageBytes;
    if(!dataSocketIsConnected)
    {
      //qDebug()<<Q_FUNC_INFO<<"Write attempt to TCP data socket.  Socket not connected";
    }
    else
    {
        dataSocket->write(messageBytes);
        dataSocket->flush();
    }
}

void MainWindow::slotDataSocketDisconnected()
{
    dataSocketIsConnected = false;
    statusBar()->showMessage( "TCP connection disconnected.");
    tcpConnectionTimer.start(2000);
}

void MainWindow::writeControlSocket(QByteArray messageBytes)
{
    if(!controlSocketIsConnected)
    {

    }
    else
    {
        //qDebug()<<Q_FUNC_INFO<<controlSocket->state()<<"\n"<<messageBytes;
        controlSocket->write(messageBytes);
        controlSocket->flush();
    }
}

//*********************************************************************************************************************
//*******************************************************  Settings  **************************************************
//*********************************************************************************************************************

void MainWindow::readSettings()                          //Restores the user's pprevious settings.
{
    //General
        styleSheetFile = settings.value("styleSheetFile", "./Startup Stylesheet.qss").toString();
        loadStyleSheet(styleSheetFile);

    //Menu Items
        autoResetAudioInCodec = settings.value("autoResetAudioInCodec",false).toBool();
        ui->actionAudio_Reset->setChecked(autoResetAudioInCodec);
        toolTipsVisible = settings.value("toolTipsVisible", false).toBool();
        ui->actionTool_Tips->setChecked(toolTipsVisible);
        QStringList cModes;
        logType = settings.value("logType", "dailyLog").toString();
        if(logType.contains("dailyLog"))
        {
            ui->actionSession_Logs->setChecked(false);
            ui->actionDaily_Logs->setChecked(true);
            //Provide a folder for the daily log files.
            QDir logDir("Logs");
            if (!logDir.exists()) { QDir::current().mkpath("Logs");  }
            writeLog("User Action:  Daily Logs option selected.");
            QFile f("./ErrorLog.txt");
            if(f.exists()) { f.remove(); }  //  Remove the session log file.
            logType="dailyLog";
        }
        else
        {
            ui->actionSession_Logs->setChecked(true);
            ui->actionDaily_Logs->setChecked(false);
            QDir logDir("Logs");
            if (logDir.exists())  { logDir.removeRecursively();  }
            writeLog("User Action:  Session Log option selected.");
            logType="sessionLog";
        }

    //Modem Tab

        ui->checkBoxBECAutoReset->setChecked(settings.value("becAutoReset",true).toBool());
        ui->spinBoxBECThreshold->setValue(settings.value("becThreshold",2500).toInt());
        ui->checkBoxEnableMATCHDR->setChecked(settings.value("matchdrChecked").toBool());


    //Audio Tab
       modemModPerCent = settings.value("modulationPct","75").toFloat();
        modemAudioOut->setFloatSampleFactor(modemModPerCent/100);                      //Convert to float

        //Restore selected audio input device.   ----------------Audio input ----------------------------
        QString savedDeviceName = settings.value("inputCODECDeviceName","").toString();                                //See if user previously selected a device.
        QAudioDevice inDevice;  //Check saved device against currently available devices.
        bool validInputDevice = false;
        for (auto &deviceInfo : m_devices->audioInputs())
        {
            if(deviceInfo.description() == savedDeviceName)
            {
                inDevice = deviceInfo;
                ui->comboBoxInputDevice->setCurrentText(savedDeviceName);     //Set the comboBox current text
                validInputDevice = true;
            }
        }
        if (!validInputDevice)
        {
            QMessageBox::information(nullptr,"AudioIn","Select a valid audio input device.");
        }

        //Restore selected audio output device.   ----------------Audio Output ----------------------------
        savedDeviceName = settings.value("outputCODECDeviceName","").toString();                                //See if user previously selected a device.
        QAudioDevice outDevice;  //Check saved device against currently available devices.
        bool validOutputDevice = false;

        for (auto &deviceInfo : m_devices->audioOutputs())
        {
            if(deviceInfo.description() == savedDeviceName)
            {
                outDevice = deviceInfo;
                ui->comboBoxOutputDevice->setCurrentText(savedDeviceName);     //Set the comboBox current text
                validOutputDevice = true;
            }
        }
        if (!validOutputDevice)
        {
            QMessageBox::information(nullptr,"AudioOut","Select a valid audio output device.");
        }

        ui->comboBoxSquelchLevel->setCurrentIndex(settings.value("preambleHuntSquelchLevel", 0).toInt());
        ui->comboBoxAgc->setCurrentIndex(settings.value("agc", 1).toInt());
        ui->comboBoxPmagNr->setCurrentIndex(settings.value("pmagNr", 0).toInt());
        ui->horizontalSliderAudioOut->setValue(settings.value("outputCODECvolume", 10000 ).toInt());    //Restore it.
        ui->horizontalSliderAudioIn->setValue(settings.value("inputCODECvolume", 10000 ).toInt());      //Restore it.
        waitBeforeTones = settings.value("waitBeforeTones",500).toInt();

    //TCP/IP Tab.
        ipv4Address = settings.value("tcpIpAddress", "127.0.0.1").toString();
        quint8 z = 0;
        QStringList slTmp = ipv4Address.split('.');
        foreach (const QString &str, slTmp)
        {
            ++z;
            if (z == 1) {ui->spinBoxIPAddress_P1->setValue(str.toInt());}
            if (z == 2) {ui->spinBoxIPAddress_P2->setValue(str.toInt());}
            if (z == 3) {ui->spinBoxIPAddress_P3->setValue(str.toInt());}
            if (z == 4) {ui->spinBoxIPAddress_P4->setValue(str.toInt());}
        }
        iDataPort = settings.value("tcpDataPort", "63023").toInt();                      // Data port in user space
        if (iDataPort==63023)
        {
            ui->radioButtonModem1->setChecked(true);
            ui->spinBoxCustomDataPort->setValue(63023);
            ui->spinBoxCustomDataPort->setEnabled(false);
        }
        else if (iDataPort==63025)
        {
            ui->radioButtonModem2->setChecked(true);
            ui->spinBoxCustomDataPort->setValue(63025);
            ui->spinBoxCustomDataPort->setEnabled(false);
        }
        else
        {
            ui->radioButtonModem3->setChecked(true);
            ui->spinBoxCustomDataPort->setValue(63027);
            ui->spinBoxCustomDataPort->setEnabled(true);

        }

    //Radio Tab
        ui->comboBoxRadioControlPort->setCurrentText(settings.value("radioControlPort","").toString());
        ui->comboBoxRadioControlPortBaudRate->setCurrentText(settings.value("radioControlPortBaudRate","115200").toString());
        ui->comboBoxRadioControlPortFraming->setCurrentText(settings.value("radioControlPortFraming", "8N1").toString());
        ui->comboBoxRadioControlPortFlowControl->setCurrentText(settings.value("radioControlPortFlowControl", "None").toString());
        ui->comboBoxRadioControlPortPtt->setCurrentText(settings.value("radioControlPortPtt","CAT").toString());
        loadRadioCommands(settings.value("radioCommandsFile","RadioCommands.txt").toString());
        ui->checkBoxMarsAle->setChecked(settings.value("marsAleInUse", false).toBool());

        settings.beginGroup( "Mainwindow" );
        {
            resize(settings.value("size", QSize(346, 390)).toSize());
            move(settings.value("pos", QPoint(200, 200)).toPoint());
        }
        settings.endGroup();

   settingsLoaded = true;
}

void MainWindow::writeSettings()                         //Saves settings.  Others are saved when changed.
{
    //General
        settings.setValue("styleSheetFile", styleSheetFile);

    //Menu Items
        settings.setValue("toolTipsVisible", toolTipsVisible);
        settings.setValue("autoResetAudioInCodec", autoResetAudioInCodec);
        settings.setValue("logType",logType);

    //ModemTab
        settings.setValue("becThreshold",QString::number(ui->spinBoxBECThreshold->value()));
        settings.setValue("becAutoReset",ui->checkBoxBECAutoReset->isChecked());
        settings.setValue("matchdrChecked", ui->checkBoxEnableMATCHDR->isChecked());
        settings.setValue("modemSyncMode", QString::number(static_cast<int>(modemSyncMode)));
        settings.setValue("speedInterleave", ui->comboBoxSpeedInterleave->currentIndex());

    //AudioTab
        if(!ui->comboBoxInputDevice->currentText().isEmpty()) settings.setValue("inputCODECDeviceName", ui->comboBoxInputDevice->currentText() );
        if(!ui->comboBoxOutputDevice->currentText().isEmpty()) settings.setValue("outputCODECDeviceName", ui->comboBoxOutputDevice->currentText() );
        settings.setValue("inputCODECvolume",ui->horizontalSliderAudioIn->value());
        settings.setValue("outputCODECvolume", ui->horizontalSliderAudioOut->value());
        settings.setValue("pmagNr",ui->comboBoxPmagNr->currentIndex());
        settings.setValue("agc",ui->comboBoxAgc->currentIndex());
        settings.setValue("preambleHuntSquelchLevel", ui->comboBoxSquelchLevel->currentIndex());
        settings.setValue("modulationPct", QString::number(modemModPerCent));
        settings.setValue("waitBeforeTones",waitBeforeTones);

    //TCPTab
        settings.setValue("tcpIpAddress", ipv4Address);
        settings.setValue("tcpDataPort", iDataPort);

    //RadioTab
        if(!ui->comboBoxRadioControlPort->currentText().isEmpty()) settings.setValue("radioControlPort", ui->comboBoxRadioControlPort->currentText());
        settings.setValue("radioControlPortBaudRate", ui->comboBoxRadioControlPortBaudRate->currentText());
        settings.setValue("radioControlPortFraming", ui->comboBoxRadioControlPortFraming->currentText());
        settings.setValue("radioControlPortFlowControl", ui->comboBoxRadioControlPortFlowControl->currentText());
        settings.setValue("radioControlPortPtt", ui->comboBoxRadioControlPortPtt->currentText());
        settings.setValue("marsAleInUse", ui->checkBoxMarsAle->isChecked());

    //Form Settings
        settings.beginGroup( "Mainwindow" );
        {
            settings.setValue("size", size());
            settings.setValue("pos", pos());
        }
        settings.endGroup();
}



//*******************************************************************************************************************************
//*******************************************************  Radio Controls  ******************************************************
//*******************************************************************************************************************************

void MainWindow::setRadioSendState()                    //Radio cmmmands before the modem transmits.
{
    QByteArray array ;
    bool hexCommand = false;
    if (ui->comboBoxRadioControlPort->currentText().contains("NONE"))
    {
        showCmdMessage("No radio control selected.");
        showCmdMessage("VOX in use.");
    }
    if (radioControlPort->isOpen())
    {
        ui->plainTextEditRadioCommands->clear();

        if(!txSettings.isEmpty() && !ui->checkBoxMarsAle->isChecked())
        {
            showCmdMessage("Set radio for data.");
            if(txSettings[0].contains("HEX", Qt::CaseInsensitive)) hexCommand = true;;
            for(int i=0;i<txSettings.count();i++)
            {
                //qDebug()<<Q_FUNC_INFO<<"Command line = "<<txSettings[i];
                if(txSettings[i].contains("WAIT",Qt::CaseInsensitive))
                {   //WAIT before sending next command.
                    QStringList part = txSettings[i].split(" ");
                    int waitTime = part[3].toInt();
                    //qDebug()<<Q_FUNC_INFO<<"WAIT milliseconds = "<<waitTime;
                    showCmdMessage("Wait " + QString::number(waitTime) + "ms.");
                    delay(waitTime);
                }
                else                                                       //Send next setting command.
                {
                    array = getCommandString(txSettings[i],3);
                    radioControlPort->write(array.data(),array.size());
                    radioControlPort->flush();
                    if(hexCommand){ showCmdMessage(array.toHex());  }
                    else showCmdMessage(array);
               }
            }
            if(!pttOn.isEmpty())
            {
                if(pttOn.contains("HEX", Qt::CaseInsensitive)) hexCommand = true;
                array = getCommandString(pttOn,3);
                radioControlPort->write(array.data(), array.size());        //PTT on
                radioControlPort->flush();
                if(hexCommand) showCmdMessage(array.toHex());
                else showCmdMessage(array);
            }
        }
        else
        {
            // No CAT support
            showCmdMessage("No CAT support.");
        }


        if(ui->comboBoxRadioControlPortPtt->currentText().contains("RTS"))
        {
            radioControlPort->setRequestToSend(true);
            showCmdMessage("RTS asserted.");
        }

        if(ui->comboBoxRadioControlPortPtt->currentText().contains("DTR"))
        {
            radioControlPort->setDataTerminalReady(true);
            showCmdMessage("DTR asserted.");
        }
        if(ui->comboBoxRadioControlPortPtt->currentText().contains("EXT"))
        {
            showCmdMessage("VOX in use.");
        }
     }
}
void MainWindow::setRadioReceiveState()            //Radio commands after the modem transmits.
{
    if (ui->comboBoxRadioControlPort->currentText().contains("NONE"))
    {
        showCmdMessage("No radio control selected");
        showCmdMessage("VOX PTT ended");
        return;
    }

    showCmdMessage("Set radio for receive.");
    QByteArray array ;
    bool hexCommand = false;
    if(radioControlPort->isOpen())
    {
        if(ui->comboBoxRadioControlPortPtt->currentText().contains("DTR"))
        {
            radioControlPort->setDataTerminalReady(false);
            showCmdMessage("DTR de-asserted");
        }

        if(ui->comboBoxRadioControlPortPtt->currentText().contains("RTS"))
        {
            radioControlPort->setRequestToSend(false);
            showCmdMessage("RTS de-asserted");
        }

        if(!pttOff.isEmpty())
        {
            if(pttOff.contains("HEX", Qt::CaseInsensitive)) hexCommand = true;
            array = getCommandString(pttOff,3);
            radioControlPort->write(array.data(),array.size());        //PTT off
            radioControlPort->flush();
            if(hexCommand) showCmdMessage(array.toHex());
            else showCmdMessage(array);
        }

        if(!rxSettings.isEmpty() && !ui->checkBoxMarsAle->isChecked())
        {
            bool hexCommand = false;
            if(pttOff.contains("HEX", Qt::CaseInsensitive)) hexCommand = true;;

            if(rxSettings[0].contains("HEX", Qt::CaseInsensitive)) hexCommand = true;;
            for(int i=0;i<rxSettings.count();i++)
            {
                if(rxSettings[i].contains("WAIT",Qt::CaseInsensitive))
                {
                    QStringList part = rxSettings[i].split(" ");
                    int waitTime = part[3].toInt();
                    showCmdMessage("Wait " + QString::number(waitTime) + "ms.");
                    delay(waitTime);
                }
                else
                {
                    QByteArray array = getCommandString(rxSettings[i],3);
                    radioControlPort->write(array.data(),array.size());
                    radioControlPort->flush();
                    if(hexCommand){ showCmdMessage(array.toHex());  }
                    else showCmdMessage(array);
                }
            }
        }
        if (controlSocketIsConnected)
        {
            writeControlSocket("RECEIVE\n");
        }
    }
    ui->labelReceiveMode->setText("HUNTING");
    ui->labelReceiveState->setStyleSheet("background-color: lightyellow");
    ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");
    if (controlSocketIsConnected)
    {
        QString sTmp =  (QString::number(getSNRAve,'f',0));
        sTmp = "QL " + sTmp;
        writeControlSocket( sTmp.toLatin1() + "\n");
    }
    getSNRAve = 0;
    initializeAudioIn(inputDevice);
    sampleBufferIn.clear();
    payload.clear();
    sm->m_eomreset = 0;
    sm->eom_rx_reset();
    sm->rx_reset();
    qreal value = ui->horizontalSliderAudioIn->value();
    qreal linearVolume = QAudio::convertVolume(value / qreal(10000),QAudio::LinearVolumeScale,QAudio::LogarithmicVolumeScale);
    if(inputCodecState==CODEC_OPEN)
    {
        audioIn->setVolume(linearVolume);                                           //Update the audio codec.
    }
}

void MainWindow::showCmdMessage(QString msg)
{
    ui->plainTextEditRadioCommands->appendPlainText(msg);
    ui->plainTextEditRadioCommands->ensureCursorVisible();
}

void MainWindow::processTcpRadioCommand(QByteArray command)
{
    //Command format examples = RADIO PTTOn   HEX FE FE 88 e0 1C 00 01 FD
    //                          RADIO TXS     HEX FE FE 88 e0 1A 06 01 01 FD
    //                          RADIO PTTOff  ASCII TX0;
    bool hexCommand;
    hexCommand = false;
    QString c = command;
    if( c.contains("HEX", Qt::CaseInsensitive))hexCommand = true;
    command = getCommandString(c,3);
    if(radioControlPort->isOpen())
    {
        radioControlPort->write(command.data(),command.size());
        radioControlPort->flush();
    }
    if(hexCommand){ showCmdMessage(command.toHex());  }
    else showCmdMessage(command);
}

void MainWindow::loadRadioCommands(QString fileName)                    //Loads the remote (CAT) commands for the radio.
{
    radioCommandsAreHex = false;
    txSettings.clear();
    rxSettings.clear();
    pttOn.clear();
    pttOff.clear();
    radioGlobalSettings.clear();

    if(fileName == "None") return;

    commandFileName = fileName;
    QFile file(commandFileName);
    if(!file.open(QIODevice::ReadOnly))  { return; }

    QString commandLine;
    while(!file.atEnd())
    {
        commandLine = file.readLine();
        //qDebug()<<"Read. "<<commandLine;
        if((commandLine.left(1) != "#") && (!commandLine.trimmed().isEmpty()))
        {
           QRegularExpression rx("[' ']{2,}");
           commandLine.replace(rx," ");
           if(commandLine.contains("RADIO",Qt::CaseInsensitive))  //Assume the lines is a command.
           {
               if(commandLine.contains("SETUP",Qt::CaseInsensitive))        radioGlobalSettings<<commandLine;
               if(commandLine.contains("TXS",Qt::CaseInsensitive))      txSettings<<commandLine;
               if(commandLine.contains("RXS",Qt::CaseInsensitive))      rxSettings<<commandLine;
               if(commandLine.contains("PTTOn", Qt::CaseInsensitive))   pttOn = commandLine;
               if(commandLine.contains("PTTOff", Qt::CaseInsensitive))  pttOff = commandLine;
               if(commandLine.contains("HEX", Qt::CaseInsensitive))     radioCommandsAreHex = true;
           }
        }
    }
    file.close();
    showCommands();
    if (!radioGlobalSettings.isEmpty() && radioControlPort->isOpen())
    {
        sendGlobalSettingsToRadio();
    }
}

QByteArray MainWindow::getCommandString( QString s, int firstCommandWord)        //Packs the ASCII data into a byte array.
{
    //RADIO PTTOff HEX FE FE 98 E0 1C 00 00 FD
    //RADIO PTTOff ASCII TX0;
    int beginningOfComment = s.indexOf("#");
    if(beginningOfComment > 0) s=s.left(beginningOfComment);     //Get rid of any comnent before processing hte command.
    QStringList c = s.split(" ",Qt::SkipEmptyParts,Qt::CaseInsensitive);
    if(!c.contains("HEX",Qt::CaseInsensitive))
    {
        QByteArray s = c[firstCommandWord].toUtf8().trimmed();                  //ASCII command.  Send as is, without white space.
        return(s);
    }
    QByteArray hexString;                                                       //Hex command.
    hexString="";                                                               //The c array is 3 keywords then the command word(s).
    for(int j=firstCommandWord; j<c.count();j++)  {                             //Skip the keywords.
        if(!c[j].isEmpty())  {  hexString.append( c[j].toLocal8Bit().trimmed());  }   //Convert and append the word.
    }
    //qDebug()<<Q_FUNC_INFO<<(QByteArray::fromHex(hexString));
    return (QByteArray::fromHex(hexString));
}

void MainWindow::showCommands()    //Lets the user see if commands were properly loaded.
{
    QString commands=commandFileName;
    extractFilename(commands);

    commands.append("\n");

    for (int i=0;i<radioGlobalSettings.size();i++)
    {
        commands.append("\nRadioSetup = " + radioGlobalSettings[i]);
    }

    for(int i=0;i<txSettings.count();i++)
    {
        commands.append("\nTX Setting = "+txSettings[i]);
    }
    if(!pttOn.isEmpty())  commands.append ("PttOn      = " + pttOn);
    if(!pttOff.isEmpty()) commands.append ("PttOff     = " + pttOff);
    for(int i=0;i<rxSettings.count();i++)
    {
        commands.append("\nRX Setting = "+rxSettings[i]);
    }
    commands.replace("\n\n", "\n");
    ui->plainTextEditRadioCommands->setPlainText(commands);
    radioCommandWindowCursor.atEnd();
    ui->plainTextEditRadioCommands->setTextCursor(radioCommandWindowCursor);
    ui->plainTextEditRadioCommands->ensureCursorVisible();
}

void MainWindow::sendGlobalSettingsToRadio()
{
    if (ui->comboBoxRadioControlPort->currentText()=="NONE")
    {
        return;
    }
    for(int i=0;i<radioGlobalSettings.count(); i++)  qDebug()<<Q_FUNC_INFO<<radioGlobalSettings[i];

    ui->plainTextEditRadioCommands->insertPlainText("Set radio global settings.\n");
    QByteArray array ;
    bool hexCommand = false;

    if(radioControlPort->isOpen())
    {
        if(!radioGlobalSettings.isEmpty() && !ui->checkBoxMarsAle->isChecked())
        {
            if(pttOff.contains("HEX", Qt::CaseInsensitive)) hexCommand = true;

            if(radioGlobalSettings[0].contains("HEX", Qt::CaseInsensitive)) hexCommand = true;
            for(int i=0;i<radioGlobalSettings.count();i++)
            {
                if(radioGlobalSettings[i].contains("WAIT",Qt::CaseInsensitive))
                {
                    QStringList part = radioGlobalSettings[i].split(" ");
                    int waitTime = part[3].toInt();
                    ui->plainTextEditRadioCommands->insertPlainText("Wait " + QString::number(waitTime) + "ms.");
                    ui->plainTextEditRadioCommands->insertPlainText("\n");
                    delay(waitTime);
                }
                else
                {
                    QByteArray array = getCommandString(radioGlobalSettings[i],3);
                    radioControlPort->write(array.data(),array.size());
                    radioControlPort->flush();
                    if(hexCommand)
                    {
                        ui->plainTextEditRadioCommands->insertPlainText(array.toHex());
                    }
                    else ui->plainTextEditRadioCommands->insertPlainText(array);
                    ui->plainTextEditRadioCommands->insertPlainText("\n");
                }
            }
        }
    }
}

void MainWindow::initializeRadioPort()                                             //Serial port for PTT and radio control.
{
    if(radioControlPort->isOpen())radioControlPort->close();
    if (ui->comboBoxRadioControlPort->currentText() == "NONE")
    {
        if(modemReady())
        {
            writeControlSocket("IDLE\n");
        }
        statusBar()->showMessage("No radio control selected");
        return;
    }
    radioControlPort->setPortName(ui->comboBoxRadioControlPort->currentText());

    radioControlPort->setBaudRate(ui->comboBoxRadioControlPortBaudRate->currentText().toInt());

    dataBits = ui->comboBoxRadioControlPortFraming->currentText().left(1);
    parity = ui->comboBoxRadioControlPortFraming->currentText().mid(1,1);
    stopBits = ui->comboBoxRadioControlPortFraming->currentText().right(1);

    if(dataBits == "8") radioControlPort->setDataBits(QSerialPort::Data8);
        else if(dataBits == "7") radioControlPort->setDataBits(QSerialPort::Data7);

    if(ui->comboBoxRadioControlPortFlowControl->currentText() == "None") radioControlPort->setFlowControl(QSerialPort::NoFlowControl);
        else if(ui->comboBoxRadioControlPortFlowControl->currentText() == "RTS/CTS") radioControlPort->setFlowControl(QSerialPort::HardwareControl);
            else if(ui->comboBoxRadioControlPortFlowControl->currentText() == "XON/XOFF") radioControlPort->setFlowControl(QSerialPort::SoftwareControl);
                else QMessageBox::information(nullptr,"QSerialPortError","Flow control selection not found.");

    if(parity == "N") radioControlPort->setParity(QSerialPort::NoParity);
        else if(parity == "E") radioControlPort->setParity(QSerialPort::EvenParity);
            else if(parity == "O") radioControlPort->setParity(QSerialPort::OddParity);
                else QMessageBox::information(nullptr,"QSerialPortError","Parity selection not found.");

    if(stopBits == "1")  radioControlPort->setStopBits(QSerialPort::OneStop);
        else if(stopBits == "2")  radioControlPort->setStopBits(QSerialPort::TwoStop);

    radioControlPort->setFlowControl(QSerialPort::NoFlowControl);  //RTS or DTR is used for PTT.

    if(!ui->comboBoxRadioControlPort->currentText().isEmpty())
    {
        radioControlPort->open(QIODevice::ReadWrite);
        if (radioControlPort->isOpen())
        {
            QString msg = "Radio Port on %1 : %2, %3, %4, %5, %6";
            statusBar()->showMessage( msg.arg(ui->comboBoxRadioControlPort->currentText())
                                         .arg(ui->comboBoxRadioControlPortBaudRate->currentText())
                                         .arg(dataBits)
                                         .arg(parity)
                                         .arg(stopBits)
                                         .arg(ui->comboBoxRadioControlPortFlowControl->currentText() ));
            radioControlPort->setDataTerminalReady(false);
            radioControlPort->setRequestToSend(false);
            if(modemReady())
            {
                writeControlSocket("IDLE\n");
                return;
            }
        }
        else { statusBar()->showMessage("Radio port open error"); }
    }
    else statusBar()->showMessage( "No CAT port selected. ");
    writeControlSocket("BUSY\n");
}

//*****************************************************************************************************************************
//***********************************************  Authorization Tests:  File Age, License  *****************************************
//*****************************************************************************************************************************


bool MainWindow::isTooOld()
{
    int lifeSpan;
    QString version (VERSION);
    if(version == "Release") return false;
    if(version.contains("alpha",Qt::CaseInsensitive)) lifeSpan = ALPHA_TIME;
    if(version.contains("beta",Qt::CaseInsensitive)) lifeSpan = BETA_TIME;

    QFile thisApplication(QApplication::applicationFilePath());
    QFileInfo thisApplicationFileInfo(thisApplication);
    QDateTime now(QDateTime::currentDateTime());
    QDateTime expired(thisApplicationFileInfo.lastModified().addDays(lifeSpan));
    daysRemaining = now.msecsTo(expired)/86400000;     //Calculate days remaining for user query inConfig menu.
    //qDebug()<<Q_FUNC_INFO<<"Difference, days = "<< daysRemaining;
    expirationDate = thisApplicationFileInfo.lastModified().addDays(lifeSpan).toString("ddMMMyyyy"); //Save for Version info in Config menu

    //Check if file has expired.
    if(thisApplicationFileInfo.lastModified().addDays(lifeSpan) < QDateTime::currentDateTime())
    {
        QMessageBox::information(this, "Service Life Test",
            "This application is older than "+QString::number(lifeSpan)+" days past the file date and has expired.\n\nExiting....");
        return true;
    }
    else {return false; }
}//isTooOld

//*******************************************************************************************************************
//***************************                           MenuBar Actions                        **********************
//*******************************************************************************************************************

void MainWindow::loadStyleSheet(QString &styleSheetFile)                                 //Also used by readSettings.
{
    QFile ss (styleSheetFile);
    ss.open( QFile::ReadOnly );
    QString style( ss.readAll() );
    qApp->setStyleSheet( style );
}

//*******************************************************************************************************************
//***************************                    Error Log Functions                           **********************
//*******************************************************************************************************************

void MainWindow::writeLog(QString text)                    //Saves sent and received activity to a log file.
{
    //qDebug()<<Q_FUNC_INFO;
    QDateTime local(QDateTime::currentDateTime());                                    //Get time now.
    QDateTime UTC = local.toUTC();                                         //Convert to UTC (Zulu).
    QString timeStampedMessage ="\n"+UTC.toString("yyyyMMdd hhmmZ ")+title+": "+text;

    QFile logFile(logFileName());
    if(logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        QTextStream stream(&logFile);
        stream<<(timeStampedMessage);
        stream.flush();
    }   logFile.close();
}  //end writeLog

QString MainWindow::logFileName()
{
    //Config option selects a folder of daily logs or just a single file for one session.
    QDir logDir("Logs");
    QString fileName;
    if (logDir.exists())  //Create the fileName for today.
    {
        QDateTime local(QDateTime::currentDateTime());
        QDateTime UTC = local.toUTC();
        fileName = "./Logs/"+ UTC.toString("yyyyMMdd") + "_ErrorLog.txt";

    }
    else fileName = "./ErrorLog.txt";
    return fileName;
}

void MainWindow::extractFilename(QString filename)
{
    if (filename.contains("\\"))
    {
        QStringList list1 = filename.split(u'\\', Qt::SkipEmptyParts);
        QString sTmp = list1.at(list1.size() - 1);
        ui->groupBoxRadioCommands->setTitle(sTmp);
    }
    else if (filename.contains("/"))
    {
        QStringList list1 = filename.split(u'/', Qt::SkipEmptyParts);
        QString sTmp = list1.at(list1.size() - 1);
        ui->groupBoxRadioCommands->setTitle(sTmp);
    }
}

void MainWindow::createIPAddress()
{
    quint8 iIPPart1 = ui->spinBoxIPAddress_P1->value();
    quint8 iIPPart2 = ui->spinBoxIPAddress_P2->value();
    quint8 iIPPart3 = ui->spinBoxIPAddress_P3->value();
    quint8 iIPPart4 = ui->spinBoxIPAddress_P4->value();
    QString sTmp = QString::number(iIPPart1) + "." + QString::number(iIPPart2) + "." + QString::number(iIPPart3) + "." + QString::number(iIPPart4);
    ipv4Address = sTmp;
}

//****************************************************************************************************************************************************
//**************************************************************** Menu Actions *******************************************************************
//************************************************************* New menu structure ***************************************************************
//****************************************************************************************************************************************************

void MainWindow::on_actionAbout_triggered()
{
    QString version (APPVERSION);
    QString minor (MINOR);
    QString type (VERSION);
    QString strAbout;
    strAbout += "AF-DM version: ";
    strAbout += version;
    strAbout += minor + " ";
    strAbout += type;
    strAbout += "\n\n\"We all stand on the shoulders of giants...\"";
    strAbout += "\n                    - Bernard of Chartres, 12th Century.";
    strAbout += "\nBased on the AMARS MS-DMT 3.x version and ";
    strAbout += "\nmodified to meet AFMARS requirements.";
    strAbout += "\nThe work of numerous individuals is reflected in ";
    strAbout += "\nthis code. \nPrincipal credit goes to the following people:";
    strAbout += "\n        Steve Hajducek, N2CKH.";
    strAbout += "\n        Bob Baker, K5LLF.";
    strAbout += "\n        Charles Brain, G4GUO.";
    strAbout += "\n        and other SDT team members.";
    strAbout += "\nAFMARS modifications and additions incorporated";
    strAbout += "\nin AF-DM by:\n          Perry Ogletree, AFA4NQ.";
    strAbout += "\nAF-DM is written in Qt 6 Open Source and the ";
    strAbout += "\nsource is available with a written request.";
    strAbout += "\nThe modem core library is written in pure C++ ";
    strAbout += "\nand is closed code. The source is not available";
    strAbout += "\nand requests for it will be ignored.";
    QMessageBox::information(this, "About AF-DM", strAbout);
}

void MainWindow::on_actionAudio_Reset_triggered()
{
    if(ui->actionAudio_Reset->isChecked())
    {
        autoResetAudioInCodec = true;
        audioInCodecResetTimer.start(3000);
        writeLog("User Action:  Enabled auto reset of audioIn.");
    }
    else
    {
        autoResetAudioInCodec = false;
        audioInCodecResetTimer.stop();
        writeLog("User Action:  Disabled auto reset of audioIn.");
    }
}

void MainWindow::on_actionDaily_Logs_triggered()
{
    if (ui->actionDaily_Logs->isChecked())
    {
        ui->actionSession_Logs->setChecked(false);
        //qDebug()<<Q_FUNC_INFO;
        //Provide a folder for the daily log files.
        QDir logDir("Logs");
        if (!logDir.exists()) { QDir::current().mkpath("Logs");  }
        writeLog("User Action:  Daily Logs option selected.");
        QFile f("./ErrorLog.txt");
        if(f.exists()) { f.remove(); }  //  Remove the session log file.
    }
    else
    {
        ui->actionSession_Logs->setChecked(true);
        QDir logDir("Logs");
        if (logDir.exists())  { logDir.removeRecursively();  }
        writeLog("User Action:  Session Log option selected.");
    }
}

void MainWindow::on_actionExit_triggered()
{
    QApplication::exit(0);
}

void MainWindow::on_actionHelp_triggered()
{
    QString sDirPath = QCoreApplication::applicationDirPath();
    QUrl pdfUrl = QUrl::fromLocalFile(sDirPath + "/Manuals/AF-DM User Guide.pdf");
    QDesktopServices::openUrl(pdfUrl);
}

void MainWindow::on_actionLog_File_triggered()
{
    QProcess::startDetached("C:\\Windows\\system32\\notepad.exe",  {logFileName()} );
}

void MainWindow::on_actionSession_Logs_triggered()
{
    if (ui->actionSession_Logs->isChecked())
    {
        ui->actionDaily_Logs->setChecked(false);
        //qDebug()<<Q_FUNC_INFO;
        QDir logDir("Logs");
        if (logDir.exists())  { logDir.removeRecursively();  }
        writeLog("User Action:  Session Log option selected.");
    }
    else
    {
        ui->actionDaily_Logs->setChecked(true);
        //qDebug()<<Q_FUNC_INFO;
        //Provide a folder for the daily log files.
        QDir logDir("Logs");
        if (!logDir.exists()) { QDir::current().mkpath("Logs");  }
        writeLog("User Action:  Daily Logs option selected.");
        QFile f("./ErrorLog.txt");
        if(f.exists()) { f.remove(); }  //  Remove the session log file.

    }
}

void MainWindow::on_actionStylesheet_triggered()
{
    QString sDirPath = QCoreApplication::applicationDirPath();
    sDirPath += "/StyleSheets";
    styleSheetFile = QFileDialog::getOpenFileName(this, "Select StyleSheet File.", sDirPath,"StyleSheets (*.qss)" );
    if (!styleSheetFile.isEmpty())
    {
        QFile file(styleSheetFile);
        if(!file.open(QIODevice::ReadOnly))
        {
            QMessageBox::information(this, "File Open Error", file.errorString());
        }
        loadStyleSheet(styleSheetFile);
        file.close();
    }
}

void MainWindow::on_actionSummary_triggered()
{
    QString name;
    QString choices;
    choices += "\nTool Tips: "; if (toolTipsVisible) choices +=  "Enabled"; else choices += "Off";
    choices += "\nAutoResetAudioIn: "; if (autoResetAudioInCodec) choices +=  "Yes"; else choices += "No";
    choices +="\nLog Type:  ";
    QDir logDir("Logs"); if (logDir.exists()) choices += "Daily"; else choices += "Session";
    if(commandFileName.isEmpty()) name = "None selected.";
    else name = commandFileName.right(commandFileName.size()-commandFileName.lastIndexOf("/")-1);
    choices +="\nRadio Commands: " + name + ".";
    QMessageBox::information(this,"Current Selections", choices);
}

void MainWindow::on_actionTool_Tips_triggered()
{
    if(ui->actionTool_Tips->isChecked())
    {
        toolTipsVisible = true;
    }
    else
    {
         toolTipsVisible = false;
    }
}

void MainWindow::on_checkBoxEnableMATCHDR_clicked(bool checked)
{
    if (controlSocketIsConnected)
    {
        if (checked)
        {
            writeControlSocket("MD ON\n");
        }
        else
        {
            writeControlSocket("MD OFF\n");
        }
    }
}

//****************************************************************************************************************************************************
//****************************************************************** Combo Actions ***************************************************************
//****************************************************************************************************************************************************

void MainWindow::on_comboBoxAgc_currentIndexChanged(int index)
{
    //qDebug()<<Q_FUNC_INFO<<index;
    switch (index)
    {
    case 0: sm->set_agc_state(0);
        //qDebug()<<Q_FUNC_INFO<<"case 0"<<index;
        break;      //Off    Per old MS-DMT Dlg
    case 1: sm->set_agc_state(2);
        //qDebug()<<Q_FUNC_INFO<<"case 1"<<index;
        break;      //High   Per old MS-DMT Dlg
    case 2: sm->set_agc_state(1);
        //qDebug()<<Q_FUNC_INFO<<"case 2"<<index;
        break;      //Low    Per old MS-DMT Dlg
    }
}

//Audio Input/Output User Selections

void MainWindow::on_comboBoxInputDevice_currentTextChanged(const QString &arg1) //Changes the audio input device.
{
    //qDebug()<<Q_FUNC_INFO<<"Entered.";
    QString deviceText=arg1;
    if(inputCodecState == CODEC_OPEN)                                               //If a device was open,
    {
        audioIn->stop();                                                              //stop the current device,
        audioIn->disconnect(this);                                                    //disconnect from slots,
    }
    inputDevice=getInputAudioDevice(deviceText);                                //look up the new device info.
    initializeAudioIn(inputDevice);                                             //Initialize audioIn.
}

void MainWindow::on_comboBoxOutputDevice_currentTextChanged(const QString &arg1)   //Changes the audio output device.
{
    //qDebug()<<Q_FUNC_INFO<<"Entered"<<arg1;
    validOutputDevice = false;
    if(!arg1.isEmpty())
    {
        QString deviceText = arg1;
        outputDevice=getOutputAudioDevice(deviceText);
        modemAudioOut->setAudioOutputDevice(outputDevice);                     //Reinitializes audioOut with new device.
        validOutputDevice = true;
        if(modemReady()) writeControlSocket("IDLE\n");
    }
    else
    {
        QMessageBox::information(nullptr, "Output Device", "Cannot be blank.  Select an output device.");
        writeControlSocket("BUSY\n");
    }
}

void MainWindow::on_comboBoxPmagNr_currentIndexChanged(int index)
{
    switch (index)
    {
    case 0: sm->set_noise_reduction_state(1); break;      //Off    Per old MS-DMT Dlg
    case 1: sm->set_noise_reduction_state(0); break;      //Low    Per old MS-DMT Dlg
    }
}

//Radio Control Port (serial port) User Selections

void MainWindow::on_comboBoxRadioControlPort_currentTextChanged(const QString &arg1) { Q_UNUSED(arg1); initializeRadioPort(); }

void MainWindow::on_comboBoxRadioControlPortBaudRate_currentTextChanged(const QString &arg1) { Q_UNUSED(arg1); initializeRadioPort(); }

void MainWindow::on_comboBoxRadioControlPortFlowControl_currentTextChanged(const QString &arg1)  {Q_UNUSED(arg1); initializeRadioPort(); }

void MainWindow::on_comboBoxRadioControlPortPtt_currentTextChanged(const QString &arg1) {Q_UNUSED(arg1); initializeRadioPort(); }

//Modem User Selections

void MainWindow::on_comboBoxSpeedInterleave_currentIndexChanged(int index)              //Set data mode (speed/interleave) for modem.
{
    //qDebug()<<Q_FUNC_INFO<<"Index = "<<index;
    sm->tx_set_mode(static_cast<Mode>(index));
    emit txModeChanged(static_cast<Mode>(ui->comboBoxSpeedInterleave->currentIndex()));
}

//Audio Processing User Selections

void MainWindow::on_comboBoxSquelchLevel_currentIndexChanged(int index)
{
    //qDebug()<<Q_FUNC_INFO<<index;
    sm->set_preamble_hunt_squelch(8+index);                //Index = 0,1 or 2, so squelch is set to 8, 9, or 10.
}

//****************************************************************************************************************************************************
//**************************************************************** Slider Actions ***************************************************************
//****************************************************************************************************************************************************

//Functions moved to ModemAudioOut class to runs in separate thread.

void MainWindow::on_horizontalSliderAudioOut_valueChanged(int value)
{
    ui->labelTxVolume->setText(QString::number(value/100));                             //Show value.
    qreal linearVolume = value/qreal(10000);
    modemAudioOut->setAudioOutVolume(linearVolume);                                 //Update the audio codec.
}

void MainWindow::on_horizontalSliderAudioIn_valueChanged(int value)
{
    ui->labelRxVolume->setText(QString::number(value/100));
    qreal linearVolume = QAudio::convertVolume(value / qreal(10000),QAudio::LinearVolumeScale,QAudio::LogarithmicVolumeScale);
    if(inputCodecState==CODEC_OPEN)
    {
        audioIn->setVolume(linearVolume);                                           //Update the audio codec.
    }
}

//****************************************************************************************************************************************************
//**************************************************************** Line Edit Actions ***************************************************************
//****************************************************************************************************************************************************

void MainWindow::on_lineEditTestCommand_returnPressed()
{
    on_pushButtonSendTestCommand_clicked();
}

//****************************************************************************************************************************************************
//**************************************************************** Push Button Actions ***********************************************************
//****************************************************************************************************************************************************

void MainWindow::on_pushButtonCancelCommandFile_clicked()
{
    txSettings.clear();
    rxSettings.clear();
    pttOn.clear();
    pttOff.clear();
    settings.setValue("radioCommandsFile", "None");
    commandFileName.clear();
    ui->plainTextEditRadioCommands->clear();
}

void MainWindow::on_pushButtonClearCommandWindow_clicked()
{
    ui->plainTextEditRadioCommands->clear();
}

void MainWindow::on_pushButtonResetRx_clicked()
{
    if(transmitting) {emit killTx();}
    writeLog("User Action:  Reset AudioIn and set receive modem.");
    initializeAudioIn(inputDevice);
    sampleBufferIn.clear();
    payload.clear();
    sm->m_eomreset = 0;
    sm->eom_rx_reset();
    sm->rx_reset();
    ui->labelReceiveMode->setText("HUNTING");
    ui->labelReceiveState->setStyleSheet("background-color: lightyellow");
    ui->labelReceiveMode->setStyleSheet("background-color: lightyellow");
}

void MainWindow::on_pushButtonSelectRadioCommandFile_clicked()
{
    QString fn = QFileDialog::getOpenFileName(this, "Select File.", "./RadioCommandsLibrary", "RadioCommand files (*.txt)" );
    if (!fn.isEmpty())
    {
        QFile file(fn);
        if(!file.open(QIODevice::ReadOnly))
        {
            QMessageBox::information(this, "File Open Error", file.errorString());
        }
        settings.setValue("radioCommandsFile", fn);
        extractFilename(fn);
        loadRadioCommands(fn);
        showCommands();
        file.close();
    }
}

void MainWindow::on_pushButtonSendTestCommand_clicked()
{
    if (ui->comboBoxRadioControlPort->currentText()=="NONE")
    {
        return;
    }
    if(ui->lineEditTestCommand->text().isEmpty()) return;
    if(radioControlPort->isOpen())
    {
        QByteArray a;
        if(ui->lineEditTestCommand->text().contains("HEX", Qt::CaseInsensitive))
        {
            a= getCommandString(ui->lineEditTestCommand->text().toUtf8(),1);
            radioControlPort->write(a, a.size());
            radioControlPort->flush();
            showCmdMessage("Sent       = "+a.toHex());
        }
        else
        {
            a= getCommandString(ui->lineEditTestCommand->text().toUtf8(),0);
            radioControlPort->write(a, a.size());
            radioControlPort->flush();
            showCmdMessage("Sent       = "+a);
        }
        radioCommandWindowCursor.atEnd();
        ui->plainTextEditRadioCommands->setTextCursor(radioCommandWindowCursor);
        ui->plainTextEditRadioCommands->ensureCursorVisible();
    }
}

void MainWindow::on_radioButtonModem1_clicked(bool checked)
{
    if (checked)
    {
        dataPort = "63023";
        iDataPort = 63023;
        controlPort = "63024";
        iControlPort = 63024;
        ui->spinBoxCustomDataPort->setValue(iDataPort);
        ui->spinBoxCustomDataPort->setEnabled(false);
    }
}

void MainWindow::on_radioButtonModem2_clicked(bool checked)
{
    if (checked)
    {
        dataPort = "63025";
        iDataPort = 63025;
        controlPort = "63026";
        iControlPort = 63026;
        ui->spinBoxCustomDataPort->setValue(iDataPort);
        ui->spinBoxCustomDataPort->setEnabled(false);
    }
}

void MainWindow::on_radioButtonModem3_clicked(bool checked)
{
    if (checked)
    {
        quint16 iTmp = 63027;
        dataPort = QString::number(iTmp);
        iDataPort = iTmp;
        controlPort = QString::number(iTmp + 1);
        iControlPort = static_cast<quint16>(iTmp + 1);
        ui->spinBoxCustomDataPort->setValue(iTmp);
        ui->spinBoxCustomDataPort->setEnabled(true);
    }
}

void MainWindow::on_spinBoxCustomDataPort_editingFinished()
{
    if (ui->radioButtonModem3->isChecked())
    {
        quint16 arg1 = ui->spinBoxCustomDataPort->value();
        if ((arg1  >= 63023) && (arg1 <= 63025))
        {
            QMessageBox::information(nullptr,"Selection Error","Use Modem #1 or #2 instead.");
            arg1=63027;
        }
        dataPort = QString::number(arg1);
        iDataPort = arg1;
        controlPort = QString::number(arg1 + 1);
        iControlPort = (arg1 + 1);
    }
}

void MainWindow::on_spinBoxIPAddress_P1_editingFinished()
{
    createIPAddress();
}

void MainWindow::on_spinBoxIPAddress_P2_editingFinished()
{
    createIPAddress();
}

void MainWindow::on_spinBoxIPAddress_P3_editingFinished()
{
    createIPAddress();
}

void MainWindow::on_spinBoxIPAddress_P4_editingFinished()
{
    createIPAddress();
}

