#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "globals.h"
#include "modemAudioOut.h"  // Not referenced but needed.
#include "qtimer.h"
#include <bridge.h>

#define MODEM_SAMPLE_RATE  48000  //From soundcard

#include <QMenuBar>
#include <QMainWindow>
#include <QAudioDevice>
#include <QMediaDevices>
#include <QAudioSink>
#include <QAudioSource>
#include <QMutex>
#include <QIODevice>
#include <QTextCursor>
#include <QSysInfo>
#include <QOperatingSystemVersion>
#include <QCoreApplication>
#include <QtGui>
#include <QSettings>
#include <QMessageBox>
#include <QFileDialog>
#include <QtConcurrent>
#include <QtEndian>
#include <QtMath>
#include <QVector>
#include <QNetworkInterface>
#include <QNetworkAddressEntry>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPointF>
#include <QVector>

typedef enum{
    SYNC_EOM_MODE,
    ASYNC_MODE,
    SYNC_MODE
}SyncMode;



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    static void status(ModemStatus status, void* param);
    static void output_data_octet( U8 octet );




signals:

    void newMessageBytes(QByteArray);
    void newAudioInSamples();
    void newModemStatus(int, void *);
    void sendMsg(QByteArray);
    void audioOutVolumeChanged(qreal);
    void startOutThread();
    void valueChanged(const QString &value);
    void finished();
    void cleanup();
    void startTxModem();
    void resizeMainWindow();
    void txModeChanged(Mode);
    void killTx();

public slots:

    void bytesReceived(QByteArray);
    void handleAudioInData();
    void handleAudioInStateChanged(QAudio::State newState);
    void handleModemStatusChanges(int status, void *);
    void onNewDataConnection();
    void onDataSocketReadyRead();
    void onNewControlConnection();
    void onControlSocketReadyRead();
    void cleanupAfterSend();



private slots:

    void on_comboBoxInputDevice_currentTextChanged(const QString &arg1);
    void on_comboBoxSpeedInterleave_currentIndexChanged(int index);
    void on_comboBoxOutputDevice_currentTextChanged(const QString &arg1);
    void on_comboBoxRadioControlPortFlowControl_currentTextChanged(const QString &arg1);
    void on_comboBoxRadioControlPortBaudRate_currentTextChanged(const QString &arg1);
    void on_comboBoxRadioControlPortPtt_currentTextChanged(const QString &arg1);
    void on_comboBoxRadioControlPort_currentTextChanged(const QString &arg1);
    void on_comboBoxSquelchLevel_currentIndexChanged(int index);

    void on_pushButtonCancelCommandFile_clicked();
    void on_pushButtonClearCommandWindow_clicked();
    void on_pushButtonResetRx_clicked();
    void on_pushButtonSelectRadioCommandFile_clicked();
    void on_pushButtonSendTestCommand_clicked();

    void on_lineEditTestCommand_returnPressed();

    void processAudioInSamples();
    void slotDataSocketDisconnected();
    void tabSelected();

    void on_horizontalSliderAudioIn_valueChanged(int value);
    void on_horizontalSliderAudioOut_valueChanged(int value);

    void on_comboBoxAgc_currentIndexChanged(int index);
    void on_comboBoxPmagNr_currentIndexChanged(int index);

    void changeDataRate(int iIndex);

    void handleAudioInCodecDisconnect();

    void on_actionAbout_triggered();

    void on_actionHelp_triggered();

    void on_actionExit_triggered();

    void on_actionTool_Tips_triggered();

    void on_actionAudio_Reset_triggered();

    void on_actionDaily_Logs_triggered();

    void on_actionSession_Logs_triggered();

    void on_actionLog_File_triggered();

    void on_actionStylesheet_triggered();

    void on_actionSummary_triggered();

    void chkMDState();

    void on_checkBoxEnableMATCHDR_clicked(bool checked);

    void on_spinBoxIPAddress_P1_editingFinished();

    void on_spinBoxIPAddress_P2_editingFinished();

    void on_spinBoxIPAddress_P3_editingFinished();

    void on_spinBoxIPAddress_P4_editingFinished();

    void on_radioButtonModem1_clicked(bool checked);

    void on_radioButtonModem2_clicked(bool checked);

    void on_radioButtonModem3_clicked(bool checked);

    void on_spinBoxCustomDataPort_editingFinished();

private:
    Ui::MainWindow *ui;

    bool event(QEvent *ev);
    bool eventFilter(QObject *obj, QEvent *event);
    void closeEvent(QCloseEvent *event);

    void delay(quint16);

    void initializeModem();
    void matchDataRate();
    void readSettings();
    void writeSettings();
    void send(QByteArray);
    void newStatusCallback(int, void *);

    void setRadioSendState();
    void setRadioReceiveState();

    QAudioDevice getOutputAudioDevice(QString &name);
    QAudioDevice getInputAudioDevice(QString &name);
    void initializeAudioIn(QAudioDevice inputDevice);
    void lookForSyncEomSequence(U8 octet);

    void resetAudioOut();
    void addToMessageBuffer(unsigned char );
    void newDataOctet( U8 octet );
    void flushMessageBuffer();

    void initializeTcp();
    void writeDataSocket(QByteArray);
    void writeControlSocket(QByteArray);
    void handleCommands(QByteArray);
    void setTcpIpAddress();
    void initializeRadioPort();
    QByteArray getCommandString( QString, int );
    void loadRadioCommands(QString);
    void processTcpRadioCommand(QByteArray);
    void showCommands();
    void showCmdMessage(QString msg);
    void extractFilename(QString filename);
    void sendGlobalSettingsToRadio();
    void createIPAddress();

    bool isTooOld();
    void loadStyleSheet(QString &);


    qint64 daysRemaining;
    QString expirationDate;

    bool modemReady();
    bool validOutputDevice {false};
    void writeLog(QString text);
    QString logFileName();

    bool enableRxLevelMeter {false};

    QString styleSheetFile;

    QString title = TITLE;

    bool toolTipsVisible {false};
    bool minimizeSize {false};                                             //Keeps screen size small when changing tabs.
    bool manualVOXTest {false};
    QString logType = "dailyLog";
    qreal dpr;                                                             //Device pixel ratio for scaling the constellation display.
    QGraphicsScene scene;

    QTimer tcpConnectionTimer;
    QTimer eomTimer;
    QTimer audioInCodecResetTimer;

    bool autoResetAudioInCodec {false};

    QByteArray bytesFromRadio;
    bool radioCommandsAreHex;
    QTextCursor radioCommandWindowCursor;

    QThread * modemAudioOutThread;

    CodecState inputCodecState;                                             //Open or closed.  Set by initializeAudioIn/Out after initialization.
    SyncMode modemSyncMode;                                                 //SYNC_EOM, SYNC, or ASYNC.
    int modemStatus;
    float modemModPerCent;

    QAudioFormat format;
    QAudioDevice outputDevice;                                              //Current output device selected by user
    QAudioDevice inputDevice;                                               //Current input device selected by user
    QAudioSource * audioIn;                                                 //Internal QAudioIn input device tied to CODEC.

    QIODevice * audioInDevice;                                              //Pointer to internal QtAudioInput device
    QByteArray chunk;                                                       //Litle endian bytes to be sent to QAudioOutput
    QByteArray sampleBufferIn;                                              //Holds all incoming little endian samples received from audioIn.
    unsigned char audioBlock[19200];                                        //9600 2-byte big endian samples from audio input for modem to convert to float.
    signed short decimatedAudioBlock[1920];
    int lenDecimated;
    QMutex inBufferMutex;

    QByteArray payload;                                                     //EOM search array for entire data stream, including message and eom.
    int payloadLength {0};                                                  //Number of octets in the payload.
    QByteArray eomBytes;                                                    //The eom search string.
    QByteArray msgBuffer;                                                   //Buffer to accumulate packets for sending to the terminal via TCP.
    quint16 msgBufferLimit {32760};                                                //Number of bytes +1 of hte message to be written to the terminal each write.
    //QString statusEnumString[12] {"DCD_TRUE","DCD FALSE","TX_TRUE","TX_FALSE","FREQ_ERROR","SNR","PREAM_SNR","VITERBI","TRAIN","TEXT","BER","BEC"};
    QTextCursor receiveWindowCursor;
    QByteArray testData;
    quint16 waitBeforeTones {500};

    //TCP stuff
    QString ipv4Address {"127.0.0.1"};
    QString dataPort {"63023"};
    quint16 iDataPort{63023};
    QString controlPort {"63024"};
    quint16 iControlPort {63024};

    QByteArray tcpMessageBuffer;                                            //Stores msg bytes from NOSS until it sends the SEND BUFFER command.
    QTcpSocket *dataSocket;
    QTcpSocket *controlSocket;

    QTcpServer *dataServer;
    QTcpServer *controlServer;
    bool controlSocketIsConnected {false};
    bool dataSocketIsConnected {false};

    QSerialPort * radioControlPort;
    QStringList tcpCommand;
    QStringList cm110sDataRate;

    QString commandFileName;
    QStringList txSettings;
    QStringList rxSettings;
    QString pttOn;
    QString pttOff;

    QString user;

    float snrs[10] {0};
    int snrsIndex {0};
    QByteArray messageBytes;
    float restingSnr;
    float getSNRAve;
    bool transmitting{false};
    float previousFER {0};
    bool settingsLoaded {false};
    QStringList radioGlobalSettings;


    QString dataBits, parity, stopBits;

    QPoint posX, posY;

};
#endif // MAINWINDOW_H
