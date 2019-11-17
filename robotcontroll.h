#ifndef ROBOTCONTROLL_H
#define ROBOTCONTROLL_H

#include <QObject>
#include <QSerialPort>
#include <QDebug>
#include <QTimer>

#include <stdio.h>
#include <string.h>
#include <sstream>

#define NDEBUG(message)  RobotControll::deBug(__FILE__, __LINE__, __FUNCTION__, message)

using namespace std;

class RobotControll : public QSerialPort
{
    Q_OBJECT

public:
    explicit RobotControll(QObject *parent = nullptr);
    virtual ~RobotControll();

    void deBug(string file, int line, string function, string message);
    string qbyteArray2string(QByteArray &data);
public:
    enum robotCommand_t {
        SetHome = 0,
        SetPosition,
        SetWidth,
        SetDuty,
        SetPositionWithArg,
        SetTime,
        Save,
        NumOfCmd,
    };
    const char* ROBOTCOMMAND[NumOfCmd] = {
        "SETHOME",
        "SETPOS",
        "SETWID",
        "SETDUTY",
        "SETPOSNARG",
        "SETTIME",
        "SAVE"
    };
    enum robotStatus_t {
        RobotIdle = 0,
        RobotSending,
        RobotDone,
        RobotProc,
        RobotReq,
        RobotErrArg,
        RobotErr,
        NumOfStt,
    };
    const char* ROBOTSTATUS[NumOfStt] = {
        "IDLE",
        "SENDING",
        "DONE",
        "PROCESSING",
        "REQUEST",
        "ERROR ARGUMENT",
        "ERROR",
    };

public:
    bool setCommand(robotCommand_t cmd, int time, const QString para = "");
    robotStatus_t getStatus();
    void resetId();
    bool isTimeOut();
signals:
    void commandTimeOut();
    void commandWorkDone();

private:
    bool packData(QByteArray &data);
    bool unPackData(QByteArray &data);
    bool writeData(QByteArray &data);
    void readData();
    void timeOut();
    void setStatus(QString response);

private:
    int id_command = 1;
    robotStatus_t robot_stt = RobotIdle;
    QTimer *timeout;
    bool istimeout = false;

};

#endif // ROBOTCONTROLL_H
