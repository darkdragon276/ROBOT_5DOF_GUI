#ifndef ROBOTCONTROLL_H
#define ROBOTCONTROLL_H

#include <QObject>
#include <QSerialPort>
#include <QDebug>
#include <QTimer>
#include <QEventLoop>

#include <stdio.h>
#include <string.h>
#include <sstream>

#include "imageprocess.h"

using namespace std;

class RobotControll : public QSerialPort
{
    Q_OBJECT

public:
    explicit RobotControll(QObject *parent = nullptr);
    virtual ~RobotControll();
    string qbyteArray2string(QByteArray &data);
public:
    enum robotCommand_t {
        SetHome = 0,
        SetPosition,
        SetWidth,
        SetDuty,
        SetPositionWithArg,
        SetTime,
        SetWidPos,
        SetPositionArgWid,
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
        "SETWIDPOS",
        "SETPOSANGWID",
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
    bool setCommandNWait(robotCommand_t cmd, const QString para = "");

public:
    bool setCommand(robotCommand_t cmd, int time, const QString para = "");
    robotStatus_t getStatus();
    void resetId();
    bool isTimeOut();
    void setWidthNPosition(Point2f pos, int time, double Width, Point2f base_center);

private:
    int id_command = 1;
    robotStatus_t robot_stt = RobotIdle;
    QTimer *timeout;
    bool istimeout = false;

};

#endif // ROBOTCONTROLL_H
