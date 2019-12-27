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
    void getObsetPosition(Point2f pos, Point3f &obset);

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
    vector<vector<Point3f>> robot_obset = {{Point3f(1.2, 0.1, 1.3), Point3f(0.8, -0.1, 1.5), Point3f(0.8, -0.1, 1.3),
                                            Point3f(0.8, 0.0, 1.3), Point3f(0.6, 0.2, 1.0), Point3f(0.5, 0.2, 1.0),
                                            Point3f(0.4, 0.2, 1.2), Point3f(0.0, 0.4, 1.7), Point3f(0.0, 0.4, 1.7)},// row 1

                                           {Point3f(1.4, 0.2, 1.5), Point3f(1.4, 0.2, 1.5), Point3f(1.2, 0.0, 1.3),
                                            Point3f(1.0, 0.4, 1.3), Point3f(0.8, 0.4, 1.3), Point3f(0.5, 0.6, 1.1),
                                            Point3f(0.3, 0.6, 1.3), Point3f(0.0, 0.6, 1.9), Point3f(0.0, 0.7, 2.1)},// row 2

                                           {Point3f(1.4, 0.2, 1.5), Point3f(1.5, 0.1, 1.8), Point3f(1.3, 0.3, 1.8),
                                            Point3f(1.1, 0.5, 1.8), Point3f(0.8, 0.7, 1.8), Point3f(0.7, 0.9, 1.8),
                                            Point3f(0.3, 1.1, 1.9), Point3f(0.2, 1.0, 1.9), Point3f(0.0, 0.9, 2.1)},// row 3

                                           {Point3f(1.4, 0.0, 1.5), Point3f(1.5, -0.1, 1.6), Point3f(1.2, 0.2, 1.7),
                                            Point3f(1.1, 0.6, 1.9), Point3f(1.1, 0.8, 2.1), Point3f(0.5, 1.0, 2.1),
                                            Point3f(0.3, 1.0, 1.7), Point3f(0.2, 1.0, 1.7), Point3f(0.0, 0.4, 1.9)},// row 4

                                           {Point3f(1.4, -0.2, 1.4), Point3f(1.3, -0.3, 1.4), Point3f(1.4, 0.0, 1.7),
                                            Point3f(1.1, 0.4, 1.7), Point3f(1.1, 0.6, 1.7), Point3f(0.6, 0.6, 1.7),
                                            Point3f(0.3, 0.6, 1.7), Point3f(0.4, 0.6, 1.4), Point3f(0.2, 0.3, 1.4)},// row 5

                                           {Point3f(1.3, -0.4, 1.0), Point3f(1.3, -0.3, 1.2), Point3f(1.4, -0.1, 1.2),
                                            Point3f(1.1, 0.1, 1.2), Point3f(1.3, 0.1, 1.2), Point3f(0.7, 0.1, 1.2),
                                            Point3f(0.5, 0.1, 1.2), Point3f(0.5, 0.3, 1.2), Point3f(0.4, 0.1, 1.2)},// row 6

                                           {Point3f(1.7, -0.8, 0.9), Point3f(1.7, -0.6, 0.9), Point3f(1.5, -0.6, 0.9),
                                            Point3f(1.3, -0.4, 1.0), Point3f(1.3, -0.2, 1.0), Point3f(0.9, -0.2, 1.0),
                                            Point3f(0.6, 0.0, 1.1), Point3f(0.6, 0.0, 1.1), Point3f(0.6, -0.2, 0.9)},// row 7
                                           };

};

#endif // ROBOTCONTROLL_H
