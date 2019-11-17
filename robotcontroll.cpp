#include "robotcontroll.h"

RobotControll::RobotControll(QObject *parent):QSerialPort(parent),
    timeout(new QTimer(this))
{
    connect(this, &QSerialPort::readyRead, this, &RobotControll::readData);
    connect(timeout, &QTimer::timeout, this, &RobotControll::timeOut);
}

RobotControll::~RobotControll()
{
    delete timeout;
}

bool RobotControll::packData(QByteArray &data)
{
    if(data.isNull() || data.isEmpty()) {
        NDEBUG("data input is empty");
        return false;
    }
    // packing
    QByteArray temp;
    temp.append(data);
    int len = temp.length();
    temp.push_front((char)0x7E);
    while(len) {
        int i = temp.length()-len;
        if (temp.at(i) == (char)0x7D || temp.at(i) == (char)0x7E ||
                temp.at(i) == (char)0x7F) {
            char mem = temp.at(i);
            mem ^= (char)0x02;
            temp.remove(i, 1);
            temp.insert(i, mem);
            temp.insert(i, 0x7D);
        }
        len--;
    }
    temp.push_back((char)0x7F);
    data.clear();
    data.append(temp);
    temp.clear();
    return true;
}

bool RobotControll::unPackData(QByteArray &data)
{
    if(data.isNull() || data.isEmpty()) {
        NDEBUG("data input is null");
        return false;
    }
    if(data.at(0) != 0x7E || data.at(data.length()-1) != 0x7F) {
        NDEBUG("begin and end charater isn't 0x7E and 0x7F");
        return false;
    }
    QByteArray temp;

    temp.append(data);
    temp.remove(0, 1);
    temp.remove(temp.length()-1, 1);
    int len = temp.length();
    while(len) {
        int i = temp.length()-len;
        if (temp.at(i) == (char)0x7D || temp.at(i) == (char)0x7E ||
                temp.at(i) == (char)0x7F) {
            char mem = temp.at(i+1);
            mem ^= (char)0x02;
            temp.remove(i, 2);
            temp.insert(i, mem);
            len--;
        }
        len--;
    }
    data.clear();
    data.append(temp);
    temp.clear();
    return true;
}

void RobotControll::deBug(string file, int line, string function, string message)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":" << message << endl;
    qDebug().noquote() << oss.str().c_str();
}

string RobotControll::qbyteArray2string(QByteArray &data)
{
    return QString::fromLocal8Bit(data).toStdString().c_str();
}

bool RobotControll::writeData(QByteArray &data) {
    if(this->isOpen()) {
        if( this->packData(data) == false ) {
            NDEBUG("error pack");
            return false;
        }
        this->write(data);
        NDEBUG(qbyteArray2string(data));
    } else {
        NDEBUG("no device");
        return false;
    }
    return true;
}

void RobotControll::readData() {
    if(this->isOpen()) {
        QByteArray data = this->readAll();
        while(data.lastIndexOf(0x7E) != -1) {
            QByteArray temp = data.mid(data.lastIndexOf(0x7E));
            data.remove(data.lastIndexOf(0x7E), data.length());
            if( this->unPackData(temp) == false ) {
                NDEBUG("error unpack");
                return;
            }
            setStatus(temp);
            if(robot_stt == RobotDone) {
                emit commandWorkDone();
            }
            //  NDEBUG(qbyteArray2string(temp));
        }
    } else {
        NDEBUG("no device");
        return;
    }
}

void RobotControll::timeOut()
{
    emit commandTimeOut();
    timeout->stop();
    istimeout = true;
}

bool RobotControll::setCommand(robotCommand_t cmd, int time, const QString para) {
    QByteArray command;
    command.clear();
    if(para == "") {
        command.append(tr("%1 %2").arg(QString::number(id_command))
                       .arg(ROBOTCOMMAND[cmd]));
    } else {
        command.append(tr("%1 %2 %3").arg(QString::number(id_command))
                       .arg(ROBOTCOMMAND[cmd]).arg(para));
    }
    if(this->writeData(command) == false) {
        NDEBUG("write data fail");
        return false;
    }
    id_command++;
    timeout->start(time);
    istimeout = false;
    return true;
}

void RobotControll::setStatus(QString response) {
    QStringList listRes = response.split(QRegExp("[:]"),
                                         QString::SplitBehavior::SkipEmptyParts );
    QString stt = listRes.at(1);
    for(int i = 0; i < NumOfStt - 1; i++) {
        if( stt == ROBOTSTATUS[i]) {
            robot_stt = static_cast<robotStatus_t>(i);
            return;
        }
    }
}

RobotControll::robotStatus_t RobotControll::getStatus()
{
    return robot_stt;
}

void RobotControll::resetId()
{
    id_command = 1;
}

bool RobotControll::isTimeOut()
{
    return istimeout;
}

