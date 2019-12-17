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
        M_DEBUG("data input is empty");
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

    Debug::_delete(temp);
    return true;
}

bool RobotControll::unPackData(QByteArray &data)
{
    if(data.isNull() || data.isEmpty()) {
        M_DEBUG("data input is null");
        return false;
    }
    if(data.at(0) != 0x7E || data.at(data.length()-1) != 0x7F) {
        M_DEBUG("begin and end charater isn't 0x7E and 0x7F");
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

    Debug::_delete(temp);
    return true;
}

string RobotControll::qbyteArray2string(QByteArray &data)
{
    return QString::fromLocal8Bit(data).toStdString().c_str();
}

bool RobotControll::writeData(QByteArray &data) {
    if(this->isOpen()) {
        if( this->packData(data) == false ) {
            M_DEBUG("error pack");
            return false;
        }
        this->write(data);
        M_DEBUG(qbyteArray2string(data));
    } else {
        M_DEBUG("no device");
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
                M_DEBUG("error unpack");
                Debug::_delete(temp, data);
                return;
            }
            setStatus(temp);
            if(robot_stt == RobotDone) {
                emit commandWorkDone();
            }
            //  M_DEBUG(qbyteArray2string(temp));
        }
    } else {
        M_DEBUG("no device");
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
        M_DEBUG("write data fail");
        Debug::_delete(command);
        return false;
    }
    id_command++;
    timeout->start(time);
    istimeout = false;

    Debug::_delete(command);
    return true;
}

void RobotControll::setStatus(QString response) {
    QStringList listRes = response.split(QRegExp("[:]"),
                                         QString::SplitBehavior::SkipEmptyParts );
    QString stt = listRes.at(1);
    for(int i = 0; i < NumOfStt - 1; i++) {
        if( stt == ROBOTSTATUS[i]) {
            robot_stt = static_cast<robotStatus_t>(i);
            M_DEBUG(ROBOTSTATUS[robot_stt]);
            return;
        }
    }
    stt.clear();
    stt.resize(0);
    listRes.clear();
}

bool RobotControll::setCommandNWait(robotCommand_t cmd, const QString para)
{
    if( this->setCommand(cmd, 3000, para) == false) {
        M_DEBUG("can't set command");
        return false;
    }
    QEventLoop loop;
    connect( this, &RobotControll::commandWorkDone, &loop, &QEventLoop::quit );
    connect( this, &RobotControll::commandTimeOut, &loop, &QEventLoop::quit );
    loop.exec();
    if(this->isTimeOut()) {
        M_DEBUG("work haven't done yet");
        return false;
    }
    M_DEBUG("command done");
    return true;
}

void RobotControll::getObsetPosition(Point2f pos, Point3f &obset)
{
    double x_raw = -pos.x/10;
    double y_raw = pos.y/10;

    int num_cols = (int)(-x_raw/2.5 + 4.5);
    if(num_cols < 0) {
        num_cols = 0;
    } else if(num_cols > 8) {
        num_cols = 8;
    }

    int num_rows = (int)(y_raw/2.5  - 1.5);
    if(num_rows < 0) {
        num_rows = 0;
    } else if(num_cols > 6) {
        num_rows = 6;
    }
    obset = Point3f(x_raw + robot_obset.at(num_rows).at(num_cols).x,
                    y_raw + robot_obset.at(num_rows).at(num_cols).y,
                    robot_obset.at(num_rows).at(num_cols).z);
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

void RobotControll::setWidthNPosition(Point2f pos, int time, double width, Point2f base_center)
{
    Point3f obset_pos, obset_base;
    getObsetPosition(pos, obset_pos);
    getObsetPosition(base_center, obset_base);
    const QString para_poswidmax = tr("%1 %2 %3 %4").arg(width + 1)
                                                    .arg(obset_pos.x)
                                                    .arg(obset_pos.y)
                                                    .arg(obset_pos.z - 0.5);
    const QString para_poswidmin = tr("%1 %2 %3 %4").arg(width - 1)
                                                    .arg(obset_pos.x)
                                                    .arg(obset_pos.y)
                                                    .arg(obset_pos.z - 0.5);
    const QString para_time = tr("%1").arg(time);
    const QString para_posbase = tr("%1 %2 %3").arg(obset_base.x)
                                               .arg(obset_base.y)
                                               .arg(obset_base.z);
    const QString para_poswidbase = tr("%1 %2 %3 %4").arg(width + 0.5)
                                                     .arg(obset_base.x)
                                                     .arg(obset_base.y)
                                                     .arg(obset_base.z);
    M_DEBUG(para_poswidmax);
    M_DEBUG(para_posbase);
    setCommandNWait(SetTime, para_time);
    setCommandNWait(SetWidPos, para_poswidmax);
    setCommandNWait(SetWidPos, para_poswidmin);
    setCommandNWait(SetHome);
    setCommandNWait(SetPosition, para_posbase);
    setCommandNWait(SetWidPos, para_poswidbase);
    setCommandNWait(SetHome);
}
