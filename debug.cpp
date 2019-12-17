#include "debug.h"

Debug::Debug()
{

}

Debug::~Debug()
{

}

void Debug::deBug(string file, int line, string function, string message)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":" << message << endl;
    qDebug().noquote() << oss.str().c_str();
}

void Debug::deBug(string file, int line, string function, const char *message)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":" << message << endl;
    qDebug().noquote() << oss.str().c_str();
}

void Debug::deBug(string file, int line, string function, Mat _mat)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":" << _mat << endl;
    qDebug().noquote() << oss.str().c_str();
}

void Debug::deBug(string file, int line, string function, Point2f _point)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":(" <<
           _point.x << "," << _point.y << ")" << endl;
    qDebug().noquote() << oss.str().c_str();
}

void Debug::deBug(string file, int line, string function, Scalar _scalar)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":" << _scalar << endl;
    qDebug().noquote() << oss.str().c_str();
}

void Debug::deBug(string file, int line, string function, QString message)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":" << message.toStdString() << endl;
    qDebug().noquote() << oss.str().c_str();
}

