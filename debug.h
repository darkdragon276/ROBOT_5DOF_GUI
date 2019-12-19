#ifndef DEBUG_H
#define DEBUG_H

#include <QtCore>
#include <QDebug>

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdarg>

#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/types.hpp"

#define M_DEBUG(message)  Debug::deBug(__FILE__, __LINE__, __FUNCTION__, message)

using namespace std;
using namespace cv;

class Debug
{
public:
    explicit Debug();
    virtual ~Debug();
    static void deBug(string file, int line, string function, string message);
    static void deBug(string file, int line, string function, const char* message);
    static void deBug(string file, int line, string function, Mat _mat);
    static void deBug(string file, int line, string function, Point2f _point);
    static void deBug(string file, int line, string function, Scalar _scalar);
    static void deBug(string file, int line, string function, QString message);
    //  QElapsedTimer _timer;
    //  _timer.start();
    //  qDebug() << "Time took" << _timer.elapsed() << "milliseconds";
    static void _delete(QByteArray& arg) {
        arg.clear();
        arg.resize(0);
    }

    static void _delete(Mat& arg)
    {
        arg.release();
    }

    template<typename type>
    static void _delete(vector<type>& arg)
    {
        arg.clear();
        arg.shrink_to_fit();
    }

    template<typename type>
    static void _delete(Ptr<type>& arg)
    {
        arg.release();
    }

    template<typename type, typename ...T>
    static void _delete(vector<type>& arg, T&... args)
    {
        Debug::_delete(arg);
        Debug::_delete(args...);
    }

    template<typename type, typename ...T>
    static void _delete(Ptr<type>& arg, T&... args)
    {
        Debug::_delete(arg);
        Debug::_delete(args...);
    }

    template<typename type, typename ...T>
    static void _delete(type& arg, T&... args)
    {
        Debug::_delete(arg);
        Debug::_delete(args...);
    }
};

#endif // DEBUG_H
