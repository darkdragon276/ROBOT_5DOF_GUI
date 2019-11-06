#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/calib3d.hpp"
#include "opencv4/opencv2/core/types.hpp"
#include "opencv4/opencv2/xfeatures2d.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

#include <QMainWindow>
#include <QDebug>
#include <QMessageBox>

#include <QSerialPortInfo>
#include <QCameraInfo>
#include <QSerialPort>

#include <QTimer>
#include <QImage>

#include <math.h>
#include <stdio.h>

#include <qlabel_custom.h>

#define PI 3.14159265

Q_DECLARE_METATYPE(QCameraInfo)

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

#define DELAY_CAPTURE_20MS  20

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    enum cv_fileString{
        CamMat = 0,
        DistMat,
        R1to0Mat,
        T1to0Mat,
        S1to0Mat,
        PathIntrincyPara,
        PathCordinateConvertPara,
        PathObjectImageSave,
        NumOfMat,
    };
    const char* getNode[NumOfMat] = {
        "CameraIntrincyMatrix",
        "DistortExtrincyMatrix",
        "CordinatesRotateMatrix",
        "CordinatesTranferMatrix",
        "CordinatesScaleMatrix",
        "cameraMatrix.yml",
        "robotMatrix.yml",
        "./images/object.jpg"
    };
    enum cv_commandString {
        SetHome = 0,
        SetPosition,
        SetWidth,
        SetDuty,
        Save,
        NumOfCmd,
    };
    const char* getCommand[NumOfCmd] = {
        "SETHOME",
        "SETPOS",
        "SETWID",
        "SETDUTY",
        "SAVE"
    };
    enum robot_status {
        RobotDone = 0,
        RobotProc,
        RobotReq,
        RobotErrArg,
        RobotErr,
        NumOfStt,
    };
    const char* getStatus[NumOfStt] = {
        "DONE",
        "PROCESSING",
        "REQUEST",
        "ERROR ARGUMENT",
        "ERROR"
    };
    // pushButton slot
private slots:

    void on_pushButton_Serial_Default_clicked();

    void on_pushButton_Serial_Connect_clicked();

    void on_pushButton_Camera_Connect_clicked();

    void on_pushButton_Request_clicked();

    void on_pushButton_Calib_clicked();

    void on_pushButton_Run_clicked();

    void on_pushButton_ShowCamera_clicked();

    void on_pushButton_SaveImg_clicked();

public slots:
    void serial_updatePortName();
    void serial_updateSetting();
    void serial_handleError(QSerialPort::SerialPortError error);
    void manual_checkBox_event(bool checked);
    void manual_checkPara_sendRequest(void);

private:
    void serial_init();
    void serial_setDefault();
    void serial_openPort();
    void serial_closePort();
    bool serial_write(QByteArray &data);
    void serial_read();
    bool serial_pack(QByteArray &data);
    bool serial_unpack(QByteArray &data);

private:
    void camera_init();
    void camera_updateDevice();
    void camera_openCamera();
    void camera_closeCamera();

private slots:
    void logs_clear();

private: //support
    void statusBar_Message(const QString &message);
    void logs_write(QString message, QColor c);
    void timer_init();

private: //controll robot
    bool robot_sendResq(cv_commandString cmd, const QString para = "");
    void robot_readResponse(QString res);
    void robot_ctrlResq(int idCmd, robot_status stt);
    void robot_init();
    robot_status robot_getStt(int idCmd);
signals:
    void robot_signalEmit(int idCmd, robot_status stt);


private: //opencv processing
    void cv_qtshow(Mat img, QImage::Format format);
    void cv_debugImage( Mat image = Mat(), bool dynamic = false);
    void cv_debug(auto mat, const string tag = "");
    void cv_readMatFromFile(cv_fileString node, cv_fileString filepath,
                            Mat &mat, bool debug = false, const string tag = "");
    Mat cv_getImageFromCamera(ColorConversionCodes flag = COLOR_BGR2BGRA);
    bool cv_undistortImage(Mat grayImage, Mat &undistortImage);
    bool cv_convertToRealPoint(Point2f imagePoint, Point2f &realPoint);

    bool cv_getPattern2CalibCamera(Mat grayImage, float realPointDistance, Size patternSize,
                                   vector<vector<Point2f>> &listImagePoints,
                                   vector<vector<Point3f>> &listRealPoints );
    bool cv_getPattern2CalibRobot(Mat grayImage, float realPointDistance, Size patternSize,
                                  vector<Point2f> &imagePoints, vector<Point2f> &realPoints);

    void cv_calibrateRobot(vector<Point2f> vecImagePoints,
                           vector<Point2f> vecRobotPoints);
    void cv_calibrateCamera(vector<vector<Point2f>> listImagePoints,
                            vector<vector<Point3f>> listRealPoints,
                            Size imageSize);
    bool cv_sendRequest(cv_commandString cmd, const QString para = "");
    void cv_saveImageFromROI();
    void cv_getMatchesFromObject(Mat object, Mat scene,
                                 vector<Point2f> &keypoint_object,
                                 vector<Point2f> &keypoint_scene);
    void cv_hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                   vector<vector<int>> &pointInGroup_idx);
signals:
    void cv_signalCalib();
    void cv_signalShow(bool dynamic = false);
    void cv_signalAutoRun();

private slots:
    void cv_calib();
    void cv_show(bool dynamic = false);
    void cv_autoRun();
    void cv_timeout();

private slots:
    void cv_getROI();
    Mat cv_getMatFromQPixmap(QPixmap pixmap);

private:
    Ui::MainWindow *m_ui = nullptr;
    QSerialPort *m_serial = nullptr;
    QLabel *m_status = nullptr;
    int id_command = 0;
    QList<robot_status> m_robotStt;

    QTimer *timer_camera_comboBox = nullptr;
    QTimer *timer_serial_comboBox = nullptr;

    VideoCapture m_camera;
    QTimer *timer_imgproc = nullptr;
    Mat cv_image;

};

#endif // MAINWINDOW_H
