#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/calib3d.hpp"
#include "opencv4/opencv2/core/types.hpp"

#include <QMainWindow>
#include <QDebug>
#include <QMessageBox>

#include <QSerialPortInfo>
#include <QCameraInfo>
#include <QSerialPort>

#include <QTimer>
#include <QImage>
#include <QMutex>
#include <QThread>

Q_DECLARE_METATYPE(QCameraInfo)

using namespace cv;
using namespace std;

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

    typedef enum {
        IMGPROC_SHOW = 27,
        IMGPROC_CALIB,
        IMGPROC_AUTO,
    } ImgProc_Ctrl_Flag_t;
// pushButton slot
private slots:

    void on_pushButton_Serial_Default_clicked();

    void on_pushButton_Serial_Connect_clicked();

    void on_pushButton_Camera_Connect_clicked();

    void on_pushButton_Request_clicked();

    void on_pushButton_Calib_clicked();

private slots:
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
    void serial_read();
    void serial_write(const QByteArray &data);
    void serial_pack(const QByteArray &data);
    void serial_unpack(const QByteArray &data);

private:
    void camera_init();
    void camera_updateDevice();
    void camera_openCamera();
    void camera_closeCamera();

private slots:
    void logs_clear();

private: //support
    void statusBar_Message(const QString &message);
    void logs_write(const QString &message, const QColor &c);
    void timer_init();

private: //opencv processing
    void cv_process_image();
    void cv_qtshow(Mat img, QImage::Format format);
    void cv_calib();
    vector<double> cv_ideal2Real_Cordinate(vector<Point2f> &idealPoints, vector<Point2f> &realPoints);
    void cv_debug(auto mat);


private: //request for controll robot
    void send_request(int &idcommand, const QString command, const QString para);
    int rec_respose(QByteArray mes);

private:
    Ui::MainWindow *m_ui = nullptr;
    QSerialPort *m_serial = nullptr;
    QLabel *m_status = nullptr;

    QByteArray m_dataserial;
    QMutex m_mutex;
    int id_command = 0;

    QTimer *timer_camera_comboBox = nullptr;
    QTimer *timer_serial_comboBox = nullptr;

    VideoCapture m_camera;
    QTimer *timer_imgproc = nullptr;
    ImgProc_Ctrl_Flag_t imgproc_ctrl_flag;




private:
    // function and varaiable of opencv
};

#endif // MAINWINDOW_H
