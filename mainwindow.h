#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QMessageBox>

#include <QSerialPortInfo>
#include <QCameraInfo>

#include <QTimer>

#include <qlabel_custom.h>
#include <robotcontroll.h>
#include <imageprocess.h>

#define PI 3.14159265

Q_DECLARE_METATYPE(QCameraInfo)

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

    // pushButton slot
private slots:

    void on_pushButton_Serial_Default_clicked();

    void on_pushButton_Serial_Connect_clicked();

    void on_pushButton_Camera_Connect_clicked();

    void on_pushButton_Calib_clicked();

    void on_pushButton_Run_clicked();

    void on_pushButton_ShowCamera_clicked();

private slots:
    void serial_updatePortName();                                   // connect timeout timer
    void serial_updateSetting();                                    // connect current text change
    void serial_handleError(QSerialPort::SerialPortError error);    // connect error handle
    void manual_checkBox_event(bool checked);                       // connect checkbox clicked
    void manual_checkPara_setCommand(void);                         // connect request button

    void logs_clear();                                              // connect clear log button
    void camera_updateDevice();                                     // connect timeout timer

private:
    void serial_init();
    void serial_setDefault();
    void serial_openPort();
    void serial_closePort();

    void camera_init();
    void camera_openCamera();
    void camera_closeCamera();

//support
    void statusBar_Message(const QString &message);
    void logs_write(QString message, QColor c);
    void timer_init();

//opencv processing
    void cv_qtshow(Mat img, QImage::Format format);
    void cv_debugImage( Mat image = Mat(), bool dynamic = false);
    void cv_debug(auto mat, const string tag = "");
    void cv_saveImageFromROI();
    Mat cv_getImageFromCamera(ColorConversionCodes flag = COLOR_BGR2BGRA);
    bool cv_sendRequest(RobotControll::robotCommand_t cmd, const QString para);
signals:
    void cv_signalCalib();
    void cv_signalShow(bool dynamic = false);
    void cv_signalAutoRun();

private slots:
    void cv_calib();
    void cv_show(bool dynamic = false);
    void cv_autoRun();
    void cv_timeout();
    void cv_getROI();
    Mat  cv_getMatFromQPixmap(QPixmap pixmap);

private:
    Ui::MainWindow *m_ui = nullptr;
    RobotControll *m_serial = nullptr;
    QLabel *m_status = nullptr;

    QTimer *timer_camera_comboBox = nullptr;
    QTimer *timer_serial_comboBox = nullptr;

    ImageProcess m_camera;
    QTimer *timer_imgproc = nullptr;
    Mat cv_image;

};

#endif // MAINWINDOW_H
