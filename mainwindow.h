#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QMessageBox>
#include <QSerialPortInfo>
#include <QCameraInfo>
#include <QTimer>
#include <QSlider>
#include <QSpinBox>

#include "qlabel_custom.h"
#include "robotcontroll.h"
#include "imageprocess.h"
#include "debug.h"

#define PI 3.14159265
#define MAIN_TIMER_REFRESH  (1000/1)

Q_DECLARE_METATYPE(QCameraInfo)

using namespace std;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private:
    void serial_init();
    void serial_setDefault();
    void serial_openPort();
    void serial_closePort();
    void logs_write(QString message, QColor c);

private slots:
    // slot in serial tag
    void manual_checkBox_event(bool checked);                       // connect checkbox clicked
    void manual_checkPara_setCommand(void);                         // connect request button
    void serial_updatePortName();                                   // connect timeout timer
    void serial_updateSetting();                                    // connect current text change
    void serial_handleError(QSerialPort::SerialPortError error);    // connect error handle
    void logs_clear();                                              // connect clear log button

    // pushbutton slot
    void on_pushButton_Serial_Default_clicked();
    void on_pushButton_Serial_Connect_clicked();

private:
    void camera_init();
    void camera_openCamera();
    void camera_closeCamera();

private slots:
    void camera_updateDevice();             // connect timeout timer

private:
    //crossover other class function
    void cv_qtshow(Mat img, QImage::Format format);
    void cv_debugImage(Mat image);

signals:
    void cv_signalShow(bool dynamic = true);

private slots:
    // mouse select roi
    void cv_getROI();                       // grab roi mouse released
    void cv_saveObjectFromROI();            // Button Save Image
    void cv_saveHSVBaseFromROI();           // Button Save HSV Range
    // timer processing image
    void cv_timeout();                      // Timer 20ms get frame
    // slot with scrollbar
    void pre_setSlider(QDoubleSpinBox *spinbox, QSlider *slider);
    void pre_setSpinBox(QSlider *slider, QDoubleSpinBox *spinbox);

    // push button
    void cv_show(bool dynamic = false);
    void cv_calib();                        // Button calib
    void cv_autoRun();                      // Button AutoRun

    // pushbutton slot
    void on_pushButton_Camera_Connect_clicked();
    void on_pushButton_ShowCamera_clicked();

    // slider and spinbox and checkbox slot
    void dip_sliderChanged(int value);
    void dip_spinBoxEditingFinished();
    void dip_checkBoxEnableClicked(bool checked);

private:
    Ui::MainWindow *m_ui = nullptr;
    RobotControll *m_serial = nullptr;
    //    QThread *thread_serial;
    QLabel *m_status = nullptr;

    QTimer *timer_camera_comboBox = nullptr;
    QTimer *timer_serial_comboBox = nullptr;

    ImageProcess m_camera;
    QTimer *timer_imgproc = nullptr;
    Mat cv_image;


};

#endif // MAINWINDOW_H
