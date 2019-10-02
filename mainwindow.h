#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "opencv4/opencv2/opencv.hpp"

#include <QMainWindow>
#include <QSerialPort>
#include <QDebug>
#include <QMessageBox>
#include <QSerialPortInfo>

#include <QCameraInfo>
#include <QCamera>
#include <QAbstractVideoSurface>

#include <QTimer>

Q_DECLARE_METATYPE(QCameraInfo)

using namespace cv;
using namespace std;

namespace Ui {
class MainWindow;
class ImageProcess;
}

class ImageProcess: public QAbstractVideoSurface
{
    Q_OBJECT
public:
    ImageProcess(QObject * parent=NULL) : QAbstractVideoSurface(parent) {}
    QList<QVideoFrame::PixelFormat> supportedPixelFormats(QAbstractVideoBuffer::HandleType type) const{
        return QList<QVideoFrame::PixelFormat>() << QVideoFrame::Format_RGB24 ; // here return whatever formats you will handle
    }

    bool present(const QVideoFrame& frame){
        if (frame.isValid()) {
            QVideoFrame cloneFrame(frame);
            cloneFrame.map(QAbstractVideoBuffer::ReadOnly);
            const QImage img(cloneFrame.bits(),
                         cloneFrame.width(),
                         cloneFrame.height(),
                         QVideoFrame::imageFormatFromPixelFormat(cloneFrame.pixelFormat()));

           // do something with the image ...

            cloneFrame.unmap();
            return true;
        }
        return false;
    }

};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pushButton_Serial_Default_clicked();

    void on_pushButton_Serial_Connect_clicked();

    void on_pushButton_Test_clicked();


private slots:
    void serial_updatePortName();
    void serial_updateSetting();
    void serial_handleError(QSerialPort::SerialPortError error);

private:
    void serial_init();
    void serial_setDefault();
    void serial_openPort();
    void serial_closePort();
    void serial_read();
    void serial_write(const QByteArray &data);

private:
    void camera_init();
    void camera_updateDevice();
    void camera_setDevice();
    void camera_openCamera();
    void camera_closeCamera();

private slots:
    void camera_handleError();
    void camera_stateControll(QCamera::State state);

private slots:
    void logs_clear();
    void on_pushButton_Camera_Connect_clicked();

private:
    void statusBar_Message(const QString &message);
    void logs_write(const QString &message, const QColor &c);
    void timer_init();
private:
    Ui::MainWindow *m_ui = nullptr;
    QSerialPort *m_serial = nullptr;
    QLabel *m_status = nullptr;
    QScopedPointer<QCamera> m_camera;
    QScopedPointer<ImageProcess> m_img;
    QTimer *timer_camera = nullptr;
    QTimer *timer_serial = nullptr;
    QTimer *timer_frame = nullptr;

private:
    // function and varaiable of opencv
};

#endif // MAINWINDOW_H
