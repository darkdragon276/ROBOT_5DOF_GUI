#include "mainwindow.h"
#include "ui_mainwindow.h"

void MainWindow::cv_process_image() {
    Mat frame;
    m_camera >> frame;
    if(frame.empty()) {
        qDebug() << "frame empty";
        camera_closeCamera();
        return;
    }
    // process
    cv_qtshow(frame, QImage::Format_RGB888);
}

void MainWindow::cv_qtshow(Mat img, QImage::Format format) {
    Mat temp;
    cvtColor(img, temp, COLOR_BGR2RGB);
    QImage* qimage = new QImage(temp.data, temp.cols, temp.rows, temp.step, format);
    m_ui->label_Camera_show->setPixmap(QPixmap::fromImage(*qimage));
    temp.release();
    img.release();
    delete qimage;
}
