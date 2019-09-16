#include "mainwindow.h"
#include <QApplication>
#include <QLabel>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Mat inputImage = imread("/home/chuhainam/Desktop/project/my-project/test.png");
    QImage img = QImage((uchar*) inputImage.data, inputImage.cols, inputImage.rows, inputImage.step, QImage::Format_RGB888);

    QLabel label;
    label.setPixmap(QPixmap::fromImage(img));

    label.show();

    return a.exec();
}
