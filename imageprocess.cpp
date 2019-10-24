#include "mainwindow.h"
#include "ui_mainwindow.h"

// camera function
void MainWindow::camera_init() {
    camera_updateDevice();
    timer_imgproc = new QTimer(this);
    connect(timer_imgproc, &QTimer::timeout, this, &MainWindow::cv_show);
    connect(this, &MainWindow::cv_signalShow, this, &MainWindow::cv_show);
    connect(this, &MainWindow::cv_signalCalib, this, &MainWindow::cv_calib);
    connect(this, &MainWindow::cv_signalAutoRun, this, &MainWindow::cv_autoRun);
}

void MainWindow::cv_autoRun() {

}

void MainWindow::cv_show() {
    const string tag = __FUNCTION__;
    // stop timer timeout
    timer_imgproc->stop();
    if(QObject::sender() == timer_imgproc) {
        Mat frame;
        if(!m_camera.isOpened()) {
            cv_debug("camera is close", tag);
            return;
        }
        m_camera >> frame;
        cv_qtshow(frame, QImage::Format_RGB888);

    } else {
        if(cv_image.empty()) {
            cv_debug("cv_image is empty", tag);
            // stop timer
            return;
        }
        cv_qtshow(cv_image, QImage::Format_RGB888);
        return;
    }
    //tam thoi se khong hien thi duoc anh tu cv_image do timer chay lien tuc.
    timer_imgproc->start(DELAY_CAPTURE_20MS);
}

void MainWindow::cv_calib() {
    const string tag = __FUNCTION__;
    Size patternSize(8,6); //interior number of corners
    vector<vector<Point3f>> listRealPoints;
    vector<vector<Point2f>> listImagePoints;
    // for loop and check to rotate image
    Mat grayImage;
    for(size_t i = 0; i < 10 ; i++ ) {
        QMessageBox::information(this, tr("Calib"), tr("move pattern (%1/10 images)").arg(i+1));
        grayImage = cv_getImageFromCamera(COLOR_BGR2GRAY);
        cv_getPattern2CalibCamera(grayImage, (float)0.025, patternSize,
                                  listImagePoints, listRealPoints);
    }

    cv_calibrateCamera(listImagePoints, listRealPoints, grayImage.size(),
                       getNote[IntrFilePath]);

    vector<Point2f> imagePoints, realPoints;
    cv_getPattern2CalibRobot(grayImage, (float)25.0, patternSize, imagePoints, realPoints);
    cv_calibrateRobot(imagePoints, realPoints, getNote[CordiConvertFilePath]);

    FileStorage readCamMat(getNote[IntrFilePath], FileStorage::READ);
    Mat intrinMat, distortMat;
    readCamMat[getNote[CamMat]] >> intrinMat;
    readCamMat[getNote[DistMat]] >> distortMat;
    cv_debug(intrinMat, tag);
    cv_debug(distortMat, tag);
    readCamMat.release();

    FileStorage readRobotMat(getNote[CordiConvertFilePath], FileStorage::READ);
    Mat rMat, tMat, sMat;
    readRobotMat[getNote[R1to0Mat]] >> rMat;
    readRobotMat[getNote[T1to0Mat]] >> tMat;
    readRobotMat[getNote[S1toMat]] >> sMat;
    cv_debug(rMat, tag);
    cv_debug(tMat, tag);
    cv_debug(sMat, tag);
    readRobotMat.release();

    cv_debug("testting", tag);
    int idx = 30;
    Mat real = (Mat_<double>(2, 1) << realPoints.at(idx).x, realPoints.at(idx).y);
    Mat img = (Mat_<double>(2, 1) << imagePoints.at(idx).x, imagePoints.at(idx).y);
    Mat res = sMat * rMat * img + tMat;
    cv_debug(img, tag);
    cv_debug(real, tag);
    cv_debug(res, tag);

    m_ui->pushButton_Calib->setEnabled(true);
}

Mat MainWindow::cv_getImageFromCamera( ColorConversionCodes flag) {
    const string tag = __FUNCTION__;
    if(!m_camera.isOpened()) {
        cv_debug("camera is close", tag);
        return Mat();
    }
    Mat imageColor, imageConvert;
    m_camera >> imageColor;
    if(flag == COLOR_BGR2BGRA) {
        imageConvert.release();
        return imageColor;
    } else {
        cvtColor(imageColor, imageConvert, flag);
        imageColor.release();
        return imageConvert;
    }
}

void MainWindow::cv_debug(auto mat, const string tag) {
    ostringstream oss;
    oss << tag << endl << mat << endl;
    qDebug() << oss.str().c_str();
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

void MainWindow::cv_getPattern2CalibRobot(Mat grayImage, float realPointDistance,
                                          Size patternSize, vector<Point2f> &imagePoints,
                                          vector<Point2f> &realPoints) {
    const string tag = __FUNCTION__;
    if(grayImage.empty()) {
        cv_debug("grayImage input is empty", tag);
        return;
    }

    Mat camMat, distortMat;
    FileStorage file(getNote[IntrFilePath], FileStorage::READ );
    file[getNote[CamMat]] >> camMat;
    file[getNote[DistMat]] >> distortMat;
    if(camMat.empty() || distortMat.empty()) {
        cv_debug("camMat or distortMat input is empty", tag);
    }

    Mat undistortImage;
    undistort( grayImage, undistortImage, camMat, distortMat);
    Point2f Obset(- 4.0*realPointDistance, 1*realPointDistance + 130); // mm

    if( findChessboardCorners(undistortImage, patternSize, imagePoints,
                              CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
                              CALIB_CB_FAST_CHECK) ) {
        for(int y = 0 ; y < patternSize.height ; y++) {
            for(int x = 0 ; x < patternSize.width ; x++) {
                realPoints.push_back(Point2f( (float)x * realPointDistance,
                                              (float)y * realPointDistance ) + Obset);
            }
        }
    } else {
        cv_debug("can't find corner", tag);
        return;
    }
    cv_image = undistortImage.clone();
    drawChessboardCorners(cv_image, patternSize, imagePoints, true);
    emit cv_show();
}

void MainWindow::cv_calibrateRobot( vector<Point2f> vecImagePoints,
                                    vector<Point2f> vecRobotPoints,
                                    const string &fileName) {
    const string tag = __FUNCTION__;
    if( (vecImagePoints.size() != vecRobotPoints.size()) || vecImagePoints.empty() ) {
        cv_debug("2 source.size is different or source is empty", tag);
        return;
    }
    Point2f point0_0 = vecRobotPoints.front();
    Point2f point0_1 = vecRobotPoints.at(7);
    Point2f point1_0 = vecImagePoints.front();
    Point2f point1_1 = vecImagePoints.at(7);
    // angle between robot point to image point
    Point2f vector0 = point0_1 - point0_0;
    Point2f vector1 = point1_1 - point1_0;
    double alpha = atan2(vector0.y, vector0.x);
    double beta = atan2(vector1.y, vector1.x);
    // angle is is must reflection.
    double theta = - (alpha - beta);
    //    cv_debug( tr("theta %1").arg(theta * 180.0 / 3.1415), tag );
    // R matrix and scale efficient from 1 to 0 cordinate
    Mat R1to0 = (Mat_<double>(2,2) << cos(theta), sin(theta),
                                     -sin(theta), cos(theta));
    double scale1to0 = norm(vector0) / norm(vector1);

    // T matrix
    Mat point0 = (Mat_<double>(2, 1) << point0_0.x, point0_0.y);
    Mat point1 = (Mat_<double>(2, 1) << point1_0.x, point1_0.y);
    Mat S1to0 = (Mat_<double>(2, 2) << scale1to0, 0, 0, scale1to0);
    Mat T1to0 = point0 - S1to0 * R1to0 * point1;

    FileStorage file(fileName, FileStorage::WRITE );
    file << getNote[R1to0Mat] << R1to0;
    file << getNote[T1to0Mat] << T1to0;
    file << getNote[S1toMat] << S1to0;
    file.release();
}

void MainWindow::cv_calibrateCamera( vector<vector<Point2f>> listImagePoints,
                                     vector<vector<Point3f>> listRealPoints,
                                     Size imageSize, const string &fileName) {
    const string tag = __FUNCTION__;
    // load camera Matrix
    Mat camMat, distortMat, rMat, tMat;
    FileStorage file(fileName, FileStorage::READ );
    file[getNote[CamMat]] >> camMat;
    file.release();
    if(camMat.empty()) {
        calibrateCamera(listRealPoints, listImagePoints, imageSize, camMat,
                        distortMat, rMat, tMat, CALIB_FIX_PRINCIPAL_POINT);
        cv_debug("camMat empty", tag);
    } else {
        calibrateCamera(listRealPoints, listImagePoints, imageSize, camMat,
                        distortMat, rMat, tMat, CALIB_FIX_PRINCIPAL_POINT +
                        CALIB_USE_INTRINSIC_GUESS);
        cv_debug("camMat is available", tag);
    }
    file.open(fileName, FileStorage::WRITE );
    file << getNote[CamMat] << camMat;
    file << getNote[DistMat] << distortMat;
    file.release();
}

void MainWindow::cv_getPattern2CalibCamera(Mat grayImage, float realPointDistance,
                                           Size patternSize, vector<vector<Point2f>> &listImagePoints,
                                           vector<vector<Point3f>> &listRealPoints) {
    const string tag = __FUNCTION__;
    //  this will be filled by the detected corners
    vector<Point2f> imagePoints;
    vector<Point3f> realPoints;
    for(int y = 0 ; y < patternSize.height ; y++) {
        for(int x = 0 ; x < patternSize.width ; x++) {
            realPoints.push_back(Point3f((float)x * realPointDistance,
                                         (float)y * realPointDistance, 0));
        }
    }
    if(grayImage.empty()) {
        cv_debug("grayImage input is empty", tag);
        return;
    }
    if( findChessboardCorners(grayImage, patternSize, imagePoints,
                              CALIB_CB_ADAPTIVE_THRESH +
                              CALIB_CB_NORMALIZE_IMAGE +
                              CALIB_CB_FAST_CHECK) ) {
        listImagePoints.push_back(imagePoints);
        listRealPoints.push_back(realPoints);
    } else {
        cv_debug("can't find corner", tag);
        return;
    }

}

void MainWindow::on_pushButton_Calib_clicked()
{
    m_ui->pushButton_Calib->setEnabled(false);
    emit cv_signalCalib();
}



















