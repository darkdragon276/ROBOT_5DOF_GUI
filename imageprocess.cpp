#include "mainwindow.h"
#include "ui_mainwindow.h"

// camera function
void MainWindow::camera_init() {
    camera_updateDevice();
    timer_imgproc = new QTimer(this);
    connect(timer_imgproc, &QTimer::timeout, this, &MainWindow::cv_process_image);
    imgproc_ctrl_flag = IMGPROC_SHOW;
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

void MainWindow::cv_process_image() {
    Mat frame;
    m_camera >> frame;
    if(frame.empty()) {
        qDebug() << "frame empty";
        camera_closeCamera();
        return;
    }
    timer_imgproc->stop();
    switch(imgproc_ctrl_flag) {
    case IMGPROC_SHOW:
        cv_qtshow(frame, QImage::Format_RGB888);
        //        imgproc_ctrl_flag = IMGPROC_CALIB;
        break;
    case IMGPROC_CALIB:
        cv_calib();
        break;
    case IMGPROC_AUTO:
        break;
    default :
        break;
    }
    timer_imgproc->start(DELAY_CAPTURE_20MS);
}

void MainWindow::cv_calib() {
    qDebug() << "in calib function";
    if(!m_camera.isOpened()) {
        qDebug() << "Camera is close";
        return;
    }
    Mat img;
    m_camera >> img;
    Mat img_corner, img_pose = img.clone();
    Size img_size = img.size();

    Size patternsize(8,6); //interior number of corners
    vector<vector<Point2f>> list_corners;
    vector<vector<Point3f>> list_objects;
    vector<Point2f> corners; //this will be filled by the detected corners
    vector<Point3f> objects;
    for(int y = 0 ; y < 6; y++) {
        for(int x = 0 ; x < 8; x++) {
            objects.push_back(Point3f((float)x * 25.0, (float)y * 25.0, 0));
        }
    }

    cvtColor(img, img_corner, COLOR_BGR2GRAY);
    bool patternfound = findChessboardCorners(img_corner, patternsize, corners,
                                              CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if(patternfound) {
        cornerSubPix(img_corner, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.1));
        list_corners.push_back(corners);
        list_objects.push_back(objects);
        drawChessboardCorners(img, patternsize, Mat(corners), patternfound);
        // calib camera
        Mat CamMat, distCoeffs, rvecs, tvecs, stdDeviationsIntrin, stdDeviationsExtrin, perViewerror;
        double acuracy = calibrateCamera(list_objects, list_corners, img_size, CamMat,
                                         distCoeffs, rvecs, tvecs, stdDeviationsIntrin, stdDeviationsExtrin,
                                         perViewerror, CALIB_FIX_PRINCIPAL_POINT);

        Q_UNUSED(acuracy);
        Mat optimal_camMat = getOptimalNewCameraMatrix(CamMat, distCoeffs, img_size, acuracy);
        Mat Tmat, Rmat, Pmat;   // Pmat = {CaMat|Tmat}
        Tmat.push_back(tvecs.at<double>(0,0));
        Tmat.push_back(tvecs.at<double>(0,1));
        Tmat.push_back(tvecs.at<double>(0,2));
        Rodrigues(rvecs, Rmat);
        hconcat(CamMat, Tmat, Pmat);

//        cv_debug(Tmat);
//        cv_debug(Rmat);
//        cv_debug(Pmat);
        vector<Point2f> cam_cordinates = {corners.at(0), corners.at(1), corners.at(2)};
        cv_debug(cam_cordinates);

        Mat undist_img;
        vector<Point2f> undist_corners;
        undistort(img_corner, undist_img, CamMat, distCoeffs);
        bool patternfound = findChessboardCorners(undist_img, patternsize, undist_corners,
                                                  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        cv_debug(undist_corners);


    }
    cv_qtshow(img_pose, QImage::Format_RGB888);
    imgproc_ctrl_flag = IMGPROC_SHOW;
}

vector<double> MainWindow::cv_ideal2Real_Cordinate(vector<Point2f> &idealPoints, vector<Point2f> &realPoints) {
//    Point2f central_ideal_point = idealPoints.at(0);
//    Point2f support_ideal_point = idealPoints.at(1);
//    Point2f central_real_point = realPoints.at(0);
//    Point2f support_real_point = realPoints.at(1);
    vector<double> Kt_matrix;
//    double fx =

    return Kt_matrix;
}

void MainWindow::cv_debug(auto mat) {
    ostringstream oss;
    oss << mat << endl;
    qDebug() << oss.str().c_str();
}
























