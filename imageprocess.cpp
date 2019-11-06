#include "mainwindow.h"
#include "ui_mainwindow.h"

// camera function
void MainWindow::camera_init() {
    camera_updateDevice();
    timer_imgproc = new QTimer(this);
    connect(timer_imgproc, &QTimer::timeout, this, &MainWindow::cv_timeout);
    connect(this, QOverload<bool>::of(&MainWindow::cv_signalShow), this, &MainWindow::cv_show);
    connect(this, &MainWindow::cv_signalCalib, this, &MainWindow::cv_calib);
    connect(this, &MainWindow::cv_signalAutoRun, this, &MainWindow::cv_autoRun);
    connect(m_ui->label_Camera_show, SIGNAL(mouseReleased()), this, SLOT(cv_getROI()));
}

void MainWindow::cv_timeout() {
    cv_debugImage(Mat(), true);
}

//-- Function return point is devided into group.
//-- pointInGroup_idx is reference from point_list.
void MainWindow::cv_hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                           vector<vector<int>> &pointInGroup_idx) {
    const string tag = __FUNCTION__;
    if(point_list.empty()) {
        cv_debug("error empty input", tag);
        return;
    }
    if(max_distance <= 0) {
        cv_debug("error distance input", tag);
        return;
    }
    vector<bool> check(point_list.size(), false);
    vector<int> temp_idx;

    for(size_t i = 0; i < point_list.size(); i++) {
        if(check.at(i)) {
            continue;
        } else {
            temp_idx.push_back(i);
            check.at(i) = true;
            for( size_t t = i + 1; t < point_list.size(); t++) {
                if(check.at(t)) {
                    continue;
                } else {
                    if( norm(point_list.at(i) - point_list.at(t)) < max_distance) {
                        temp_idx.push_back(t);
                        check.at(t) = true;
                    }
                }
            }
            pointInGroup_idx.push_back(temp_idx);
            temp_idx.clear();
        }
    }
    cv_debug(pointInGroup_idx.size(), tag);
}

void MainWindow::cv_getMatchesFromObject(Mat object, Mat scene, vector<Point2f> &keypoint_object,
                                         vector<Point2f> &keypoint_scene)
{
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    Mat img_object = object.clone(), img_scene = scene.clone();
    Ptr<SURF> detector = SURF::create( 200.0, 10, 3, false, false );
    vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
    detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    vector<vector<DMatch>> knn_matches;
    vector<DMatch> matches;
    matcher->knnMatch(descriptors_scene, descriptors_object, knn_matches, 2);
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            matches.push_back(knn_matches[i][0]);
        }
    }
    //-- Return position of keypoint
    for( size_t i = 0; i < matches.size(); i++ ) {
        //-- Get the keypoints from the good matches
        keypoint_object.push_back( keypoints_object[ matches[i].trainIdx ].pt );
        keypoint_scene.push_back( keypoints_scene[ matches[i].queryIdx ].pt );
    }
    // Must return vector<vector<keypoint>> tuong ung voi vector<vector<scene>>
    //-- Release memory
    detector.release();
    keypoints_object.clear();
    keypoints_scene.clear();
    descriptors_object.release();
    descriptors_scene.release();
    matcher.release();
    matches.clear();
    knn_matches.clear();
    img_object.release();
    img_scene.release();

    //-- Draw matches
    //    Mat img_matches;
    //    drawMatches( img_scene, keypoints_scene, img_object, keypoints_object, matches, img_matches, Scalar::all(-1),
    //                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //    cv_debugImage(img_matches, false);
    //    img_matches.release();
}

void MainWindow::cv_autoRun() {
    const string tag = __FUNCTION__;

    Mat img_object = imread(getNode[PathObjectImageSave]);
    Mat img_scene = cv_getImageFromCamera();

    vector<Point2f> matchObjectPoint;
    vector<Point2f> matchScenePoint;
    cv_getMatchesFromObject(img_object, img_scene, matchObjectPoint, matchScenePoint);

    vector<vector<int>> groupPointIdx;
    cv_hierarchicalClustering(matchScenePoint, 150.0, groupPointIdx);

    for(size_t i = 0; i < groupPointIdx.size(); i++) {
        for(size_t y = 0; y < groupPointIdx.at(i).size(); y++) {
            circle(img_scene, matchScenePoint.at(groupPointIdx.at(i).at(y)), 1, Scalar(i*10+100, i*20+100, i*70+10), 2);
        }
    }
    cv_debugImage(img_scene);

    //-- Localize the object

//
//    Mat H = findHomography( obj, scene, RANSAC );
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    vector<Point2f> obj_corners(4);
//    obj_corners[0] = Point2f(0, 0);
//    obj_corners[1] = Point2f( (float)img_object.cols, 0 );
//    obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
//    obj_corners[3] = Point2f( 0, (float)img_object.rows );
//    std::vector<Point2f> scene_corners(4);
//    if(!H.empty()) {
//        perspectiveTransform( obj_corners, scene_corners, H);
//        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//        line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
//                scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
//        line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
//                scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
//                scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
//                scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        cv_debugImage(img_matches, false);
//        cv_debug("H is available", tag);
//    }

    /*
    if( cv_sendRequest(SetHome) == false ) {
        return;
    }
    Mat undistImg;
    if(cv_undistortImage(grayImg, undistImg) == false) {
        cv_debug("error undistort image", tag);
        grayImg.release();
        return;
    }

    // change to detect image point of object
    vector<Point2f> imagePoints, realPoints;
    if( cv_getPattern2CalibRobot(grayImg, (float)25.0, Size(8, 6),
                                 imagePoints, realPoints) == false ) {
        return;
    }

    cv_debug("testting", tag);
    for(size_t idx = 0; idx < realPoints.size() ; idx++) {
        Mat real = (Mat_<double>(2, 1) << realPoints.at(idx).x, realPoints.at(idx).y);
        Mat img = (Mat_<double>(2, 1) << imagePoints.at(idx).x, imagePoints.at(idx).y);
        cv_debug(img, tag);
        cv_debug(real, tag);

        // get real point
        Point2f res;
        cv_convertToRealPoint(imagePoints.at(idx), res);
        cv_debug(res, tag);

        if( cv_sendRequest(SetWidth, tr("2.0")) == false) {
            return;
        }
        if( cv_sendRequest(SetPosition, tr("%1 %2 0").arg(-res.x/10).arg(res.y/10)) == false) {
            return;
        }

        if( cv_sendRequest(SetHome) == false) {
            return;
        }
        cv_debug("request done", tag);
        for(int i = 0 ; i < 1000000; i++) {

        }
    }
    grayImg.release();
    undistImg.release();
    */
}

void MainWindow::cv_show(bool dynamic) {
    const string tag = __FUNCTION__;
    // stop timer timeout

    if(dynamic) {
        timer_imgproc->start(DELAY_CAPTURE_20MS);
    }

    if(cv_image.empty()) {
        cv_debug("cv_image is empty", tag);
        return;
    }
    cv_qtshow(cv_image, QImage::Format_RGB888);
    //tam thoi se khong hien thi duoc anh tu cv_image do timer chay lien tuc.

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
        if( cv_getPattern2CalibCamera(grayImage, (float)0.025, patternSize,
                                      listImagePoints, listRealPoints) == false ) {

            m_ui->pushButton_Calib->setEnabled(true);
            listImagePoints.clear();
            listRealPoints.clear();
            grayImage.release();
            return;
        }
    }

    cv_calibrateCamera(listImagePoints, listRealPoints, grayImage.size());

    vector<Point2f> imagePoints, realPoints;
    if( cv_getPattern2CalibRobot(grayImage, (float)25.0, patternSize,
                                 imagePoints, realPoints) == false ) {

        m_ui->pushButton_Calib->setEnabled(true);
        listImagePoints.clear();
        listRealPoints.clear();
        grayImage.release();
        imagePoints.clear();
        realPoints.clear();
        return;
    }
    cv_calibrateRobot(imagePoints, realPoints);

    listImagePoints.clear();
    listRealPoints.clear();
    grayImage.release();
    imagePoints.clear();
    realPoints.clear();
    m_ui->pushButton_Calib->setEnabled(true);
}

bool MainWindow::cv_convertToRealPoint(Point2f imagePoint, Point2f &realPoint) {
    const string tag = __FUNCTION__;
    Mat rMat, sMat, tMat;
    Mat imgPoint = (Mat_<double>(2, 1) << imagePoint.x, imagePoint.y);
    cv_readMatFromFile(R1to0Mat, PathCordinateConvertPara, rMat);
    cv_readMatFromFile(S1to0Mat, PathCordinateConvertPara, sMat);
    cv_readMatFromFile(T1to0Mat, PathCordinateConvertPara, tMat);
    if(rMat.empty() || sMat.empty() || tMat.empty()) {
        cv_debug("rMat or sMat or tMat empty", tag);
        rMat.release();
        sMat.release();
        tMat.release();
        return false;
    }
    Mat res = sMat * rMat * imgPoint + tMat;
    // Mat.at<type>( row, col);
    realPoint.x = res.at<double>(0, 0);
    realPoint.y = res.at<double>(1, 0);

    rMat.release();
    sMat.release();
    tMat.release();
    res.release();
    return true;
}

bool MainWindow::cv_undistortImage(Mat grayImage, Mat &undistortImage) {
    const string tag = __FUNCTION__;

    if(grayImage.empty()) {
        cv_debug("grayImage is empty", tag);
        return false;
    }
    Mat camMat, distortMat;
    cv_readMatFromFile(CamMat, PathIntrincyPara, camMat);
    cv_readMatFromFile(DistMat, PathIntrincyPara, distortMat);
    if(camMat.empty() || distortMat.empty()) {
        cv_debug("camMat or distortMat input is empty", tag);
        QMessageBox::critical(this, tr("Error"), tr("Need calib camera first"));

        camMat.release();
        distortMat.release();
        return false;
    }
    undistort( grayImage, undistortImage, camMat, distortMat);
    camMat.release();
    distortMat.release();
    return true;
}

// + undistort image
bool MainWindow::cv_getPattern2CalibRobot(Mat grayImage, float realPointDistance,
                                          Size patternSize, vector<Point2f> &imagePoints,
                                          vector<Point2f> &realPoints) {
    const string tag = __FUNCTION__;

    Mat undistortImage;
    if(cv_undistortImage(grayImage, undistortImage) == false) {
        cv_debug("undistort error", tag);
        undistortImage.release();
        return false;
    }

    Point2f Obset(- 4.0*realPointDistance, 1*realPointDistance + 50); // mm
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
        undistortImage.release();
        return false;
    }
    // debug image
    drawChessboardCorners(undistortImage, patternSize, imagePoints, true);
    cv_debugImage(undistortImage, true);
    undistortImage.release();
    return true;
}

void MainWindow::cv_calibrateRobot( vector<Point2f> vecImagePoints,
                                    vector<Point2f> vecRobotPoints) {
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

    FileStorage file(getNode[PathCordinateConvertPara], FileStorage::WRITE );
    file << getNode[R1to0Mat] << R1to0;
    file << getNode[T1to0Mat] << T1to0;
    file << getNode[S1to0Mat] << S1to0;

    file.release();
    point0.release();
    point1.release();
    R1to0.release();
    T1to0.release();
    S1to0.release();
}

void MainWindow::cv_calibrateCamera( vector<vector<Point2f>> listImagePoints,
                                     vector<vector<Point3f>> listRealPoints,
                                     Size imageSize) {
    const string tag = __FUNCTION__;
    // load camera Matrix
    Mat camMat, distortMat, rMat, tMat;
    cv_readMatFromFile(CamMat, PathIntrincyPara, camMat);
    if(camMat.empty()) {
        calibrateCamera(listRealPoints, listImagePoints, imageSize, camMat,
                        distortMat, rMat, tMat, CALIB_FIX_PRINCIPAL_POINT);
        cv_debug("calib with new camMat", tag);
    } else {
        calibrateCamera(listRealPoints, listImagePoints, imageSize, camMat,
                        distortMat, rMat, tMat, CALIB_FIX_PRINCIPAL_POINT +
                        CALIB_USE_INTRINSIC_GUESS);
        cv_debug("calib with old camMat", tag);
    }
    FileStorage file(getNode[PathIntrincyPara], FileStorage::WRITE );
    file << getNode[CamMat] << camMat;
    file << getNode[DistMat] << distortMat;

    file.release();
    camMat.release();
    distortMat.release();
    rMat.release();
    tMat.release();
}

bool MainWindow::cv_getPattern2CalibCamera(Mat grayImage, float realPointDistance,
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
        imagePoints.clear();
        realPoints.clear();
        return false;
    }
    if( findChessboardCorners(grayImage, patternSize, imagePoints,
                              CALIB_CB_ADAPTIVE_THRESH +
                              CALIB_CB_NORMALIZE_IMAGE +
                              CALIB_CB_FAST_CHECK) ) {
        listImagePoints.push_back(imagePoints);
        listRealPoints.push_back(realPoints);
    } else {
        cv_debug("can't find corner", tag);
        imagePoints.clear();
        realPoints.clear();
        return false;
    }

    imagePoints.clear();
    realPoints.clear();
    return true;
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

void MainWindow::cv_saveImageFromROI() {
    const string tag = __FUNCTION__;
    if(cv_image.empty()) {
        cv_debug("cv image is empty", tag);
        return;
    }
    imwrite(getNode[PathObjectImageSave], cv_image);
    QMessageBox::information(this, tr("image"), tr("save ok"));
    emit cv_signalShow(true);
}

void MainWindow::cv_readMatFromFile(cv_fileString node, cv_fileString filepath,
                                    Mat &mat, bool debug, const string tag) {
    FileStorage file(getNode[filepath], FileStorage::READ );
    file[getNode[node]] >> mat;
    if(debug) {
        cv_debug(mat, tag);
    }
    file.release();
}

void MainWindow::cv_debug(auto mat, const string tag) {
    ostringstream oss;
    if(tag == "") {
        oss << mat << endl;
    } else {
        oss << tag << ":" << endl << mat << endl;
    }
    qDebug().noquote() << oss.str().c_str();
}

// emit signal to show image from input.
// can use dynamic flag to show image for loop.
void MainWindow::cv_debugImage( Mat image, bool dynamic) {
    timer_imgproc->stop();
    if(image.empty()) {
        cv_image = cv_getImageFromCamera();
    } else {
        cv_image = image.clone();
    }
    emit cv_signalShow(dynamic);
}

void MainWindow::cv_qtshow(Mat img, QImage::Format format) {
    Mat temp;
    cvtColor(img, temp, COLOR_BGR2RGB);
    QImage* qimage = new QImage(temp.data, temp.cols, temp.rows, temp.step, format);
    m_ui->label_Camera_show->setFixedSize(qimage->size());
    m_ui->label_Camera_show->setPixmap(QPixmap::fromImage(*qimage));
    temp.release();
    img.release();
    delete qimage;
}

Mat MainWindow::cv_getMatFromQPixmap(QPixmap pixmap)
{
    QImage qimg = pixmap.toImage().convertToFormat(QImage::Format_RGB888).rgbSwapped();
    return Mat(qimg.height(), qimg.width(), CV_8UC3, qimg.bits(), qimg.bytesPerLine()).clone();
}

bool MainWindow::cv_sendRequest(cv_commandString cmd, const QString para) {
    const string tag = __FUNCTION__;
    if( robot_sendResq(cmd, para) == false) {
        return false;
    }
    return true;
}

void MainWindow::on_pushButton_Calib_clicked()
{
    m_ui->pushButton_Calib->setEnabled(false);
    emit cv_signalCalib();
}

void MainWindow::on_pushButton_Run_clicked()
{
    emit cv_signalAutoRun();
}
void MainWindow::on_pushButton_SaveImg_clicked()
{
    cv_saveImageFromROI();
}

void MainWindow::cv_getROI()
{
    QRect rect = m_ui->label_Camera_show->getRect();
    cv_image = cv_getMatFromQPixmap(m_ui->label_Camera_show->grab(rect));
    cv_debugImage(cv_image);
}

void MainWindow::on_pushButton_ShowCamera_clicked()
{
    emit cv_signalShow(true);
}

















