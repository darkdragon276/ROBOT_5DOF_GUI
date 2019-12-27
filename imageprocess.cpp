#include "imageprocess.h"

const char* ImageProcess::NODEPATH[ImageProcess::NumOfMat] = {
    "CameraIntrincyMatrix",
    "DistortExtrincyMatrix",
    "CordinatesRotateMatrix",
    "CordinatesTranferMatrix",
    "CordinatesScaleMatrix",
    "AlphaParameter",
    "BetaParameter",
    "GammaParameter",
    "ThreshBinaryParameter",
    "HSVMaxParameter",
    "HSVMinParamter",
    "cameraMatrix.yml",
    "robotMatrix.yml",
    "./images/object.jpg",
    "./images/base.jpg",
    "dipParameter.yml",
};

ImageProcess::ImageProcess()
{
    this->set(CAP_PROP_FPS, DIP_CAMERA_FPS);
}

ImageProcess::~ImageProcess()
{

}

void ImageProcess::init()
{
    _timer_release_buffer = new QTimer(this);
    QObject::connect(_timer_release_buffer, &QTimer::timeout, this, &ImageProcess::_grabImage);
    _timer_release_buffer->start(DIP_TIMER_FPS);
}

void ImageProcess::denit()
{
    Debug::_delete(img);
    _timer_release_buffer->stop();
}

void ImageProcess::_grabImage()
{
    this->grab();
}

void ImageProcess::deBugImage(Mat img)
{
    setMode(ModeDebug);
    setImage(img);
}

const char *ImageProcess::getNode(nodePath_t node)
{
    return NODEPATH[node];
}

void ImageProcess::getMatFromFile(nodePath_t node, nodePath_t filepath,
                                  Mat &mat, bool debug)
{
    FileStorage file(getNode(filepath), FileStorage::READ );
    file[getNode(node)] >> mat;
    if(debug) {
        M_DEBUG(mat);
    }
    file.release();
}

void ImageProcess::getParameterFromFile(double &_alpha, double &_beta, double &_gamma,
                                        double &_threshbinary, Scalar &_hsv_max, Scalar &_hsv_min)
{
    FileStorage file(getNode(PathDIPParameter), FileStorage::READ );
    file[getNode(AlphaDouble)] >> _alpha;
    file[getNode(BetaDouble)] >> _beta;
    file[getNode(GammaDouble)] >> _gamma;
    file[getNode(ThreshBinaryDouble)] >> _threshbinary;
    file[getNode(HSVMaxScalar)] >> _hsv_max;
    file[getNode(HSVMinScalar)] >> _hsv_min;
    file.release();
}

Mat ImageProcess::getMatFromQPixmap(QPixmap pixmap)
{
    QImage qimg = pixmap.toImage().convertToFormat(QImage::Format_RGB888).rgbSwapped();
    return Mat(qimg.height(), qimg.width(), CV_8UC3, qimg.bits(), qimg.bytesPerLine()).clone();
}

bool ImageProcess::undistortImage(Mat grayImage, Mat &undistortImage) {
    if(grayImage.empty()) {
        M_DEBUG("grayImage is empty");
        return false;
    }
    Mat camMat, distortMat;
    getMatFromFile(CamMat, PathIntrincyPara, camMat);
    getMatFromFile(DistMat, PathIntrincyPara, distortMat);
    if(camMat.empty() || distortMat.empty()) {
        M_DEBUG("camMat or distortMat input is empty");
        Debug::_delete(camMat, distortMat);
        return false;
    }
    Mat tempImage;
    undistort( grayImage, tempImage, camMat, distortMat);
    undistortImage.release();
    undistortImage = tempImage.clone();

    Debug::_delete(tempImage, camMat, distortMat);
    return true;
}

bool ImageProcess::getPattern2CalibCamera(Mat grayImage, float realPointDistance,
                                          Size patternSize, vector<vector<Point2f>> &listImagePoints,
                                          vector<vector<Point3f>> &listRealPoints)
{
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
        M_DEBUG("grayImage input is empty");
        Debug::_delete(imagePoints, realPoints);
        return false;
    }
    if( findChessboardCorners(grayImage, patternSize, imagePoints,
                              CALIB_CB_ADAPTIVE_THRESH +
                              CALIB_CB_NORMALIZE_IMAGE +
                              CALIB_CB_FAST_CHECK) ) {
        listImagePoints.push_back(imagePoints);
        listRealPoints.push_back(realPoints);
    } else {
        M_DEBUG("can't find corner");
        Debug::_delete(imagePoints, realPoints);
        return false;
    }

    Debug::_delete(imagePoints, realPoints);
    return true;
}

// + undistort image
bool ImageProcess::getPattern2CalibRobot(Mat grayImage, float realPointDistance,
                                         Size patternSize, vector<Point2f> &imagePoints,
                                         vector<Point2f> &realPoints) {
    Mat uImage = grayImage.clone();
    Point2f Obset(- 3.5*realPointDistance, 1*realPointDistance + 50); // mm
    if( findChessboardCorners(uImage, patternSize, imagePoints,
                              CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
                              CALIB_CB_FAST_CHECK) ) {
        for(int y = patternSize.height - 1 ; y > -1 ; y--) {
            for(int x = patternSize.width - 1  ; x > -1  ; x--) {
                realPoints.push_back(Point2f((float)x * realPointDistance,
                                             (float)y * realPointDistance) + Obset);
            }
        }
    } else {
        M_DEBUG("can't find corner");
        Debug::_delete(uImage);
        return false;
    }
    // debug image
    drawChessboardCorners(uImage, patternSize, imagePoints, true);
    deBugImage(uImage);
    Debug::_delete(uImage);
    return true;
}

void ImageProcess::calibCamera( vector<vector<Point2f>> listImagePoints,
                                vector<vector<Point3f>> listRealPoints,
                                Size imageSize) {
    // load camera Matrix
    Mat camMat, distortMat, rMat, tMat;
    getMatFromFile(CamMat, PathIntrincyPara, camMat);
    if(camMat.empty()) {
        calibrateCamera(listRealPoints, listImagePoints, imageSize, camMat,
                        distortMat, rMat, tMat, CALIB_FIX_PRINCIPAL_POINT);
        M_DEBUG("calib with new camMat");
    } else {
        calibrateCamera(listRealPoints, listImagePoints, imageSize, camMat,
                        distortMat, rMat, tMat, CALIB_FIX_PRINCIPAL_POINT +
                        CALIB_USE_INTRINSIC_GUESS);
        M_DEBUG("calib with old camMat");
    }
    FileStorage file(getNode(PathIntrincyPara), FileStorage::WRITE );
    file << getNode(CamMat) << camMat;
    file << getNode(DistMat) << distortMat;

    file.release();
    Debug::_delete(camMat, distortMat, rMat, tMat);
}

void ImageProcess::calibRobot( vector<Point2f> vecImagePoints,
                               vector<Point2f> vecRobotPoints) {
    if( (vecImagePoints.size() != vecRobotPoints.size()) || vecImagePoints.empty() ) {
        M_DEBUG("2 source.size is different or source is empty");
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

    FileStorage file(getNode(PathCordinateConvertPara), FileStorage::WRITE );
    file << getNode(R1to0Mat) << R1to0;
    file << getNode(T1to0Mat) << T1to0;
    file << getNode(S1to0Mat) << S1to0;

    file.release();
    Debug::_delete(point0, point1, R1to0, T1to0, S1to0);
}

void ImageProcess::getMatchesFromObject(Mat object, Mat scene, vector<Point2f> &keypoint_object,
                                        vector<Point2f> &keypoint_scene)
{
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    if(object.empty() || scene.empty()) {
        M_DEBUG("input empty");
        return;
    }
    Mat img_object = object.clone(), img_scene = scene.clone();
    Ptr<SURF> detector = SURF::create( 100.0, 20, 3, false, false );
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
    Debug::_delete(detector, keypoints_object, keypoints_scene, descriptors_object,
                   descriptors_scene, matcher, matches, knn_matches, img_object, img_scene);
}

void ImageProcess::homographyTranform(Mat imgObject, Mat imgScene, vector<Point2f> PointObject,
                                      vector<Point2f> PointScene, vector<vector<int>> groupPointIdx,
                                      vector<Point2f> &_vec_center)
{
    vector<Point2f> object, scene;
    vector<Point2f> centerObject;
    centerObject.push_back(Point2f((double)(imgObject.rows / 2), (double)(imgObject.cols / 2)));
    vector<Point2f> centerScene;

    if(!_vec_center.empty()) {
        _vec_center.clear();
    }
    Mat H;
    for(size_t i = 0; i < groupPointIdx.size(); i++) {
        for(size_t z = 0; z <groupPointIdx.at(i).size(); z++) {
            object.push_back(PointObject.at(groupPointIdx.at(i).at(z)));
            scene.push_back(PointScene.at(groupPointIdx.at(i).at(z)));
        }
        H.release();
        H = findHomography(object, scene, RANSAC);
        if(H.empty()) {
            M_DEBUG("H empty");
        } else {
            perspectiveTransform(centerObject, centerScene, H);
            _vec_center.push_back(Filter::meansVectorPoints(centerScene));
            circle(imgScene, _vec_center.back(), 3, Scalar(255, 0, 255), 3);
            M_DEBUG(_vec_center.back());
            M_DEBUG("well");
        }
        object.clear();
        scene.clear();
    }
    Debug::_delete(object, scene, centerObject, centerScene, H);
}

bool ImageProcess::toReal(Point2f imagePoint, Point2f &realPoint) {
    Mat rMat, sMat, tMat;
    Mat imgPoint = (Mat_<double>(2, 1) << imagePoint.x, imagePoint.y);
    getMatFromFile(R1to0Mat, PathCordinateConvertPara, rMat);
    getMatFromFile(S1to0Mat, PathCordinateConvertPara, sMat);
    getMatFromFile(T1to0Mat, PathCordinateConvertPara, tMat);
    if(rMat.empty() || sMat.empty() || tMat.empty()) {
        M_DEBUG("rMat or sMat or tMat empty");
        Debug::_delete(rMat, sMat, tMat, imgPoint);
        return false;
    }
    Mat res = sMat * rMat * imgPoint + tMat;
    // Mat.at<type>( row, col);
    realPoint.x = res.at<double>(0, 0);
    realPoint.y = res.at<double>(1, 0);

    Debug::_delete(rMat, sMat, tMat, imgPoint, res);
    return true;
}

bool ImageProcess::toReal(double _img_pixel, double &_real_distance)
{
    Mat rMat, sMat, tMat;
    Mat imgPoint_origin = (Mat_<double>(2, 1) << 0, 0);
    Mat imgPoint_distance = (Mat_<double>(2,1) << 0, _img_pixel);
    getMatFromFile(R1to0Mat, PathCordinateConvertPara, rMat);
    getMatFromFile(S1to0Mat, PathCordinateConvertPara, sMat);
    getMatFromFile(T1to0Mat, PathCordinateConvertPara, tMat);
    if(rMat.empty() || sMat.empty() || tMat.empty()) {
        M_DEBUG("rMat or sMat or tMat empty");
        Debug::_delete(rMat, sMat, tMat, imgPoint_origin, imgPoint_distance);
        return false;
    }
    Mat real_origin = sMat * rMat * imgPoint_origin + tMat;
    Mat real_distance = sMat * rMat * imgPoint_distance + tMat;
    Point2f real_point_origin(real_origin.at<double>(0,0), real_origin.at<double>(0,1));
    Point2f real_point_distance(real_distance.at<double>(0,0), real_distance.at<double>(0,1));
    _real_distance = norm(real_point_origin - real_point_distance);

    Debug::_delete(rMat, sMat, tMat, imgPoint_origin, imgPoint_distance,
                   real_origin, real_distance);
    return true;
}

void ImageProcess::gammaCorrectionContrast(Mat _img, double gamma_, Mat &res)
{
    CV_Assert(gamma_ >= 0);
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i) {
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);
    }
    res = _img.clone();
    LUT(_img, lookUpTable, res);
    Debug::_delete(lookUpTable);
}

void ImageProcess::basicLinearTransformContrast(Mat _img, double alpha_, int beta_, Mat &res)
{
    _img.convertTo(res, -1, alpha_, beta_);
}

void ImageProcess::getImage(Mat &image)
{
    image.release();
    img.assignTo(image);
}

void ImageProcess::setImage(Mat image)
{
    if(!image.empty()) {
        image.assignTo(img);
        return;
    }
    M_DEBUG("image is empty");
}

void ImageProcess::setPreProcessParameter(double _gamma, double _alpha, double _beta,
                                          double _threshbinary, Scalar _hsv_high, Scalar _hsv_low)
{
    gamma = _gamma;
    alpha = _alpha;
    beta = _beta;
    threshbinary = _threshbinary;
    hsv_high = _hsv_high;
    hsv_low = _hsv_low;
    setParameterIntoFile();
}

// save parameter with alpha, gamma, beta.
void ImageProcess::setParameterIntoFile()
{
    FileStorage file(getNode(PathDIPParameter), FileStorage::WRITE );
    file << getNode(AlphaDouble) << alpha;
    file << getNode(BetaDouble) << beta;
    file << getNode(GammaDouble) << gamma;
    file << getNode(ThreshBinaryDouble) << threshbinary;
    file << getNode(HSVMaxScalar) << hsv_high;
    file << getNode(HSVMinScalar) << hsv_low;
    file.release();
}

Mat ImageProcess::getImageFromCamera( ColorConversionCodes flag) {
    if(!this->isOpened()) {
        M_DEBUG("camera is close");
        return Mat();
    }
    Mat imageColor, imageConvert;
    this->retrieve(imageColor);
    if(flag == COLOR_BGR2BGRA) {
        imageConvert.release();
        return imageColor;
    } else {
        cvtColor(imageColor, imageConvert, flag);
        imageColor.release();
        return imageConvert;
    }
}

void ImageProcess::setMode(ImageProcess::processMode_t _mode)
{
    mode = _mode;
}

void ImageProcess::setVecObjects(vector<Filter::Object_t> vec_objects)
{
    objects_detected.clear();
    if(vec_objects.empty()) {
        M_DEBUG("vec_object input is zero");
        return;
    }
    objects_detected = vec_objects;
}

bool ImageProcess::getObject(Filter::Object_t& _object, int index)
{
    if(objects_detected.empty()) {
        M_DEBUG("no object detect");
        return false;
    }
    if(index >= (int)objects_detected.size()) {
        M_DEBUG("index larger than object num");
        return false;
    }
    _object = {Point2f(0, 0), 0, 0};
    _object = objects_detected.at(index);
    return true;
}

void ImageProcess::process()
{
    if(mode == ModeDebug) {
        return;
    }

    // preprocessing
    Mat raw_img = getImageFromCamera();
    Mat gamma_img, linear_img, blur_img;
    gammaCorrectionContrast(raw_img, gamma, gamma_img);
    basicLinearTransformContrast(raw_img, alpha, (int)beta, linear_img);
    medianBlur (linear_img, blur_img, 3);
    if(mode == ModeNull) {
        setImage(raw_img);
        Debug::_delete(raw_img, gamma_img, linear_img, blur_img);
        return;
    }
    if(mode == ModeShowConvertImage) {
        setImage(blur_img);
        Debug::_delete(raw_img, gamma_img, linear_img, blur_img);
        return;
    }

    // init Base
    if(process_num != 0) {
        Mat initbase_img = blur_img.clone();
        initBase(initbase_img);
        process_num --;
    }
    // remove base mask
    Mat color_temp, bitwise_img;
    cvtColor(base_mask, color_temp, COLOR_GRAY2BGR);
    bitwise_or(color_temp, blur_img, bitwise_img);
    if(mode == ModeNonBase) {
        setImage(bitwise_img);
        Debug::_delete(raw_img, gamma_img, linear_img, blur_img,
                       color_temp, bitwise_img);
        return;
    }

    // basic processing
    vector<Filter::Object_t> vec_objects;
    vec_objects.clear();
    Mat basic_img = bitwise_img.clone();
    basicProcess(basic_img, vec_objects);
    if(mode == ModeBasicProcessing) {
        exPort(raw_img, vec_objects);
        Debug::_delete(raw_img, gamma_img, linear_img, blur_img,
                       basic_img, vec_objects);
        return;
    }

    // suft matching
    Mat suft_img = bitwise_img.clone();
    suftProcess(suft_img, vec_objects);
    if(mode == ModeSUFT) {
        exPort(raw_img, vec_objects);
        Debug::_delete(raw_img, gamma_img, linear_img, blur_img,
                       basic_img, suft_img, vec_objects);
        return;
    }
}

void ImageProcess::basicProcess(Mat &color_img, vector<Filter::Object_t> &vec_objects)
{
    Mat temp_img = color_img.clone();
    Mat gray_img, thresh_img, hsv_img;
    cvtColor(temp_img, gray_img, COLOR_BGR2GRAY);
    threshold(gray_img, thresh_img, threshbinary, 255, THRESH_BINARY);
    // find contour
    vector<vector<Point>> object_contours;
    vector<vector<Point>> contours;
    vector<Point> approx;
    findContours(thresh_img, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    for( size_t i = 0; i < contours.size(); i++ ) {
        approx.clear();
        approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.01, true);
        if(fabs(contourArea(approx)) > 2000 && fabs(contourArea(approx)) < 30000 &&
                isContourConvex(approx)) {
            object_contours.push_back(approx);
        }
    }
    if(object_contours.empty()) {
        M_DEBUG("shape empty");
        Debug::_delete(object_contours, contours, approx, temp_img, gray_img, thresh_img, hsv_img);
        return;
    }

    // object finding...
    vec_objects.clear();
    contour_filter.setVecContour(object_contours);
    while(!contour_filter.isReadyGet(vec_objects));
    Debug::_delete(object_contours, contours, approx, temp_img, hsv_img,
                   gray_img, thresh_img);
}

void ImageProcess::suftProcess(Mat &color_img, vector<Filter::Object_t> &vec_objects)
{
    if(vec_objects.empty()) {
        M_DEBUG("no object detect");
        return;
    }

    Mat img_object = imread(getNode(PathObjectSave));
    vector<int> erase_idx;
    for(size_t i = 0; i < vec_objects.size(); i++) {
        vector<Point2f> matchObjectPoint;
        vector<Point2f> matchScenePoint;
        Rect roi = Rect((int)(vec_objects.at(i).center.x - vec_objects.at(i).radius_img - OBSET_SUFT_ROI),
                        (int)(vec_objects.at(i).center.y - vec_objects.at(i).radius_img - OBSET_SUFT_ROI),
                        (int)(vec_objects.at(i).radius_img*2 + OBSET_SUFT_ROI),
                        (int)(vec_objects.at(i).radius_img*2 + OBSET_SUFT_ROI));
        if(roi.br().x > color_img.cols || roi.br().y > color_img.rows
                || roi.tl().x < 0 || roi.tl().y < 0) {
            continue;
        }

        Mat img_scene = Mat(color_img, roi);
        getMatchesFromObject(img_object, img_scene, matchObjectPoint, matchScenePoint);
        // don't have key point erase
        if(matchScenePoint.size() < 5) {
            erase_idx.push_back(i);
        }
    }
    for(size_t i = 0; i < erase_idx.size(); i++) {
        vector<Filter::Object_t>::iterator vec_objects_idx = vec_objects.begin();
        advance(vec_objects_idx, erase_idx.at(i) - i);
        vec_objects.erase(vec_objects_idx);
    }

    if(vec_objects.empty()) {
        M_DEBUG("no object detect");
    }
    vec_objects.shrink_to_fit();
    //    M_DEBUG(vec_objects.size());
    suft_filter.setVecObject(vec_objects);
    while(!suft_filter.isReadyGet(vec_objects));
    Debug::_delete(img_object);
}

void ImageProcess::exPort(Mat &image, vector<Filter::Object_t> &vec_objects)
{
    setVecObjects(vec_objects);
    foreach (Filter::Object_t object, vec_objects) {
        ostringstream strs;
        strs << (int)object.radius_img;
        circle(image, object.center, 2, Scalar(255, 255, 0), 2);
        putText(image, strs.str(), object.center, FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255), 2 , 8 , false);
    }
    setImage(image);
}

void ImageProcess::initBase(Mat rgb_img)
{
    if(rgb_img.empty()) {
        M_DEBUG("input empty");
        return;
    }

    Mat base_img = imread(getNode(PathBaseAreaSave));
    if(base_img.empty() || rgb_img.empty()) {
        M_DEBUG("Base Image or Input Image empty");
        return;
    }

    Scalar rgb_base_mean = mean(base_img);
    Scalar rgb_low = Scalar(OBSET_RANGE_RGB, OBSET_RANGE_RGB, OBSET_RANGE_RGB, 0);
    Scalar rgb_high = Scalar(OBSET_RANGE_RGB, OBSET_RANGE_RGB, OBSET_RANGE_RGB, 255);
    Mat rgb_mask, rgb_dilate, rgb_erode, rgb_blur, rgb_thresh;
    inRange(rgb_img, rgb_base_mean - rgb_low, rgb_base_mean + rgb_high, rgb_mask);

    // remove some tiny hole
    int dilation_size = 11;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(dilation_size, dilation_size));
    dilate(rgb_mask, rgb_dilate, element);
    blur(rgb_dilate, rgb_blur, Size(5, 5));
    threshold(rgb_blur, rgb_thresh, 100, 255, THRESH_BINARY);
    erode(rgb_thresh, rgb_erode, element);

    if(process_num == MAX_PROCESS_NUM) {
        base_mask.release();
        base_mask = rgb_erode;
    } else {
        Mat bitwise_img;
        Mat pre_mask = base_mask;
        bitwise_or(pre_mask, rgb_erode, bitwise_img);
        base_mask.release();
        base_mask = bitwise_img;
    }
    //    I_DEBUG(base_mask);
    //    M_DEBUG(rgb_base_mean);

    if(process_num == 1) {
        vector<vector<Point>> base_contours;
        vector<vector<Point>> contours;
        vector<Point> approx;
        findContours(base_mask, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
        for( size_t i = 0; i < contours.size(); i++ ) {
            approx.clear();
            approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.01, true);
            if(fabs(contourArea(approx)) > 2000 && fabs(contourArea(approx)) < 30000 &&
                    isContourConvex(approx)) {
                base_contours.push_back(approx);
            }
        }
        if(base_contours.size() != 1) {
            M_DEBUG(tr("over than 1 base: %1").arg(base_contours.size()));
            return;
        }
        base_center = Filter::meansVectorPoints(Filter::toVectorPoint2f(base_contours.at(0)));
    }
}

void ImageProcess::setBase()
{
    process_num = MAX_PROCESS_NUM;
}

Point2f ImageProcess::getBaseCenter()
{
    return base_center;
}
