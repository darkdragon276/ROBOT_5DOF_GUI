#include "imageprocess.h"

const char* ImageProcess::NODEPATH[ImageProcess::NumOfMat] = {
    "CameraIntrincyMatrix",
    "DistortExtrincyMatrix",
    "CordinatesRotateMatrix",
    "CordinatesTranferMatrix",
    "CordinatesScaleMatrix",
    "cameraMatrix.yml",
    "robotMatrix.yml",
    "./images/object.jpg"
};

ImageProcess::ImageProcess()
{

}

ImageProcess::~ImageProcess()
{

}

void ImageProcess::deBug(string file, int line, string function, auto message)
{
    ostringstream oss;
    oss << file << "(" << line << ")" << "/" << function << ":" << message << endl;
    qDebug().noquote() << oss.str().c_str();
}

//-- Function return point is devided into group.
//-- pointInGroup_idx is reference from point_list.
void ImageProcess::hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                           vector<vector<int>> &pointInGroup_idx) {
    if(point_list.empty()) {
        I_DEBUG("error empty input");
        return;
    }
    if(max_distance <= 0) {
        I_DEBUG("error distance input");
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
    I_DEBUG(pointInGroup_idx.size());
}

void ImageProcess::getMatchesFromObject(Mat object, Mat scene, vector<Point2f> &keypoint_object,
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

void ImageProcess::avgVectorPoints(vector<Point2f> vecPoint, Point2f &means)
{
    if(vecPoint.empty()) {
        I_DEBUG("input error");
        return;
    }
    Point2f temp(0, 0);
    for(size_t i = 0; i < vecPoint.size(); i ++) {
        temp += vecPoint.at(i);
    }
    means = Point2f(means.x / (double)vecPoint.size(), means.y / (double)vecPoint.size());
}

void ImageProcess::drawRoiAroundObject(Mat imgObject, Mat imgScene,
                                        vector<Point2f> PointObject,
                                        vector<Point2f> PointScene,
                                        vector<vector<int>> groupPointIdx)
{
    vector<Point2f> object, scene;
    vector<Point2f> centerObject;
    centerObject.push_back(Point2f((double)(imgObject.rows / 2), (double)(imgObject.cols / 2)));
    vector<Point2f> centerScene;

    for(size_t i = 0; i < groupPointIdx.size(); i++) {
        for(size_t z = 0; z <groupPointIdx.at(i).size(); z++) {
            object.push_back(PointObject.at(groupPointIdx.at(i).at(z)));
            scene.push_back(PointScene.at(groupPointIdx.at(i).at(z)));
        }

        Mat H = findHomography(object, scene, RANSAC);
        if(H.empty()) {
            I_DEBUG("H empty");
        } else {
            perspectiveTransform(centerObject, centerScene, H);
            circle(imgScene, centerScene.at(0), 10, Scalar(0, 0, 255));
            I_DEBUG("well");
        }
        object.clear();
        scene.clear();
    }
}

void ImageProcess::basicLinearTransformContrast(Mat img, double alpha_, int beta_, Mat &res)
{
    img.convertTo(res, -1, alpha_, beta_);
}

void ImageProcess::gammaCorrectionContrast(Mat img, double gamma_, Mat &res)
{
    CV_Assert(gamma_ >= 0);
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i) {
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);
    }
    res = img.clone();
    LUT(img, lookUpTable, res);
}

bool ImageProcess::convertToRealPoint(Point2f imagePoint, Point2f &realPoint) {
    Mat rMat, sMat, tMat;
    Mat imgPoint = (Mat_<double>(2, 1) << imagePoint.x, imagePoint.y);
    getMatFromFile(R1to0Mat, PathCordinateConvertPara, rMat);
    getMatFromFile(S1to0Mat, PathCordinateConvertPara, sMat);
    getMatFromFile(T1to0Mat, PathCordinateConvertPara, tMat);
    if(rMat.empty() || sMat.empty() || tMat.empty()) {
        I_DEBUG("rMat or sMat or tMat empty");
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

bool ImageProcess::undistortImage(Mat grayImage, Mat &undistortImage) {
    if(grayImage.empty()) {
        I_DEBUG("grayImage is empty");
        return false;
    }
    Mat camMat, distortMat;
    getMatFromFile(CamMat, PathIntrincyPara, camMat);
    getMatFromFile(DistMat, PathIntrincyPara, distortMat);
    if(camMat.empty() || distortMat.empty()) {
        I_DEBUG("camMat or distortMat input is empty");
        camMat.release();
        distortMat.release();
        return false;
    }
    Mat tempImage;
    undistort( grayImage, tempImage, camMat, distortMat);
    undistortImage.release();
    undistortImage = tempImage.clone();
    tempImage.release();
    camMat.release();
    distortMat.release();
    return true;
}

const char *ImageProcess::getNode(nodePath_t node)
{
    return NODEPATH[node];
}

// + undistort image
bool ImageProcess::getPattern2CalibRobot(Mat grayImage, float realPointDistance,
                                          Size patternSize, vector<Point2f> &imagePoints,
                                          vector<Point2f> &realPoints) {

    Mat uImage;
    if(undistortImage(grayImage, uImage) == false) {
        I_DEBUG("undistort error");
        uImage.release();
        return false;
    }

    Point2f Obset(- 4.0*realPointDistance, 1*realPointDistance + 50); // mm
    if( findChessboardCorners(uImage, patternSize, imagePoints,
                              CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
                              CALIB_CB_FAST_CHECK) ) {
        for(int y = 0 ; y < patternSize.height ; y++) {
            for(int x = 0 ; x < patternSize.width ; x++) {
                realPoints.push_back(Point2f( (float)x * realPointDistance,
                                              (float)y * realPointDistance ) + Obset);
            }
        }
    } else {
        I_DEBUG("can't find corner");
        uImage.release();
        return false;
    }
    // debug image
    drawChessboardCorners(uImage, patternSize, imagePoints, true);
    uImage.release();
    return true;
}

void ImageProcess::calibRobot( vector<Point2f> vecImagePoints,
                               vector<Point2f> vecRobotPoints) {
    if( (vecImagePoints.size() != vecRobotPoints.size()) || vecImagePoints.empty() ) {
        I_DEBUG("2 source.size is different or source is empty");
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
    point0.release();
    point1.release();
    R1to0.release();
    T1to0.release();
    S1to0.release();
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
        I_DEBUG("calib with new camMat");
    } else {
        calibrateCamera(listRealPoints, listImagePoints, imageSize, camMat,
                        distortMat, rMat, tMat, CALIB_FIX_PRINCIPAL_POINT +
                        CALIB_USE_INTRINSIC_GUESS);
        I_DEBUG("calib with old camMat");
    }
    FileStorage file(getNode(PathIntrincyPara), FileStorage::WRITE );
    file << getNode(CamMat) << camMat;
    file << getNode(DistMat) << distortMat;

    file.release();
    camMat.release();
    distortMat.release();
    rMat.release();
    tMat.release();
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
        I_DEBUG("grayImage input is empty");
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
        I_DEBUG("can't find corner");
        imagePoints.clear();
        realPoints.clear();
        return false;
    }

    imagePoints.clear();
    realPoints.clear();
    return true;
}

void ImageProcess::getMatFromFile(nodePath_t node, nodePath_t filepath,
                                    Mat &mat, bool debug)
{
    FileStorage file(getNode(filepath), FileStorage::READ );
    file[getNode(node)] >> mat;
    if(debug) {
        I_DEBUG(mat);
    }
    file.release();
}

void ImageProcess::setImage(Mat image)
{
    if(!image.empty()) {
        img.release();
        image.assignTo(img);
        return;
    }
    I_DEBUG("image is empty");
}

void ImageProcess::getImage(Mat &image)
{
    image.release();
    img.assignTo(image);
}
