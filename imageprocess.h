#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/calib3d.hpp"
#include "opencv4/opencv2/core/types.hpp"
#include "opencv4/opencv2/xfeatures2d.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <math.h>

#include <QMainWindow>
#include <QDebug>
#include <QTimer>
#include <QImage>
#include <QtCore>
#include <QPixmap>

#define I_DEBUG(message)  ImageProcess::deBug(__FILE__, __LINE__, __FUNCTION__, message)

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

class ImageProcess:public VideoCapture
{
public:
    explicit ImageProcess();
    virtual ~ImageProcess();
    static void deBug(string file, int line, string function, auto message);
    enum nodePath_t{
        CamMat = 0,
        DistMat,
        R1to0Mat,
        T1to0Mat,
        S1to0Mat,
        PathIntrincyPara,
        PathCordinateConvertPara,
        PathObjectImageSave,
        NumOfMat,
    };
    static const char* NODEPATH[NumOfMat];

public: // static function don't need call object
    // unused
    static void basicLinearTransformContrast(Mat img, double alpha_, int beta_, Mat &res);
    static void avgVectorPoints(vector<Point2f> vecPoint, Point2f &means);
    static void hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                       vector<vector<int> > &pointInGroup_idx);
    static void drawRoiAroundObject(Mat imgObject, Mat imgScene, vector<Point2f> PointObject,
                                    vector<Point2f> PointScene, vector<vector<int> > groupPointIdx);

    // used
    static const char* getNode(nodePath_t node);
    static bool undistortImage(Mat grayImage, Mat &undistortImage);
    static bool getPattern2CalibCamera(Mat grayImage, float realPointDistance, Size patternSize,
                                       vector<vector<Point2f> > &listImagePoints,
                                       vector<vector<Point3f> > &listRealPoints);
    static bool getPattern2CalibRobot(Mat grayImage, float realPointDistance, Size patternSize,
                                      vector<Point2f> &imagePoints, vector<Point2f> &realPoints);
    static void getMatchesFromObject(Mat object, Mat scene, vector<Point2f> &keypoint_object,
                                     vector<Point2f> &keypoint_scene);

    static void calibCamera(vector<vector<Point2f> > listImagePoints,
                            vector<vector<Point3f>> listRealPoints, Size imageSize);
    static void calibRobot(vector<Point2f> vecImagePoints, vector<Point2f> vecRobotPoints);

    static bool convertToRealPoint(Point2f imagePoint, Point2f &realPoint);

    static void gammaCorrectionContrast(Mat img, double gamma_, Mat &res);

    static void getMatFromFile(nodePath_t node, nodePath_t filepath,
                                Mat &mat, bool debug = false);
    static Mat getMatFromQPixmap(QPixmap pixmap);

public: // non static function => must call object
    void getImage(Mat &image);
    void setImage(Mat image);
    void setHSVRange(Scalar _hsv_high, Scalar _hsv_low);
    Mat getImageFromCamera(ColorConversionCodes flag = COLOR_BGR2BGRA);
    void preProcess();


private:
    Mat img;
    Scalar hsv_high, hsv_low;
};

#endif // IMAGEPROCESS_H
