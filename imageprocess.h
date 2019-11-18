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
        AlphaDouble,
        BetaDouble,
        GammaDouble,
        ThreshBinaryDouble,
        PathIntrincyPara,
        PathCordinateConvertPara,
        PathObjectImageSave,
        PathDIPParameter,
        NumOfMat,
    };
    static const char* NODEPATH[NumOfMat];

    typedef enum {
        ModeBasicProcessing = 5,
        ModeSUFT,
        ModeNull,
    }processMode_t;
public: // static function don't need call object
    // unused
    static void avgVectorPoints(vector<Point2f> vecPoint, Point2f &means);
    // support
    static const char* getNode(nodePath_t node);
    static void getMatFromFile(nodePath_t node, nodePath_t filepath,
                                Mat &mat, bool debug = false);
    static void getParameterFromFile(double &_alpha, double &_beta, double &_gamma, double &_threshbinary);
    static Mat getMatFromQPixmap(QPixmap pixmap);
    // to calib camera
    static bool undistortImage(Mat grayImage, Mat &undistortImage);
    static bool getPattern2CalibCamera(Mat grayImage, float realPointDistance, Size patternSize,
                                       vector<vector<Point2f> > &listImagePoints,
                                       vector<vector<Point3f> > &listRealPoints);
    static bool getPattern2CalibRobot(Mat grayImage, float realPointDistance, Size patternSize,
                                      vector<Point2f> &imagePoints, vector<Point2f> &realPoints);


    static void calibCamera(vector<vector<Point2f> > listImagePoints,
                            vector<vector<Point3f>> listRealPoints, Size imageSize);
    static void calibRobot(vector<Point2f> vecImagePoints, vector<Point2f> vecRobotPoints);

    // to matching and draw object roi
    static void getMatchesFromObject(Mat object, Mat scene, vector<Point2f> &keypoint_object,
                                     vector<Point2f> &keypoint_scene);

    static void hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                       vector<vector<int> > &pointInGroup_idx);

    static void drawRoiAroundObject(Mat imgObject, Mat imgScene, vector<Point2f> PointObject,
                                    vector<Point2f> PointScene, vector<vector<int> > groupPointIdx);
    static bool convertToRealPoint(Point2f imagePoint, Point2f &realPoint);
    // preprocessing image
    static void gammaCorrectionContrast(Mat img, double gamma_, Mat &res);
    static void basicLinearTransformContrast(Mat img, double alpha_, int beta_, Mat &res);

public: // non static function => must call object
    void getImage(Mat &image);
    void setImage(Mat image);
    void setHSVRange(Scalar _hsv_high, Scalar _hsv_low);
    void setPreProcessParameter(double _gamma, double _alpha, double _beta,
                                double _threshbinary);
    void setParameterIntoFile();

    Mat getImageFromCamera(ColorConversionCodes flag = COLOR_BGR2BGRA);
    void setMode(processMode_t _mode);

    void preProcess();




private:
    Mat img;
    Scalar hsv_high, hsv_low;
    double gamma = 1.0, alpha = 1.0, beta = 0.0;
    double threshbinary = 100;
    processMode_t mode = ModeNull;
};

#endif // IMAGEPROCESS_H
