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
#include <QObject>

#include "filter.h"
#include "debug.h"

#define I_DEBUG(image)      ImageProcess::deBugImage(image)
#define DIP_CAMERA_FPS      (60.f)
#define DIP_TIMER_FPS       (1000.f/DIP_CAMERA_FPS)
#define DIP_MAIN_FPS        (1000.f/10.f)

using namespace cv;
using namespace cv::xfeatures2d;

class ImageProcess:public VideoCapture, public QObject
{
public:
    ImageProcess();
    ~ImageProcess();
    void init();
    void denit();
    void _grabImage();
    void deBugImage(Mat img);
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
        HSVMaxScalar,
        HSVMinScalar,
        PathIntrincyPara,
        PathCordinateConvertPara,
        PathObjectSave,
        PathBaseAreaSave,
        PathDIPParameter,
        NumOfMat,
    };
    static const char* NODEPATH[NumOfMat];

    typedef enum {
        ModeShowConvertImage = 27,
        ModeNonBase,
        ModeBasicProcessing,
        ModeSUFT,
        ModeNull,
        ModeDebug,
    }processMode_t;
public: // static function don't need call object
    // support
    static const char* getNode(nodePath_t node);
    static void getMatFromFile(nodePath_t node, nodePath_t filepath,
                                Mat &mat, bool debug = false);
    static void getParameterFromFile(double &_alpha, double &_beta, double &_gamma,
                                     double &_threshbinary, Scalar &_hsv_max, Scalar &_hsv_min);
    static Mat getMatFromQPixmap(QPixmap pixmap);
    // to calib camera
    static bool undistortImage(Mat grayImage, Mat &undistortImage);
    bool getPattern2CalibCamera(Mat grayImage, float realPointDistance, Size patternSize,
                                       vector<vector<Point2f> > &listImagePoints,
                                       vector<vector<Point3f> > &listRealPoints);
    bool getPattern2CalibRobot(Mat grayImage, float realPointDistance, Size patternSize,
                                      vector<Point2f> &imagePoints, vector<Point2f> &realPoints);
    void calibCamera(vector<vector<Point2f> > listImagePoints,
                            vector<vector<Point3f>> listRealPoints, Size imageSize);
    void calibRobot(vector<Point2f> vecImagePoints, vector<Point2f> vecRobotPoints);

    // to matching and draw object roi
    static void getMatchesFromObject(Mat object, Mat scene, vector<Point2f> &keypoint_object,
                                     vector<Point2f> &keypoint_scene);
    static void homographyTranform(Mat imgObject, Mat imgScene, vector<Point2f> PointObject,
                                   vector<Point2f> PointScene, vector<vector<int> > groupPointIdx, vector<Point2f> &_vec_center);
    static bool toReal(Point2f imagePoint, Point2f &realPoint);
    static bool toReal(double _img_pixel, double &_real_distance);

    // preprocessing image
    static void gammaCorrectionContrast(Mat _img, double gamma_, Mat &res);
    static void basicLinearTransformContrast(Mat _img, double alpha_, int beta_, Mat &res);

public: // non static function => must call object
    void getImage(Mat &image);
    void setImage(Mat image);
    void setPreProcessParameter(double _gamma, double _alpha, double _beta,
                                double _threshbinary, Scalar _hsv_high, Scalar _hsv_low);
    void setParameterIntoFile();
    Mat getImageFromCamera(ColorConversionCodes flag = COLOR_BGR2BGRA);
    void setMode(processMode_t _mode);
    void setVecObjects(vector<Filter::Object_t> vec_objects);
    bool getObject(Filter::Object_t& _object, int index);

    // loop processing
    void process();
    void basicProcess(Mat &color_img, vector<Filter::Object_t> &vec_objects);
    void suftProcess(Mat &color_img, vector<Filter::Object_t> &vec_objects);
    void exPort(Mat &image, vector<Filter::Object_t> &vec_objects);
    void clearFilter();

    // base process
    void initBase(Mat rgb_img);
    void setBase();
    Point2f getBaseCenter();


private:

    Mat img; Scalar hsv_high, hsv_low;
    double gamma = 1.0, alpha = 1.0, beta = 0.0;
    double threshbinary = 100;
    processMode_t mode = ModeNull;
    QTimer *_timer_release_buffer;
    Filter contour_filter, suft_filter;
    vector<Filter::Object_t> objects_detected;

    // base process
    static const int MAX_PROCESS_NUM = 20;
    static const int OBSET_RANGE_RGB = 50;
    static const int OBSET_SUFT_ROI = 5;
    int process_num = MAX_PROCESS_NUM;
    Mat base_mask;
    Point2f base_center;
};

#endif // IMAGEPROCESS_H
