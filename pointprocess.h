#ifndef POINTPROCESS_H
#define POINTPROCESS_H
#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/types.hpp"

#include <math.h>
#include <QtCore>
#include <QObject>
#include <QTimer>

#include "debug.h"

using namespace cv;
class PointProcess: public QObject
{
    Q_OBJECT
public:
    explicit PointProcess(QObject *parent=Q_NULLPTR);
    ~PointProcess();

    typedef enum {
        normal,
        reference,
    } Piority_t;

    typedef struct {
        Point2f center;
        double radius_img;
        double angle;
//        vector<Point2f> contour;
    } Object_t;

    static vector<Point2f> toVectorPoint2f(vector<Point> _vec_point);
    static Point2f meansVectorPoints(vector<Point2f> _vec_point);
    static Point2f meansVectorPoints(vector<Point2f> _vec_point, double &radius);
    void hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                int max_group, vector<vector<Point2f>> &group_point);
    void hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                vector<Point2f> ref_list, int max_group,
                                vector<vector<Point2f>> &group_point);
    void hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                int max_group, vector<vector<int> > &pointInGroup_idx);
    void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2);
    double getAngle(vector<Point2f> vec_contour, Mat &color_img);
    void filledPara(vector<Point2f> contour, Object_t &object, Mat &color_image);
    // process method for vector<contour>
    void setVecContour(vector<vector<Point>> _vec_contour, Mat &color_image);
    void cluster();
    bool isReadyGet(vector<Object_t> &_vec_object);

signals:
    void signalCluster();
private:
    // process method for vector<point>
    static const int MAX_NUM_SIZE = 3, MAX_GROUP_NUM = 10, ACURACY_GROUP_SIZE = 40;

    // process method for vector<contour>
    vector<Object_t> objects_raw, objects_cluster;
    int object_per_frame[MAX_NUM_SIZE] = {0};
    bool ready_get_flag = false;
};

#endif // POINTPROCESS_H
