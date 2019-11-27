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

    static vector<Point2f> toVectorPoint2f(vector<Point> _vec_point);
    static Point2f meansVectorPoints(vector<Point2f> _vec_point);
    static Point2f meansVectorPoints(vector<Point2f> _vec_point, double &radius);
    void hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                int max_group, vector<vector<Point2f>> &group_point);
    void hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                vector<Point2f> ref_list, int max_group,
                                vector<vector<Point2f>> &group_point);
    void hierarchicalClustering(vector<Point2f> point_list, double max_distance, int max_group,
                                vector<vector<int> > &pointInGroup_idx);
    void setVecPoint(vector<Point2f> _vec_point, Piority_t _level = normal);
    void cluster();
    bool isReadyGet(vector<Point2f> &_vec_center);


signals:
    void signalCluster();
private:
    static const int MAX_NUM_SIZE = 5, MAX_GROUP_NUM = 10, ACURACY_GROUP_SIZE = 40;
    vector<Point2f> vec_point, vec_center;
    int vec_num_point[MAX_NUM_SIZE] = {0};
    bool ready_get_flag = false;
};

#endif // POINTPROCESS_H
