#include "pointprocess.h"

PointProcess::PointProcess(QObject *parent)
{
    Q_UNUSED(parent);
    QObject::connect(this, &PointProcess::signalCluster, this, &PointProcess::cluster);
}

PointProcess::~PointProcess()
{

}

vector<Point2f> PointProcess::toVectorPoint2f(vector<Point> _vec_point)
{
    vector<Point2f> _vec_point2f;
    if(_vec_point.empty()) {
        M_DEBUG("vector point input empty");
        return _vec_point2f;
    }
    for(size_t i = 0; i < _vec_point.size(); i++) {
        _vec_point2f.push_back((Point2f)_vec_point.at(i));
    }
    return _vec_point2f;
}

// to caculate center of group point
Point2f PointProcess::meansVectorPoints(vector<Point2f> _vec_point)
{
    if(_vec_point.empty()) {
        M_DEBUG("vector<Point> empty");
        return Point2f();
    }
    Point2f temp(0, 0);
    for(size_t i = 0; i < _vec_point.size(); i ++) {
        temp += _vec_point.at(i);
    }
    return Point2f(temp.x / (double)_vec_point.size(), temp.y / (double)_vec_point.size());
}

// get center and radius of group point
Point2f PointProcess::meansVectorPoints(vector<Point2f> _vec_point, double &radius)
{
    if(_vec_point.empty()) {
        M_DEBUG("vector<Point> empty");
        return Point2f();
    }
    Point2f temp(0, 0);
    double _vec_point_size = (double)_vec_point.size();
    for(size_t i = 0; i < _vec_point.size(); i++) {
        temp += _vec_point.at(i);
    }
    Point2f means(temp.x/_vec_point_size, temp.y/_vec_point_size);

    double _sum = 0;
    for(size_t i = 0; i < _vec_point.size(); i++) {
        _sum += norm(means - _vec_point.at(i));
    }
    radius = _sum/_vec_point_size;
    return means;
}

void PointProcess::hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                          int max_group, vector<vector<Point2f>> &group_point) {
    if(point_list.empty()) {
        M_DEBUG("error empty input");
        return;
    }
    if(max_distance <= 0) {
        M_DEBUG("error distance input");
        return;
    }
    vector<bool> check(point_list.size(), false);
    vector<Point2f> temp;
    for(size_t i = 0; i < point_list.size(); i++) {
        if(check.at(i)) {
            continue;
        } else {
            temp.push_back(point_list.at(i));
            check.at(i) = true;
            for( size_t j = i + 1; j < point_list.size(); j++) {
                if(check.at(j)) {
                    continue;
                } else {
                    if( norm(point_list.at(i) - point_list.at(j)) <= max_distance) {
                        temp.push_back(point_list.at(j));
                        check.at(j) = true;
                    }
                }
            }
            group_point.push_back(temp);
            temp.clear();
            if(group_point.size() >= (size_t)max_group) {
                break;
            }
        }
    }
    Debug::_delete(check, temp);
}

void PointProcess::hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                          vector<Point2f> ref_list, int max_group,
                                          vector<vector<Point2f>> &group_point)
{
    if(point_list.empty()) {
        M_DEBUG("error empty input");
        return;
    }
    if(max_distance <= 0) {
        M_DEBUG("error distance input");
        return;
    }
    vector<bool> check(point_list.size(), false);
    vector<Point2f> temp;

    for(size_t i = 0; i < ref_list.size(); i++) {
        for( size_t j = 0; j < point_list.size(); j++) {
            if(check.at(j)) {
                continue;
            } else {
                if( norm(point_list.at(j) - ref_list.at(i)) <= max_distance) {
                    temp.push_back(point_list.at(j));
                    check.at(j) = true;
                }
            }
        }
        group_point.push_back(temp);
        temp.clear();
        if(group_point.size() >= (size_t)max_group) {
            break;
        }
    }
    Debug::_delete(check, temp);
}

void PointProcess::hierarchicalClustering(vector<Point2f> point_list, double max_distance,
                                          int max_group, vector<vector<int>> &pointInGroup_idx) {
    if(point_list.empty()) {
        M_DEBUG("error empty input");
        return;
    }
    if(max_distance <= 0) {
        M_DEBUG("error distance input");
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
            for( size_t j = i + 1; j < point_list.size(); j++) {
                if(check.at(j)) {
                    continue;
                } else {
                    if( norm(point_list.at(i) - point_list.at(j)) < max_distance) {
                        temp_idx.push_back(j);
                        check.at(j) = true;
                    }
                }
            }
            pointInGroup_idx.push_back(temp_idx);
            temp_idx.clear();
            if(pointInGroup_idx.size() >= (size_t)max_group) {
                break;
            }
        }
    }
    Debug::_delete(check, temp_idx);
}

void PointProcess::filledPara(vector<Point2f> contour, PointProcess::Object_t &object)
{
    if(contour.empty()) {
        M_DEBUG("contour empty");
        return;
    }
    double radius;
    double angle;
    // caculate center, radius and angle of countour point.
    Point2f center = meansVectorPoints(contour, radius);

    // assign to object
    object.center = center;
    object.radius_img = radius;
    object.angle = angle;
}

void PointProcess::setVecContour(vector<vector<Point> > _vec_contour)
{
    ready_get_flag = false;
    // pop front
    vector<Object_t>::iterator last_idx = objects_raw.begin();
    advance(last_idx, object_per_frame[0]);
    objects_raw.erase(objects_raw.begin(), last_idx);

    // push back
    Object_t object_temp;
    vector<Point2f> contour_2f;
    foreach (vector<Point> contour, _vec_contour) {
        contour_2f = toVectorPoint2f(contour);
        filledPara(contour_2f, object_temp);
        objects_raw.push_back(object_temp);
    }

    // set index
    int num_contour = (int)_vec_contour.size();
    for(int i = 0; i < MAX_NUM_SIZE - 1; i++) {
        object_per_frame[i] = object_per_frame[i+1];
    }
    object_per_frame[MAX_NUM_SIZE - 1] = num_contour;

    Debug::_delete(contour_2f);
    emit signalCluster();

}

void PointProcess::cluster()
{
    vector<vector<int>> groups_idx;
    vector<Point2f> list_center;
    foreach (Object_t object, objects_raw) {
        list_center.push_back(object.center);
    }
    hierarchicalClustering(list_center, ACURACY_GROUP_SIZE, MAX_GROUP_NUM, groups_idx);

    objects_cluster.clear();
    foreach (vector<int> group_idx, groups_idx) {
        Object_t object_temp;
        foreach (int idx, group_idx) {
            object_temp.angle += objects_raw.at(idx).angle;
            object_temp.radius_img += objects_raw.at(idx).radius_img;
            object_temp.center += objects_raw.at(idx).center;
        }
        object_temp.angle /= (double)group_idx.size();
        object_temp.radius_img /= (double)group_idx.size();
        object_temp.center = Point2f(object_temp.center.x/(double)group_idx.size(),
                                     object_temp.center.y/(double)group_idx.size());
        objects_cluster.push_back(object_temp);
    }

    ready_get_flag = true;
    Debug::_delete(groups_idx, list_center);

}

bool PointProcess::isReadyGet(vector<Object_t> &_vec_object)
{
    if(ready_get_flag) {
        if(!_vec_object.empty()) {
            _vec_object.clear();
        }
        _vec_object.insert(_vec_object.begin(), objects_cluster.begin(), objects_cluster.end());
    }
    return ready_get_flag;
}
