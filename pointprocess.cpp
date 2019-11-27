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

void PointProcess::setVecPoint(vector<Point2f> _vec_point, PointProcess::Piority_t _level)
{
    Q_UNUSED(_level);
    ready_get_flag = false;
    // push to vec_point
    // remove first
    vector<Point2f>::iterator last_idx = vec_point.begin();
    advance(last_idx, vec_num_point[0]);
    vec_point.erase(vec_point.begin(), last_idx);
    // push last
    vec_point.insert(vec_point.end(), _vec_point.begin(), _vec_point.end());
    // push to vec_num_point
    int _size_vec_point = (int)_vec_point.size();
    for(int i = 0; i < MAX_NUM_SIZE - 1; i++) {
        vec_num_point[i] = vec_num_point[i+1];
    }
    vec_num_point[2] = _size_vec_point;
    emit signalCluster();
}

void PointProcess::cluster()
{
    vector<vector<Point2f>> _group_point;
    vec_center.clear();
    hierarchicalClustering(vec_point, ACURACY_GROUP_SIZE, MAX_GROUP_NUM, _group_point);
    for(size_t i = 0; i < _group_point.size(); i++) {
        vec_center.push_back(meansVectorPoints(_group_point.at(i)));
    }
    ready_get_flag = true;
    Debug::_delete(_group_point);
}

bool PointProcess::isReadyGet(vector<Point2f> &_vec_center)
{
    if(ready_get_flag) {
        if(!_vec_center.empty()) {
            _vec_center.clear();
        }
        _vec_center.insert(_vec_center.begin(), vec_center.begin(), vec_center.end());
    }
    return ready_get_flag;
}
