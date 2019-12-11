#ifndef SIMPLEKALMANFILTER_H
#define SIMPLEKALMANFILTER_H

#include <QObject>
#include <math.h>

class SimpleKalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit SimpleKalmanFilter(float mea_e = 2, float est_e = 2, float q = 1.0/60.0,
                                QObject *parent = nullptr);
    ~SimpleKalmanFilter();
    SimpleKalmanFilter(float mea_e, float est_e, float q);
    float updateEstimate(float mea);
    void setMeasurementError(float mea_e);
    void setEstimateError(float est_e);
    void setProcessNoise(float q);
    float getKalmanGain();
    float getEstimateError();

private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
};

#endif // SIMPLEKALMANFILTER_H
