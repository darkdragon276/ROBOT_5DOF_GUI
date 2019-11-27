#ifndef QLABEL_CUSTOM_H
#define QLABEL_CUSTOM_H

#include <QLabel>
#include <QMouseEvent>
#include <QRubberBand>

#include <iostream>
#include <algorithm>

class QLabel_custom : public QLabel
{
    Q_OBJECT
public:
    explicit QLabel_custom(QWidget *parent = nullptr);
    ~QLabel_custom();
    void mouseMoveEvent(QMouseEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    QRect getRect();
    void getTopLeftPoint(QPoint &tlpoint);
    void getBottomRightPoint(QPoint &brpoint);
    QPoint getLastPoint();
    int x, y;
    QPoint first_point, last_point;
    QPoint topleft_point, bottomright_point;
    int width, heigh;
    QRubberBand *rubber = NULL;

signals:
    void mouseMove();
    void mousePressed();
    void mouseReleased();

public slots:
};

#endif // QLABEL_CUSTOM_H
