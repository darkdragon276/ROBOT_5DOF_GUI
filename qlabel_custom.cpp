#include "qlabel_custom.h"

QLabel_custom::QLabel_custom(QWidget *parent) :
    QLabel(parent)
{
    rubber = new QRubberBand(QRubberBand::Rectangle, this);
}

QLabel_custom::~QLabel_custom()
{
    delete rubber;
}

void QLabel_custom::mouseMoveEvent(QMouseEvent *ev)
{
    if(ev->x() < 0) {
        this->x = 0;
    } else if( ev->x() > this->size().width()) {
        this->x = this->size().width();
    } else {
        this->x = ev->x();
    }

    if(ev->y() < 0) {
        this->y = 0;
    } else if( ev->y() > this->size().height()) {
        this->y = this->size().height();
    } else {
        this->y = ev->y();
    }
    rubber->setGeometry(QRect(first_point, ev->pos()).normalized());
    emit mouseMove();
}

void QLabel_custom::mousePressEvent(QMouseEvent *ev)
{
    first_point = ev->pos();
    rubber->setGeometry(QRect(first_point, QSize()));
    rubber->show();
    emit mousePressed();
}

void QLabel_custom::mouseReleaseEvent(QMouseEvent *ev)
{
    last_point = ev->pos();
    rubber->hide();
    topleft_point = QPoint(std::min(first_point.x(), last_point.x()), std::min(first_point.y(), last_point.y()));
    bottomright_point = QPoint(std::max(first_point.x(), last_point.x()), std::max(first_point.y(), last_point.y()));
    emit mouseReleased();
}

QRect QLabel_custom::getRect()
{
    return QRect(topleft_point, bottomright_point);
}

void QLabel_custom::getTopLeftPoint(QPoint &tlpoint)
{
    tlpoint = topleft_point;
}

void QLabel_custom::getBottomRightPoint(QPoint &brpoint)
{
    brpoint = bottomright_point;
}


