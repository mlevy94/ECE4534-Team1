#ifndef ROVERINDICATOR_H
#define ROVERINDICATOR_H

#include <QWidget>
#include <QPoint>
#include <QMatrix>
#include <QPainter>
#include <QPen>
#include <QDebug>

class RoverIndicator : public QWidget
{
    Q_OBJECT
public:
    explicit RoverIndicator(int pixelX, int pixelY, Qt::GlobalColor color, QPointF startPos, int startRot, QWidget *parent = 0);

    // static members
    static const int sm_penWidth = 3;
    static const Qt::GlobalColor sm_penColor = Qt::black;
    static const int sm_pixelPadding = 3;
    static const double sm_scaleX;
    static const double sm_scaleY;

signals:

public slots:
    void paintEvent(QPaintEvent *);

    void setRotation(int rotation);

    void setColor(const Qt::GlobalColor &color);

    void setPos(const QPointF &pos, int rotation);

    QPointF getPos() const;

private:
    int m_sizeX, m_sizeY, m_rotation;
    Qt::GlobalColor m_color;
    QPointF m_pos;

    int offsX, offsY;


};

#endif // ROVERINDICATOR_H
