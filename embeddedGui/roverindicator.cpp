#include "roverindicator.h"

double const RoverIndicator::sm_scaleX = 0.3;
double const RoverIndicator::sm_scaleY = 0.6;


// CONSTRUCTOR
RoverIndicator::RoverIndicator(int pixelX, int pixelY, Qt::GlobalColor color, QPointF startPos, int startRot, QWidget *parent)
    : QWidget(parent), m_sizeX(pixelX), m_sizeY(pixelY), m_color(color), m_pos(startPos), m_rotation(startRot)
{
    // set the widget's size
    this->setFixedSize(m_sizeX + sm_pixelPadding, m_sizeY + sm_pixelPadding);

    // calculate move offsets
    offsX = m_sizeX/2;
    offsY = m_sizeY/2;

    // move and show the widget
    this->move(m_pos.x() - offsX, m_pos.y() - offsY);
    this->show();

}

// PAINTER
void RoverIndicator::paintEvent(QPaintEvent *)
{
    // create a painter
    QPainter painter(this);

    // define the triangle polygon
    QPolygon tri;

    tri << QPoint(offsX,0)
        << QPoint(0,m_sizeY)
        << QPoint(m_sizeX,m_sizeY);


    QMatrix rotMat;
    rotMat.translate(offsX,offsY).rotate(m_rotation).scale(sm_scaleX, sm_scaleY).translate(-offsX,-offsY);
    tri = rotMat.map(tri);

    // define a drawing pen
    QPen bPen;
    bPen.setWidth(sm_penWidth);
    bPen.setColor(sm_penColor);

    // fill the triangle
    QBrush bkFillBr;
    bkFillBr.setColor(m_color);
    bkFillBr.setStyle(Qt::SolidPattern);
    QPainterPath bkFillPath;
    bkFillPath.addPolygon(tri);
    painter.fillPath(bkFillPath, bkFillBr);
}

// Setter functions
void RoverIndicator::setPos(const QPointF &pos, int rotation)
{
    m_pos = pos;
    this->update();
    this->move(m_pos.x() - offsX, m_pos.y() - offsY);
    this->setRotation(rotation);
}

void RoverIndicator::setColor(const Qt::GlobalColor &color)
{
    m_color = color;
}

void RoverIndicator::setRotation(int rotation)
{
    m_rotation = rotation;

}

// Getter functions

QPointF RoverIndicator::getPos() const
{
    return m_pos;
}
