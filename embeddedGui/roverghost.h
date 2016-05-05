#ifndef ROVERGHOST_H
#define ROVERGHOST_H

#include <QPainter>
#include <QWidget>
#include <QBrush>
#include <QColor>

class RoverGhost : public QWidget
{
    Q_OBJECT
public:
    explicit RoverGhost(int sqSize, QPointF pos, Qt::GlobalColor col, QWidget *parent = 0)
        : QWidget(parent), m_size(sqSize), m_pos(pos), m_col(col)
    {
        // set the size
        this->setFixedSize(m_size,m_size);

        // calc offsets
        m_offs = m_size/2;

        // move and show
        this->move(QPoint(m_pos.x() - m_offs, m_pos.y() - m_offs));
        this->show();
    }

signals:

public slots:
    void paintEvent(QPaintEvent *)
    {
        // create a painter
        QPainter painter(this);

        // paint the circle
        QColor col(m_col);
        col.setAlphaF(0.2);
        painter.setBrush(QBrush(col));
        painter.drawEllipse(QPointF(m_offs,m_offs),m_offs,m_offs);
    }

private:
    int m_size, m_offs;
    QPointF m_pos;
    Qt::GlobalColor m_col;
};

#endif // ROVERGHOST_H

