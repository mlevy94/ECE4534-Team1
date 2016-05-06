#ifndef OBJECTTOKEN_H
#define OBJECTTOKEN_H

#include <QPainter>
#include <QWidget>

class ObjectToken : public QWidget
{
    Q_OBJECT
public:
    explicit ObjectToken(int sqSize, QPointF pos, QString id, QString type, QWidget *parent = 0)
        : QWidget(parent), m_sqSize(sqSize), m_type(type), m_id(id), m_pos(pos)
    {
        // set size of widget
        this->setFixedSize(m_sqSize + 1,m_sqSize + 1);

        // calc offsets
        m_offs = m_sqSize/2;

        // set color
        if(m_type == "Object") { m_col = sm_objectCol; }
        else if(m_type == "Token") { m_col = sm_tokenCol; }

        // move and show
        this->move(QPoint(m_pos.x() - m_offs, m_pos.y() - m_offs));
        this->show();
    }

    static const Qt::GlobalColor sm_objectCol = Qt::gray;
    static const Qt::GlobalColor sm_tokenCol = Qt::yellow;

signals:

public slots:
    void paintEvent(QPaintEvent *)
    {
        // make a painter
        QPainter painter(this);

        // set up box
        QRect rect(QPoint(0,0),QPoint(m_sqSize,m_sqSize));

        // draw the box
        QColor col(m_col);
        col.setAlphaF(0.5);
        painter.setBrush(QBrush(col));
        painter.drawRect(rect);

        // draw info text
        QFont font("Arial", QFont::Bold);
        font.setPointSizeF(sm_fontRatio * m_sqSize);
        painter.setFont(font);
        painter.drawText(rect,Qt::AlignCenter,m_id);
    }

    QString id() const
    {
        return m_id;
    }

    void setId(const QString &id)
    {
        m_id = id;
    }


    QPointF pos() const
    {
        return m_pos;
    }

    void setPos(const QPointF &pos)
    {
        m_pos = pos;
        this->move(QPoint(m_pos.x() - m_offs, m_pos.y() - m_offs));
    }

private:
    int m_sqSize, m_offs;
    QString m_type;
    QString m_id;
    QPointF m_pos;
    Qt::GlobalColor m_col;
    double sm_fontRatio = 0.21;
};



#endif // OBJECTTOKEN_H


