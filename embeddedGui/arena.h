#ifndef ARENA_H
#define ARENA_H

#include <QWidget>
#include <QPainter>
#include <QPolygon>
#include <QPolygonF>
#include <QPen>
#include <QBrush>
#include <QLine>
#include <QPoint>
#include <QDebug>
#include <QList>
#include <QVector>
#include <QLine>

#include "roverindicator.h"
#include "roverghost.h"
#include "objecttoken.h"



class Arena : public QWidget
{
    Q_OBJECT

public:
    Arena(int pixelX, int pixelY, QWidget *parent = 0);
    ~Arena();

    // STATIC MEMBERS
    static const int sm_penWidth = 3;
    static const Qt::GlobalColor sm_penColor = Qt::black;
    static const Qt::GlobalColor sm_backgroundColor = Qt::white;

    static const int sm_numXGridLines = 6;
    static const int sm_numYGridLines = 6;

    static const int sm_inchX = 36;
    static const int sm_inchY = 36;

    // rover indicator presets
    static const double sm_rIndicatorSizeRat;
    static const Qt::GlobalColor sm_leadRoverColor = Qt::red;
    static const Qt::GlobalColor sm_followRoverColor = Qt::green;

    // rover ghost presets
    static const double sm_ghostSizeRatio;

    // object/token presets
    static const double sm_objectTokenSizeRatio;

public slots:
    void paintEvent(QPaintEvent *);

    // public setters
    void setLeadRoverPos(QPointF posInch, int rotation);
    void setFollowRoverPos(QPointF posInch, int rotation);
    void setLeadRoverRot(int rot);
    void setFollowRoverRot(int rot);

    void addObstacle(QString id, QPointF pos);
    void addToken(QString id, QPointF pos);
    void moveObstacle(QString id, QPointF pos);
    void moveToken(QString id, QPointF pos);

    void setTerrainTile(int posX, int posY, Qt::GlobalColor color);
    void clearLeadRoverPath();
    void clearAll();

private:
    int m_sizeX;
    int m_sizeY;
    double m_inchToPixelRatX;
    double m_inchToPixelRatY;
    RoverIndicator *m_leadRoverInd;
    RoverIndicator *m_followRoverInd;
    QList<ObjectToken*> m_objects;
    QList<ObjectToken*> m_tokens;
    QList<QPair<QRect,Qt::GlobalColor> > m_terrainTiles;
    int leadGhostCounter;
    int followGhostCounter;
    int ghostCounterRollover;
    QVector<QLineF> leadRoverPath;
    QPointF prev;
    QPointF curr;


};

#endif // ARENA_H
