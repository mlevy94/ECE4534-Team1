#include "arena.h"

double const Arena::sm_rIndicatorSizeRat = 0.2;
double const Arena::sm_ghostSizeRatio = 0.012;
double const Arena::sm_objectTokenSizeRatio = 0.13;

// CONSTRUCTOR
Arena::Arena(int pixelX, int pixelY, QWidget *parent)
    : QWidget(parent), m_sizeX(pixelX), m_sizeY(pixelY)
{
    // set the size of the widget
    this->setFixedSize(m_sizeX + 1,m_sizeY + 1);

    // set inch to pixel ratios
    m_inchToPixelRatX = m_sizeX / sm_inchX;
    m_inchToPixelRatY = m_sizeY / sm_inchY;

    // set up rover indicators
    int rIndSquareSizeX = sm_rIndicatorSizeRat * m_sizeX;
    int rIndSquareSizeY = sm_rIndicatorSizeRat * m_sizeY;

    m_leadRoverInd = new RoverIndicator(rIndSquareSizeX,rIndSquareSizeY, sm_leadRoverColor,QPoint(-100,-100),0,this);
    m_followRoverInd = new RoverIndicator(rIndSquareSizeX,rIndSquareSizeY, sm_followRoverColor,QPoint(-100,-100),0,this);

    leadGhostCounter = 0;
    followGhostCounter = 0;
    ghostCounterRollover = 10;

    leadRoverPath.clear();
    leadRoverPath.clear();
    prev.setX(-100);
    prev.setY(-100);
    curr.setX(-100);
    curr.setY(-100);


}

// DESTRUCTOR
Arena::~Arena()
{

}

// SET LEAD ROVER POS
void Arena::setLeadRoverPos(QPointF posInch, int rotation)
{
    leadGhostCounter++;

   QPointF roverPrevPos = m_leadRoverInd->getPos();
   m_leadRoverInd->setPos(QPointF(posInch.x() * m_inchToPixelRatX, posInch.y() * m_inchToPixelRatY), rotation );

   if(leadGhostCounter == ghostCounterRollover){
       if(roverPrevPos.x() > 0 && roverPrevPos.y() >0){

       }
       leadGhostCounter = 0;
   }

   prev = curr;
   curr = roverPrevPos;

   if(prev.x() > 0 && curr.x() > 0){
       leadRoverPath.push_back(QLineF(prev,curr));
   }

}

// CLEAR LEAD ROVER PATH
void Arena::clearLeadRoverPath(){
    leadRoverPath.clear();
    this->update();
}

// SET FOLLOW ROVER POS
void Arena::setFollowRoverPos(QPointF posInch, int rotation)
{
    followGhostCounter++;
    QPointF roverPrevPos = m_followRoverInd->getPos();
    m_followRoverInd->setPos(QPointF(posInch.x() * m_inchToPixelRatX, posInch.y() * m_inchToPixelRatY), rotation );

    if(followGhostCounter == ghostCounterRollover){
        RoverGhost *newRoverGhost = new RoverGhost(m_sizeX * sm_ghostSizeRatio,roverPrevPos,sm_followRoverColor,this);
        followGhostCounter = 0;
    }
}

// SET FOLLOW ROVER ROT
void Arena::setLeadRoverRot(int rot)
{
    m_leadRoverInd->setRotation(rot);
}

// SET FOLLOW ROVER ROT
void Arena::setFollowRoverRot(int rot)
{
    m_followRoverInd->setRotation(rot);
}

// ADD OBSTACLE
void Arena::addObstacle(QString id, QPointF pos)
{
    ObjectToken *newObstacle = new ObjectToken(sm_objectTokenSizeRatio*m_sizeX,
               QPointF(pos.x()*m_inchToPixelRatX, pos.y()*m_inchToPixelRatY), id, "Object", this);
    m_objects.push_back(newObstacle);
}

// ADD TOKEN
void Arena::addToken(QString id, QPointF pos)
{
    ObjectToken *newToken = new ObjectToken(sm_objectTokenSizeRatio*m_sizeX,
             QPointF(pos.x()*m_inchToPixelRatX, pos.y()*m_inchToPixelRatY), id, "Token", this);
    m_objects.push_back(newToken);
}

// MOVE OBSTACLE
void Arena::moveObstacle(QString id, QPointF pos)
{
    size_t listSize = m_objects.size();

    for(int i = 0; i < listSize; ++i)
    {
        if(m_objects.at(i)->id() == id)
        {
            m_objects.at(i)->setPos(QPointF(pos.x()*m_inchToPixelRatX, pos.y()*m_inchToPixelRatY));
        }
    }
}

// MOVE TOKEN
void Arena::moveToken(QString id, QPointF pos)
{
    size_t listSize = m_tokens.size();

    for(int i = 0; i < listSize; ++i)
    {
        if(m_tokens.at(i)->id() == id)
        {
            m_tokens.at(i)->setPos(QPointF(pos.x()*m_inchToPixelRatX, pos.y()*m_inchToPixelRatY));
        }
    }
}

// SET TERRAIN TILE
void Arena::setTerrainTile(int posX, int posY, Qt::GlobalColor color)
{
    int xInterval = m_sizeX / sm_numXGridLines;
    int yInterval = m_sizeY / sm_numYGridLines;

    QPoint topLeft(posX*xInterval, posY*yInterval);
    QPoint bottomRight(topLeft.x() + xInterval, topLeft.y() + yInterval);

    QRect newTerrainTile(topLeft,bottomRight);
    QPair<QRect,Qt::GlobalColor> newPair(newTerrainTile,color);
    m_terrainTiles.push_back(newPair);
}

// CLEAR ALL
void Arena::clearAll()
{
    m_terrainTiles.clear();
    m_objects.clear();
    leadRoverPath.clear();
}

// PAINT EVENT
void Arena::paintEvent(QPaintEvent *)
{
    // create a painter
    QPainter painter(this);

    // define the boundary
    QPolygon boundary;

    boundary << QPoint(0,0)
               << QPoint(m_sizeX, 0)
               << QPoint(m_sizeX, m_sizeY)
               << QPoint(0, m_sizeY);

    // define a drawing pen
    QPen bPen;
    bPen.setWidth(sm_penWidth);    
    bPen.setColor(sm_penColor);

    QPen roverPathPen;
    roverPathPen.setWidth(2);
    roverPathPen.setColor(Qt::red);

    // fill the rectangle
    QBrush bkFillBr;
    bkFillBr.setColor(sm_backgroundColor);
    bkFillBr.setStyle(Qt::SolidPattern);
    QPainterPath bkFillPath;
    bkFillPath.addPolygon(boundary);
    painter.fillPath(bkFillPath, bkFillBr);

    // draw the terrain tiles
    size_t listSize = m_terrainTiles.size();
    for(int i = 0; i < listSize; ++i)
    {
        QBrush fillBr;
        fillBr.setColor(m_terrainTiles.at(i).second);
        fillBr.setStyle(Qt::Dense5Pattern);
        QPainterPath fillPath;
        fillPath.addRect(m_terrainTiles.at(i).first);
        painter.fillPath(fillPath,fillBr);
    }

    // draw bounding rectangle
    painter.setPen(bPen);
    painter.drawPolygon(boundary);

    // draw the vertical grid lines
    QPoint linePTop(0,0);
    QPoint linePBottom(0,m_sizeY);

    for(int i = 0; i < sm_numXGridLines - 1; ++i)
    {
        float offset = m_sizeX / sm_numXGridLines;
        linePTop.setX(linePTop.x() + offset);
        linePBottom.setX(linePBottom.x() + offset);

        painter.drawLine(linePTop,linePBottom);
    }

    // draw the horizontal grid lines
    QPoint linePLeft(0,0);
    QPoint linePRight(m_sizeX,0);

    for(int i = 0; i < sm_numYGridLines - 1; ++i)
    {
        float offset = m_sizeY / sm_numYGridLines;
        linePLeft.setY(linePLeft.y() + offset);
        linePRight.setY(linePRight.y() + offset);
        painter.drawLine(linePLeft,linePRight);
    }

    // draw the rover path
    painter.setPen(roverPathPen);
    painter.drawLines(leadRoverPath);
}
