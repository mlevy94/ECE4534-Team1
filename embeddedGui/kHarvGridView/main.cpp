#include "arena.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Arena w(387,387);
    w.setTerrainTile(2,2,Qt::green);
    w.setTerrainTile(3,4,Qt::red);
    w.setLeadRoverRot(180);
    w.setFollowRoverRot(45);
    w.addToken("T1", QPointF(3, 3));
    w.addObject("O1", QPointF(10, 10));
    w.addObject("O2", QPointF(35, 50));
    w.moveObject("O2",QPointF(15,35));

    for(int i = 4; i < 30; ++i)
    {
        w.setLeadRoverPos(QPointF(25.5,i*0.25));
    }

    w.setFollowRoverPos(QPointF(25.3,25));
    w.setFollowRoverPos(QPointF(28.2,9));

    w.show();

    return a.exec();
}
