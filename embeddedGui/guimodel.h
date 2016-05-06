#ifndef GUIMODEL_H
#define GUIMODEL_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <iostream>
#include <QList>
#include <QPointF>
#include <QMetaType>
#include <QTimer>
#include <QFile>
#include <QTime>
#include <QElapsedTimer>

#include "comm_k.h"

Q_DECLARE_METATYPE(QList<RpiMsg>)

union RoverStat{double rovStat; uint8_t in[8]; };

typedef struct
{
    uint8_t id;
    uint8_t reserved;
    uint8_t xcordMsb;
    uint8_t xcordLsb;
    uint8_t ycordMsb;
    uint8_t ycordLsb;
    uint8_t angleMsb;
    uint8_t angleLsb;
    unsigned int numSamples;

}ROVER_DATA;



class GuiModel : public QObject
{
    Q_OBJECT
public:
    explicit GuiModel(QObject *parent = 0);
    ~GuiModel();

signals:

    // signal to send string to screen
    void printToTextEdit(QString out);
    void printToTextEdit_NL(QString out);

    void netStatGuiUpdate(int lrMissing, int frMissing, int coMissing, int seMissing,
                          QString lrStatus, QString frStatus, QString coStatus, QString seStatus,
                          int lrTotal, int frTotal, int coTotal, int seTotal,
                          int missingTotal, int messageTotal);
    void netStatGuiUpdate2(int lrYct, int lrRct, int frYct, int frRct,
                           int coYct, int coRct, int seYct, int seRct);
    void updateLeadRoverACC(double leadRoverTravelError, double leadRoverTravelErrorMax,
                            double leadRoverRotationalError, double leadRoverRotationalErrorMax);

    void accStatGuiUpdate(double leadRoverTravelError, double leadRoverTravelErrorMax, double leadRoverRotationalError, double leadRoverRotationalErrorMax);
    void updateLeadRover(QPointF inPoint, int inRot);
    void updateFollower(QPointF inPoint, int inRot);
    void updateObject(QString obj, QPointF inPoint, int inRot);
    void updateLeadRoverTokenStats(QString nearest, QPointF leadRoverPosition);
    void updateTokenStats(int tokensFound, int tokensRetreived);
    void updateSensorStats(double lrAcc, double sensorAcc, double sensorMax);
    void updateTokenSensorStats(double t1RunningError,double  t2RunningError,double  t3RunningError,double  t4RunningError);
    void updateObstacleSensorStats(double o1RunningError,double  o2RunningError,double  o3RunningError,double  o4RunningError);
    void updateTotalTokens(int updateTotalTokens);
    void runtimeUpdate(QElapsedTimer runtime, QTime difference);
    void endGame(int tokensRetrieved);

public slots:

    void recieiveMessage(RpiMsg inMsg);
    void clearNetStats();
    void clearSensorAccuracyStats();
    void clearRoverStats();
    void doWork();
    void timedGuiUpdate();
    void setInitList(initialList initListIn);
    void startGame();
    void processFoundToken();


private slots:

    void processRpiMsg(RpiMsg &inmsg);
    void processNetStat(RpiMsg &inmsg);
    void processObjectPos(RpiMsg &inmsg);
    void runStats();
    void startLeadRoverMove(RpiMsg &inmsg);
    void runRoverStats();



private:

    QList<RpiMsg> * rpiMsgQ;
    int itt;

    int monitorTotal;
    int lrTotal;
    int frTotal;
    int coTotal;
    int seTotal;

    QString lrStatus;
    QString frStatus;
    QString coStatus;
    QString seStatus;

    int lrMissing;
    int frMissing;
    int coMissing;
    int seMissing;

    int missingTotal;
    int messageTotal;

    QList<QPointF> leadTracker;
    QList<QPointF> followTracker;
    QList<QPair<QString, QPointF> > initList;

    QTimer *netstatTimer;

    int maxSensorError;

    int totalObstacles;
    int totalTokens_m;
    int tokensFound;
    int tokensRetrieved;
    int tokensMissed;

    QElapsedTimer runtime;
    QTime endtime;

    QPointF leadRoverPosition;
    float leadRoverAngle;
    QPointF followerPosition;

    double leadRunningTokenError;
    int leadTokenErrorDivisor;

    double followRunningError;
    int followDivisor;

    double sensorRunningError;
    int sensorDivisor;
    double sensorMaxError;

    double t1RunningError;
    double t2RunningError;
    double t3RunningError;
    double t4RunningError;

    int t1errorCt;
    int t2errorCt;
    int t3errorCt;
    int t4errorCt;

    double o1RunningError;
    double o2RunningError;
    double o3RunningError;
    double o4RunningError;

    int o1errorCt;
    int o2errorCt;
    int o3errorCt;
    int o4errorCt;

    float leadRoverTravelError;
    float leadRoverTravelErrorMax;
    float leadRoverRotationalError;
    float leadRoverRotationalErrorMax;
    float runningError_LeadRover_travel;
    float runningError_LeadRover_angle;

    QPointF lastLeadRoverPosition;
    float lastLeadRoverAngle;
    uint8_t leadRoverTarget;
    uint8_t leadRoverMovementType;

    unsigned int travel_divisor_LeadRover;
    unsigned int rotational_divisor_LeadRover;

    bool debugShowHex;

};

#endif // GUIMODEL_H
