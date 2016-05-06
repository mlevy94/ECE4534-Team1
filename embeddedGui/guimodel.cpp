#include "guimodel.h"

GuiModel::GuiModel(QObject *parent) : QObject(parent)
{

    rpiMsgQ = new QList<RpiMsg>;
    lrTotal = 0;
    frTotal = 0;
    coTotal = 0;
    seTotal = 0;

    lrStatus = QString("UNK");
    frStatus = QString("UNK");
    coStatus = QString("UNK");
    seStatus = QString("UNK");

    lrMissing = 0;
    frMissing = 0;
    coMissing = 0;
    seMissing = 0;

    missingTotal = 0;
    messageTotal = 0;

    netstatTimer = new QTimer(this);
    connect(netstatTimer, SIGNAL(timeout()), this, SLOT(timedGuiUpdate()));
    netstatTimer->start(1000);

    debugShowHex = false;

    clearRoverStats();

}

GuiModel::~GuiModel()
{
    delete rpiMsgQ;
}

//////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////
void GuiModel::recieiveMessage(RpiMsg inmsg)
{
    rpiMsgQ->push_back(inmsg);
    doWork();

}

void GuiModel::doWork()
{

        while(!rpiMsgQ->isEmpty()){
            RpiMsg inmsg = rpiMsgQ->takeFirst();
            processRpiMsg(inmsg);
        }

}

void GuiModel::timedGuiUpdate()
{
    this->runStats();

    emit netStatGuiUpdate(lrMissing, frMissing, coMissing, seMissing,
                          lrStatus, frStatus, coStatus, seStatus,
                          lrTotal, frTotal, coTotal, seTotal,
                          missingTotal, messageTotal);
    emit accStatGuiUpdate(leadRoverTravelError,
                          leadRoverTravelErrorMax,
                          leadRoverRotationalError,
                          leadRoverRotationalErrorMax);
    emit runtimeUpdate(runtime, endtime);

    emit updateLeadRoverACC(leadRoverTravelError, leadRoverTravelErrorMax,
                        leadRoverRotationalError, leadRoverRotationalErrorMax);
}

void GuiModel::setInitList(initialList initListIn)
{
    initList.clear();
    initList = initListIn;
    clearSensorAccuracyStats();
    qDebug() << "INITLIST" << initList;
}

//////////////////////////////////////////////////////////////////////////////////////
/// Process Messages
//////////////////////////////////////////////////////////////////////////////////////
void GuiModel::processRpiMsg(RpiMsg &inmsg)
{

    if(inmsg.type != OBJECT_POS && inmsg.type != TOKEN_POS
           && inmsg.type != NET_STAT){
        //qDebug() << "INCOMING MSG TYPE:" <<  QString(inmsg.type).toLatin1().toHex();
    }

    switch(inmsg.type){

    case DEBUG_MSG:
    case OUTPUT_TO_MONITOR:

        printToTextEdit(QString('\n'));

        switch(inmsg.source){

        case CLIENT:
            emit printToTextEdit("CLIENT : ");
            break;
        case LEAD_ROVER:
            emit printToTextEdit("LD ROVR: ");
            break;
        case FOLLOWER:
            emit printToTextEdit("FL ROVR: ");
            break;
        case SENSORS:
            emit printToTextEdit("SENSORS: ");
            break;
        case COORDINATOR:
            emit printToTextEdit("COORDNR: ");
            break;
        case MONITOR:
            emit printToTextEdit("MONITOR: ");
            break;
        case ROUTER:
            emit printToTextEdit("ROUTER : ");
            break;
        default:
            break;
        }

        for(itt = 0 ; itt < inmsg.size ; itt++){

            if(debugShowHex == true){
                emit printToTextEdit(QString(inmsg.msg[itt]).toLatin1().toHex() + QString(" "));
            }

            else{
                emit printToTextEdit(QString(inmsg.msg[itt]));
            }

        }

    break;

    case NET_STAT:
        processNetStat(inmsg);
    break;

    case OBJECT_POS:
        processObjectPos(inmsg);
    break;

    case START_GAME:
        qDebug() << "ERROR: Start game message received from external source";
        break;

    case END_GAME:
        emit endGame(tokensRetrieved);
        emit updateTokenStats(tokensFound, tokensRetrieved);
        break;

    case TOKEN_FOUND:
        qDebug() << "Token Found message received";
        processFoundToken();
        break;

    case ROVER_MOVE:
        qDebug() << "ROVER MOVE MSG RECEIVED";
        qDebug() << QString(inmsg.type).toLatin1().toHex() << QString(inmsg.msg[0]).toLatin1().toHex()
                << QString(inmsg.msg[1]).toLatin1().toHex();
        startLeadRoverMove(inmsg);
        break;

    default:
        qDebug() << "qui model process message arrived at default";
    break;

    }
}

void GuiModel::processNetStat(RpiMsg &inmsg)
{

    ////////////////////////////////////////////////////////////////
    // PROCESS NET STAT
    ////////////////////////////////////////////////////////////////

    QString status = QString("");
    int missing = 0;
    int total = 0;

    switch(inmsg.msg[1]){
    case GREEN:
        status = QString("Green");
        break;
    case YELLOW:
        status = QString("Yellow");
        break;
    case RED:
        status = QString("Red");
        break;
    }

    missing = ( (inmsg.msg[2] << 24) | (inmsg.msg[3] << 16) | (inmsg.msg[4] << 8) | inmsg.msg[5] );
    total = ( (inmsg.msg[6] << 24) | (inmsg.msg[7] << 16) | (inmsg.msg[8] << 8) | inmsg.msg[9] );

    switch(inmsg.msg[0]){

    case MONITOR:
        // do nothing because the monitor does not use wifi to communicate with PI
        emit printToTextEdit(QString("GUI: ERROR: NETSTAT MSG FOR MONITOR IGNORED ***\n"));
        break;

    case LEAD_ROVER:
        lrStatus = status;
        lrMissing = missing;
        lrTotal = total;
     break;

    case FOLLOWER:
        frStatus = status;
        frMissing = missing;
        frTotal = total;
     break;

    case SENSORS:
        seStatus = status;
        seMissing = missing;
        seTotal = total;
     break;

    case COORDINATOR:
        coStatus = status;
        coMissing = missing;
        coTotal = total;
     break;

    default:
        qDebug() << "Something wrong slecting who's netstats we have";
        qDebug() << inmsg.msg[0];
    }

    missingTotal = lrMissing + frMissing + coMissing + seMissing;
    messageTotal = lrTotal + frTotal + coTotal + seTotal;

}

void GuiModel::processObjectPos(RpiMsg &inmsg){

    uint32_t temp = 0;
    float x;
    float y;
    float angle;
    float length;
    float width;
    double error;
    QPointF position;
    QPointF position_converted;
    QPointF distance;
    QString nearest = "ERROR";
    int dist = 5000;
    distance.setX(500);
    distance.setY(500);

    QStringList myOptions;
    myOptions << "T1" << "T2" << "T3" << "T4" << "O1" << "O2" << "O3" << "O4";

    temp = ( (inmsg.msg[1]) << 8 | (inmsg.msg[2])  );
        x = float((float)temp/10);
    temp = ( (inmsg.msg[3]) << 8 | (inmsg.msg[4])  );
        y = float((float)temp/10);
    temp = ( (inmsg.msg[5]) << 8 | (inmsg.msg[6]) );
        angle = ((float)temp);
    temp = ( (inmsg.msg[7]) << 8 | (inmsg.msg[8]) );
        length = float((float)temp/10);
    temp = ( (inmsg.msg[9]) << 8 | (inmsg.msg[10]) );
        width = float((float)temp/10);

    position.setX(x);
    position.setY(y);

    position_converted.setX(x*10);
    position_converted.setY(y*10);


    switch(inmsg.msg[0]){

        case LEAD:
            //qDebug() << "Leader Information Identified";
            emit updateLeadRover(position, angle);
            leadTracker.push_front(position);
            leadRoverPosition.setX(position_converted.x());
            leadRoverPosition.setY(position_converted.y());
            leadRoverAngle = angle;
        break;

        case FOLLOW:
            //qDebug() << "Follower Information Identified";
            followTracker.push_front(position);
        break;

        case OBSTACLE:
                //qDebug() << "Obs Information Identified";

                for (itt = 4 ; itt < 8 ; itt++){

                    if(initList.at(itt).second.x() > 0){

                        distance = initList.at(itt).second - position_converted;

                        if(distance.manhattanLength() < dist){
                            nearest = initList.at(itt).first;
                            dist = distance.manhattanLength();
                        }
                    }
                }

                if (nearest == "ERROR"){
                    // NO OBSTACLES IN THE INIT LIST
                    qDebug() << "ERROR: POSSIBLY NO OBSTACLES IN GUIMODEL INITLIST";
                }

                //qDebug() << "NEAREST OBS" << myOptions.indexOf(nearest);

                switch(myOptions.indexOf(nearest)){

                case O1:
                    if(initList.at(4).second.x() >= 0){
                        //qDebug() << "O1";
                        emit updateObject("O1", position, 0);
                        error = (initList.at(4).second - position_converted).manhattanLength();
                        sensorRunningError += error;
                        o1RunningError += error;
                        o1errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                    }
                    break;
                case O2:
                    if(initList.at(5).second.x() >=0){
                        //qDebug() << "O2";
                        emit updateObject("O2", position, 0);
                        error= (initList.at(5).second - position_converted).manhattanLength();
                        sensorRunningError += error;
                        o2RunningError += error;
                        o2errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                    }
                    break;
                case O3:
                    if(initList.at(6).second.x() >=0){
                        //qDebug() << "O3";
                        emit updateObject("O3", position, 0);
                        error= (initList.at(6).second - position_converted).manhattanLength();
                        sensorRunningError += error;
                        o3RunningError += error;
                        o3errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                    }
                    break;
                case O4:
                    if(initList.at(7).second.x() >=0){
                        //qDebug() << "O4";
                        emit updateObject("O4", position, 0);
                        error= (initList.at(7).second - position_converted).manhattanLength();
                        sensorRunningError += error;
                        o4RunningError += error;
                        o4errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                        break;
                    }
                default:
                    qDebug() << "Something wrong deciding which obs this is";
                    break;

                }

                break;

        case TOKEN:

                //qDebug() << "Token Information Identified";

                for (itt = 0 ; itt < 4 ; itt++){

                    if(initList.at(itt).second.x() > 0){
                        distance = initList.at(itt).second - position_converted;

                         //qDebug() << "TOKEN POSITION TEST" << initList.at(itt) << distance;

                        if(distance.manhattanLength() < dist){
                            nearest = initList.at(itt).first;
                            dist = distance.manhattanLength();
                        }
                    }
                }

                //qDebug() << "NEAREST TOKEN" << myOptions.indexOf(nearest);

                if (nearest == "ERROR"){
                    // NO TOKENS IN THE INIT LIST
                }

                switch(myOptions.indexOf(nearest)){

                case T1:

                    if(initList.at(0).second.x() >= 0){
                        emit updateObject("T1", position, 0);
                        error= (initList.at(0).second - position*10).manhattanLength();
                        sensorRunningError += error;
                        t1RunningError += error;
                        t1errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                    }

                    break;
                case T2:

                    if(initList.at(1).second.x() >= 0){
                        emit updateObject("T2", position, 0);
                        error= (initList.at(1).second - position*10).manhattanLength();
                        sensorRunningError += error;
                        t2RunningError += error;
                        t2errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                    }

                    break;
                case T3:

                    if(initList.at(2).second.x() >=0){
                        emit updateObject("T3", position, 0);
                        error= (initList.at(2).second - position*10).manhattanLength();
                        sensorRunningError += error;
                        t3RunningError += error;
                        t3errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                    }
                    break;
                case T4:
                    if(initList.at(3).second.x() >= 0){
                        emit updateObject("T4", position, 0);
                        error= (initList.at(3).second - position*10).manhattanLength();
                        sensorRunningError += error;
                        t4RunningError += error;
                        t4errorCt++;
                        if(error >= sensorMaxError){
                            sensorMaxError = error;
                        }
                        sensorDivisor++;
                    }
                    break;
                default:
                    qDebug() << "Something wrong deciding which token this is";
                    break;

                }


                break;


        case ROVER_MOVE:
            startLeadRoverMove(inmsg);
            break;


        default:
        break;
    }

    runStats();

}

void GuiModel::processFoundToken()
{
    tokensFound++;
    int itt = 0;
    QPointF distance;
    distance.setX(500);
    distance.setY(500);
    QString nearest = "ERROR";

    int dist = 5000;

    for (itt = 0 ; itt < 4 ; itt++){

        if(initList.at(itt).second.x() >= 0){

            distance = initList.at(itt).second - leadRoverPosition;

            if(distance.manhattanLength() < dist){
                nearest = initList.at(itt).first;
                dist = distance.manhattanLength();
            }

        }

    }

    leadRunningTokenError += dist;
    leadTokenErrorDivisor++;

    runStats();

    emit updateLeadRoverTokenStats(nearest, leadRoverPosition);

}

void GuiModel::startLeadRoverMove(RpiMsg &inmsg)
{




    switch(inmsg.msg[0]){
    case ROVER_FORWARD:
    case ROVER_BACKWARD:
    case ROVER_LEFT:
    case ROVER_RIGHT:
        lastLeadRoverPosition = leadRoverPosition;
        lastLeadRoverAngle = leadRoverAngle;

        leadRoverMovementType= inmsg.msg[0];
        leadRoverTarget = inmsg.msg[1];
        break;
    case ROVER_STOP:
        runRoverStats();
        break;
    default:
        qDebug() << "startLeadRoverMove Failed at default";
        break;
    }

}

void GuiModel::runRoverStats()
{
            qDebug() << "RUNNING ROVER STATS";
            float x1,x2,y1,y2;
            float ang1,ang2;
            float error;

            x2 = lastLeadRoverPosition.x();
            x1 = leadRoverPosition.x();

            qDebug() << "x1,x2" << x1 << x2;

            y2 = lastLeadRoverPosition.y();
            y1 = leadRoverPosition.y();

            ang1 = lastLeadRoverAngle;
            ang2 = leadRoverAngle;

            float dist = sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
            float ang_diff = ang2 - ang1;

            switch( leadRoverMovementType ){

            case ROVER_FORWARD:
            case ROVER_BACKWARD:

                qDebug() << "DIST IS" << dist;

                error = abs(  (double)leadRoverTarget - dist )/10;
                 runningError_LeadRover_travel += error;

                if(error >  leadRoverTravelErrorMax){
                     leadRoverTravelErrorMax = error;
                }

                travel_divisor_LeadRover++;

                break;

            case ROVER_LEFT:
            case ROVER_RIGHT:

                qDebug() << "ANG DIFF IS" << ang_diff;


                if( (ang_diff) < 0){
                    error = abs( (leadRoverTarget) - (360 +  ang_diff ) );
                }

                else if( ((int)ang2 - (int)ang1) >= 0){
                    error = abs( ( leadRoverTarget) - (ang_diff) );
                }

                else {
                    // should never arriver here
                }

                 runningError_LeadRover_angle += error;

                if(error >  leadRoverRotationalErrorMax){
                     leadRoverRotationalErrorMax = error;
                }

                rotational_divisor_LeadRover++;

                break;

            case ROVER_STOP:
                // This is an error. ROVER_STOP should not be stored in the
                // initial movement's id field
                break;
            default:
                break;
            }

            leadRoverTravelError = (100*runningError_LeadRover_travel)/(36*travel_divisor_LeadRover);
            leadRoverRotationalError = (10*runningError_LeadRover_angle)/(36*rotational_divisor_LeadRover);


}

//////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////
void GuiModel::runStats()
{

    double t1error, t2error, t3error, t4error, o1error, o2error, o3error, o4error, sensorError, leadTokenError;


    if (leadTokenErrorDivisor  > 0) { leadTokenError = (100*leadRunningTokenError)/(360*leadTokenErrorDivisor); }
    else{ leadTokenError = -1; }

    if (sensorDivisor - 1 > 0){ sensorError = (100*sensorRunningError)/(360*(sensorDivisor-1)); }
    else { sensorError = -1; }

    if ( t1errorCt > 0) {  t1error = (100*t1RunningError)/(360*t1errorCt); }
    else { t1error = -1; }

    if ( t2errorCt > 0) { t2error = (100*t2RunningError)/(360*t2errorCt); }
    else { t2error = -1; }

    if ( t3errorCt > 0) { t3error = (100*t3RunningError)/(360*t3errorCt); }
    else { t3error = -1; }

    if ( t4errorCt > 0) { t4error = (100*t4RunningError)/(360*t4errorCt); }
    else { t4error = -1; }

    if ( o1errorCt > 0) { o1error = (100*o1RunningError)/(360*o1errorCt); }
    else { o1error = -1; }

    if ( o2errorCt > 0) { o2error = (100*o2RunningError)/(360*o2errorCt); }
    else { o2error = -1; }

    if ( o3errorCt > 0) { o3error = (100*o3RunningError)/(360*o3errorCt); }
    else { o3error = -1; }

    if ( o4errorCt > 0) { o4error = (100*o4RunningError)/(360*o4errorCt); }
    else { o4error = -1; }

    emit updateSensorStats(leadTokenError, sensorError, sensorMaxError);
    emit updateTokenSensorStats(t1error, t2error, t3error, t4error);
    emit updateObstacleSensorStats(o1error, o2error , o3error , o4error);
    emit updateTotalTokens(totalTokens_m);
    emit updateTokenStats(tokensFound, tokensRetrieved);

}

void GuiModel::clearNetStats()
{
    lrTotal = 0;
    frTotal = 0;
    coTotal = 0;
    seTotal = 0;

    lrStatus = QString("UNK");
    frStatus = QString("UNK");
    coStatus = QString("UNK");
    seStatus = QString("UNK");

    lrMissing = 0;
    frMissing = 0;
    coMissing = 0;
    seMissing = 0;

    missingTotal = 0;
    messageTotal = 0;

    emit netStatGuiUpdate(lrMissing, frMissing, coMissing, seMissing,
                          lrStatus, frStatus, coStatus, seStatus,
                          lrTotal, frTotal, coTotal, seTotal,
                          missingTotal, messageTotal);


}

void GuiModel::clearSensorAccuracyStats(){

    totalTokens_m =0;
    totalObstacles=0;

    for (itt = 0 ; itt < 4 ; itt++){


        if( initList.at(itt).first == "T1" ||
                initList.at(itt).first == "T2" ||
                initList.at(itt).first == "T3" ||
                initList.at(itt).first == "T4"){

                if( initList.at(itt).second.x() >= 0){
                    totalTokens_m++;
                }
        }

    }

    for (itt = 4 ; itt < 8 ; itt++){


        if( initList.at(itt).first == "O1" ||
                initList.at(itt).first == "O2" ||
                initList.at(itt).first == "O3" ||
                initList.at(itt).first == "O4"){

                if( initList.at(itt).second.x() >= 0){

                }
        }

    }

    tokensFound = 0;
    tokensRetrieved = 0;
    tokensMissed = 0;

    leadRunningTokenError =0;
    leadTokenErrorDivisor = 0;
    followRunningError =0;
    followDivisor =0;
    sensorRunningError = 0;
    sensorDivisor = 0;
    sensorMaxError = 0;

   t1RunningError=0;
   t2RunningError=0;
   t3RunningError=0;
   t4RunningError=0;

   o1RunningError=0;
   o2RunningError=0;
   o3RunningError=0;
   o4RunningError=0;

   t1errorCt=0;
   t2errorCt=0;
   t3errorCt=0;
   t4errorCt=0;

   o1errorCt=0;
   o2errorCt=0;
   o3errorCt=0;
   o3errorCt=0;

   leadRoverTravelError = 0;
   leadRoverTravelErrorMax = 0;
   leadRoverRotationalError = 0;
   leadRoverRotationalErrorMax= 0;

   this->runStats();

}

void GuiModel::clearRoverStats(){

            lastLeadRoverPosition.setX(-100);
            lastLeadRoverPosition.setY(-100);
            lastLeadRoverAngle = 0;
            leadRoverTarget = 0;
            leadRoverMovementType = 0;


            travel_divisor_LeadRover = 0;
            rotational_divisor_LeadRover = 0;
            runningError_LeadRover_travel = 0;
            runningError_LeadRover_angle = 0;
            leadRoverTravelError = 0;
            leadRoverRotationalError = 0;
            leadRoverTravelErrorMax = 0;
            leadRoverRotationalErrorMax = 0;
}

void GuiModel::startGame(){

    runtime.start();

}

