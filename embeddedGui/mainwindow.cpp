#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    qRegisterMetaType<RpiMsg>("RpiMsg");
    qRegisterMetaType<initialList>("initialList");

    arenaFile_m = "/home/pi/ece4534/test1.txt";
    initTestCase();

    /////////////////////////////////////////////////////////////////////////////
    // Declare workers and threads
    /////////////////////////////////////////////////////////////////////////////
    serial_thread_m = new QThread();
    serial_worker_m = new Serial_worker;
    guiModel_thread_m = new QThread();
    gui_model_m = new GuiModel;


    /////////////////////////////////////////////////////////////////////////////
    // Serial Worker
    /////////////////////////////////////////////////////////////////////////////
    serial_worker_m->moveToThread(serial_thread_m);

    // Signals and Slots
    connect(serial_thread_m,          SIGNAL(finished()),
            serial_worker_m,          SLOT(deleteLater()));

    connect(serial_worker_m,          SIGNAL(printToTextEdit(QString)),
            this,                     SLOT(writeToTextEdit(QString)));

    connect(serial_worker_m,          SIGNAL(printToTextEdit_NL(QString)),
            this,                     SLOT(writeToTextEdit_NL(QString)));

    connect(this,                     SIGNAL(operate()),
            serial_worker_m,          SLOT(doWork()));

    connect(this,                     SIGNAL(writeSerialToWorker(QByteArray, QByteArray)),
            serial_worker_m,          SLOT(writeSerial(const QByteArray, const QByteArray)));

    connect(this,                     SIGNAL(sendInitList(initialList)),
            serial_worker_m,          SLOT(setInitList(initialList)), Qt::QueuedConnection);

    connect(serial_worker_m,          SIGNAL(sendMsgtoModel(RpiMsg)),
            gui_model_m,              SLOT(recieiveMessage(RpiMsg)), Qt::QueuedConnection);

    serial_thread_m->start();

    /////////////////////////////////////////////////////////////////////////////
    // GUI Model
    /////////////////////////////////////////////////////////////////////////////
    gui_model_m->moveToThread(guiModel_thread_m);

    // Signals and Slots
    connect(guiModel_thread_m,    SIGNAL(finished()),
            gui_model_m,          SLOT(deleteLater()));

    connect(gui_model_m,        SIGNAL(printToTextEdit(QString)),
            this,               SLOT(writeToTextEdit(QString)));

    connect(gui_model_m,        SIGNAL(printToTextEdit_NL(QString)),
            this,               SLOT(writeToTextEdit_NL(QString)));

    connect(this,               SIGNAL(operate()),
            gui_model_m,        SLOT(doWork()));

    connect(this,               SIGNAL(clearNetStats()),
            gui_model_m,        SLOT(clearNetStats()));

    connect(gui_model_m,        SIGNAL(netStatGuiUpdate(int,int,int,int,QString,QString,QString,QString,int,int,int,int,int, int)),
            this,               SLOT(receiveNetStatGuiUpdate(int,int,int,int,QString,QString,QString,QString,int,int,int,int,int, int)));

    connect(gui_model_m,        SIGNAL(updateLeadRover(QPointF,int)),
            this,               SLOT(updateLeadRover(QPointF,int)));

    connect(gui_model_m,        SIGNAL(updateFollower(QPointF,int)),
            this,               SLOT(updateFollower(QPointF,int)));

    connect(gui_model_m,        SIGNAL(updateObject(QString,QPointF,int)),
            this,               SLOT(updateObject(QString,QPointF,int)));

    connect(this,               SIGNAL(startGameToModel()),
            gui_model_m,        SLOT(startGame()));

    connect(gui_model_m,        SIGNAL(updateTokenStats(int, int)),
            this,               SLOT(updateTokenStats(int, int)));

    connect(this,               SIGNAL(sendInitList(initialList)),
            gui_model_m,        SLOT(setInitList(initialList)), Qt::QueuedConnection);

    connect(gui_model_m,        SIGNAL(updateLeadRoverTokenStats(QString,QPointF)),
            this,               SLOT(updateLeadRoverTokenStats(QString,QPointF)));

    connect(gui_model_m,        SIGNAL(updateSensorStats(double,double,double)),
            this,               SLOT(updateSensorStats(double,double,double)));

    connect(gui_model_m,        SIGNAL(updateTokenSensorStats(double,double,double,double)),
            this,               SLOT(tokenSensorStats(double,double,double,double)));

    connect(gui_model_m,        SIGNAL(updateObstacleSensorStats(double,double,double,double)),
            this,               SLOT(obstacleSensorStats(double,double,double,double)));

    connect(this,               SIGNAL(clearAccStats()),
            gui_model_m,        SLOT(clearSensorAccuracyStats()));

    connect(this,               SIGNAL(clearAccStats()),
            this,               SLOT(clearLeadRoverTokenStats()));

    connect(this,               SIGNAL(clearAccStats()),
            gui_model_m,        SLOT(clearRoverStats()));


    connect(gui_model_m,        SIGNAL(updateTotalTokens(int)),
            this,               SLOT(totalTokens(int)));

    connect(this,               SIGNAL(startGameToModel()),
            gui_model_m,        SLOT(startGame()));

    connect(gui_model_m,        SIGNAL(runtimeUpdate(QElapsedTimer,QTime)),
            this,               SLOT(updateRuntime(QElapsedTimer, QTime)));

    connect(gui_model_m,        SIGNAL(endGame(int)),
            this,               SLOT(endGame(int)));

    connect(gui_model_m,        SIGNAL(updateLeadRoverACC(double,double,double,double)),
            this,               SLOT(updateRoverAcc(double,double,double,double)));

    guiModel_thread_m->start();

    emit sendInitList(initList_m);

    operate();

    receiveNetStatGuiUpdate(0, 0, 0, 0,
                            QString("UNK"), QString("UNK"), QString("UNK"), QString("UNK"),
                            0, 0, 0, 0,
                            0, 0);


}

MainWindow::~MainWindow()
{


    serial_thread_m->quit();
    serial_thread_m->wait();

    guiModel_thread_m->quit();
    guiModel_thread_m->wait();


    while(!guiModel_thread_m->isFinished());
    while(!serial_thread_m->isFinished());

    delete serial_worker_m;
    delete gui_model_m;

    delete ui;
}

void MainWindow::writeToTextEdit(QString in)
{
    ui->textEdit->moveCursor(QTextCursor::End);
    ui->textEdit->insertPlainText(in);

}

void MainWindow::writeToTextEdit_NL(QString in)
{
    ui->textEdit->append(in);
}

void MainWindow::receiveNetStatGuiUpdate(int lrMissing, int frMissing,
                                         int coMissing, int seMissing,
                                         QString lrStatus, QString frStatus,
                                         QString coStatus, QString seStatus,
                                         int lrTotal, int frTotal,
                                         int coTotal, int seTotal,
                                         int missingTotal, int messageTotal)
{
    ui->lead_net_missing->setText(QString::number(lrMissing));
    ui->follower_net__missing->setText(QString::number(frMissing));
    ui->cord_net_missing->setText(QString::number(coMissing));
    ui->sensors_net_missing->setText(QString::number(seMissing));

    ui->lead_net_status->setText(lrStatus);
    ui->follower_net_status->setText(frStatus);
    ui->cord_net_status->setText(coStatus);
    ui->sensors_net_status->setText(seStatus);

    ui->lead_net_total->setText(QString::number(lrTotal));
    ui->follower_net__total->setText(QString::number(frTotal));
    ui->cord_net_total->setText(QString::number(coTotal));
    ui->sensors_net_total->setText(QString::number(seTotal));

    ui->total_net_total->setText(QString::number(messageTotal));
    ui->total_net_missing->setText(QString::number(missingTotal));

}

void MainWindow::clearRoverAccStats()
{
    ui->LEAD_TRAV_ACC->clear();
    ui->LEAD_TRAV_ACC_MAX->clear();
    ui->LEAD_ROT_ACC->clear();
    ui->LEAD_ROT_ACC_MAX->clear();

}

void MainWindow::updateRoverAcc(double leadRoverTravelError, double leadRoverTravelErrorMax, double leadRoverRotationalError, double leadRoverRotationalErrorMax)
{
    ui->LEAD_TRAV_ACC->setText(QString::number(leadRoverTravelError, 'f', 2));
    ui->LEAD_TRAV_ACC_MAX->setText(QString::number(leadRoverTravelErrorMax, 'f', 2));
    ui->LEAD_ROT_ACC->setText(QString::number(leadRoverRotationalError, 'f', 2));
    ui->LEAD_ROT_ACC_MAX->setText(QString::number(leadRoverRotationalErrorMax, 'f', 2));
}

void MainWindow::updateLeadRover(QPointF inPoint, int inRot)
{
    arena_m->setLeadRoverPos(inPoint, inRot);
    ui->LRX->setText(QString::number( inPoint.x(), 'f', 1 ) );
    ui->LRY->setText(QString::number( inPoint.y(), 'f', 1 ) );
    ui->LRO->setText(QString::number(inRot, 'f', 1) );

}

void MainWindow::updateObject(QString obj, QPointF inPoint, int inRot)
{
    //qDebug() << "UPDATING OBJECT" << obj << inPoint;

    int xcord;
    int ycord;
    QPointF modified;
    QStringList myOptions;
    myOptions << "T1" << "T2" << "T3" << "T4" << "O1" << "O2" << "O3" << "O4";
    xcord = (inPoint.x()-30)/60;
    ycord = (inPoint.y()-30)/60;

    modified.setX(xcord);
    modified.setY(ycord);

    arena_m->moveObstacle(obj,inPoint);

    switch(myOptions.indexOf(obj)){

    case T4:
        ui->T4_LOC_S->setText( QString::number(inPoint.x(), 'f', 1)  + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;
    case T3:
        ui->T3_LOC_S->setText( QString::number(inPoint.x(), 'f', 1) + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;
    case T2:
        ui->T2_LOC_S->setText( QString::number(inPoint.x(), 'f', 1) + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;
    case T1:
        ui->T1_LOC_S->setText( QString::number(inPoint.x(), 'f', 1) + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;
    case O1:
        ui->O1_LOC_S->setText( QString::number(inPoint.x(), 'f', 1) + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;
    case O2:
        ui->O2_LOC_S->setText( QString::number(inPoint.x(), 'f', 1) + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;
    case O3:
        ui->O3_LOC_S->setText( QString::number(inPoint.x(), 'f', 1) + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;
    case O4:
        ui->O4_LOC_S->setText( QString::number(inPoint.x(), 'f', 1) + QString(", ") + QString::number(inPoint.y(), 'f', 1)  );

        break;

    }


}

void MainWindow::updateFollower(QPointF inPoint, int inRot)
{
    arena_m->setFollowRoverPos(inPoint, inRot);
    ui->FRX->setText(QString::number( inPoint.x() ) );
    ui->FRY->setText(QString::number( inPoint.y() ) );
    ui->FRO->setText(QString::number(inRot));

}

void MainWindow::updateLeadRoverTokenStats(QString nearest, QPointF leadRoverPosition)
{
    qDebug() << "Nearest" << nearest;
    QStringList myOptions;
    myOptions << "T1" << "T2" << "T3" << "T4";


    switch(myOptions.indexOf(nearest)){

    case T4:
        ui->T4_LOC_L->setText( QString::number(leadRoverPosition.x()) + QString(", ") + QString::number(leadRoverPosition.y())  );

        break;
    case T3:
        ui->T3_LOC_L->setText( QString::number(leadRoverPosition.x()) + QString(", ") + QString::number(leadRoverPosition.y())  );

        break;
    case T2:
        ui->T2_LOC_L->setText( QString::number(leadRoverPosition.x()) + QString(", ") + QString::number(leadRoverPosition.y())  );

        break;
    case T1:
        ui->T1_LOC_L->setText( QString::number(leadRoverPosition.x()) + QString(", ") + QString::number(leadRoverPosition.y())  );

        break;
    }
}

void MainWindow::clearLeadRoverTokenStats(){
    ui->T4_LOC_L->clear();
    ui->T3_LOC_L->clear();
    ui->T2_LOC_L->clear();
    ui->T1_LOC_L->clear();
    ui->lrTokenAcc->clear();
}

void MainWindow::startGame()
{

  qDebug() << "STARTGAME";
  ui->tokens_found->setText(QString("0"));
  ui->tokens_missed->setText(QString("0"));
  ui->tokens_retrieved->setText(QString("0"));

  QByteArray type;
  QByteArray out;

  type = "72";                        // Start
  out =  arenaNum_m + "AAAAAAAAAAAAAAAAAAAAAA";   // Filler Data

  emit startGameToModel();
  emit writeSerialToWorker(type,out);

}

void MainWindow::endGame(int tokensRetreived){
    ui->tokens_missed->setText(QString::number(totalTokens_m-tokensRetreived));
}

void MainWindow::updateTokenStats(int tokensFound, int tokensRetreived)
{
  ui->tokens_found->setText(QString::number(tokensFound));
  ui->tokens_retrieved->setText(QString::number(tokensRetreived));

}

void MainWindow::updateRuntime(QElapsedTimer runtime, QTime othertime)
{
   // ui->runtime->setText(runtime.toString("hh:mm:ss"));
}

void MainWindow::updateSensorStats(double lrAcc, double sensorAcc, double sensorMax)
{
    if (lrAcc > 0) { ui->lrTokenAcc->setText(QString::number(lrAcc)); }
    if (sensorMax > 0) { ui->sensorMaxError->setText(QString::number(sensorMax)); }
    if (sensorAcc >0) { ui->sensorAveError->setText(QString::number(sensorAcc)); }

}

void MainWindow::tokenSensorStats(double t1error, double t2error, double t3error, double t4error)
{
    if (t1error >0) { ui->T1_LOC_SErr->setText(QString::number(t1error, 'f', 2)); }
    if (t2error >0) { ui->T2_LOC_SErr->setText(QString::number(t2error, 'f', 2)); }
    if (t3error >0) { ui->T3_LOC_SErr->setText(QString::number(t3error, 'f', 2)); }
    if (t4error >0) { ui->T4_LOC_SErr->setText(QString::number(t4error, 'f', 2)); }
}

void MainWindow::obstacleSensorStats(double o1error, double o2error, double o3error, double o4error)
{
    if (o1error >0) { ui->O1_LOC_SErr->setText(QString::number(o1error, 'f', 2)); }
    if (o2error >0) { ui->O2_LOC_SErr->setText(QString::number(o2error, 'f', 2)); }
    if (o3error >0) { ui->O3_LOC_SErr->setText(QString::number(o3error, 'f', 2)); }
    if (o4error >0) { ui->O4_LOC_SErr->setText(QString::number(o4error, 'f', 2)); }
}

void MainWindow::initTestCase()
{
    /////////////////////////////////////////////////////////////////////////////
    // Read in default data
    /////////////////////////////////////////////////////////////////////////////
    QFile infile(arenaFile_m);
    if(!infile.open(QIODevice::ReadOnly | QIODevice::Text)){
        qDebug() <<"Failed to open file";
        return;
    }

    initList_m.clear();

    while(!infile.atEnd()){
        QString line = infile.readLine();
        process_initData(line);
    }


    /////////////////////////////////////////////////////////////////////////////
    // Arena
    /////////////////////////////////////////////////////////////////////////////
    arena_m = new Arena(360,360,ui->tab_3);
    arena_m->move(10,10);
    arena_m->show();
    arena_m->setLeadRoverPos(QPointF(-120,-120), 180);

    // Set obstacles color = RED
    // Set tokens color = Green

    QStringList myOptions;
    myOptions << "T4" << "T3" << "T2" << "T1" << "O4" << "O3" << "O2" << "O1";

    QPointF tempPointF;
    int xcord;
    int ycord;
    int itt;

    ui->T4_LOC_K->clear();
    ui->T3_LOC_K->clear();
    ui->T2_LOC_K->clear();
    ui->T1_LOC_K->clear();

    ui->O4_LOC_K->clear();
    ui->O3_LOC_K->clear();
    ui->O2_LOC_K->clear();
    ui->O1_LOC_K->clear();

    ui->T4_LOC_S->clear();
    ui->T3_LOC_S->clear();
    ui->T2_LOC_S->clear();
    ui->T1_LOC_S->clear();

    ui->O4_LOC_S->clear();
    ui->O3_LOC_S->clear();
    ui->O2_LOC_S->clear();
    ui->O1_LOC_S->clear();

    ui->T4_LOC_SErr->clear();
    ui->T3_LOC_SErr->clear();
    ui->T2_LOC_SErr->clear();
    ui->T1_LOC_SErr->clear();

    ui->O4_LOC_SErr->clear();
    ui->O3_LOC_SErr->clear();
    ui->O2_LOC_SErr->clear();
    ui->O1_LOC_SErr->clear();

    ui->T4_LOC_L->clear();
    ui->T3_LOC_L->clear();
    ui->T2_LOC_L->clear();
    ui->T1_LOC_L->clear();



    // Set known obs and tokens
    for (itt = 0 ; itt < initList_m.size() ; itt++){

        tempPointF = initList_m.at(itt).second;
        xcord = (tempPointF.x()-30)/60;
        ycord = (tempPointF.y()-30)/60;


        switch(myOptions.indexOf(initList_m.at(itt).first)){

        case 0:
            if (tempPointF.x() > 0){
                ui->T4_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::yellow);
            }
            break;
        case 1:
            if (tempPointF.x() > 0){
                ui->T3_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::yellow);
            }
            break;
        case 2:
            if (tempPointF.x() > 0){
                ui->T2_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::yellow);
            }
            break;
        case 3:
            if (tempPointF.x() > 0){
                ui->T1_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::yellow);
            }
            break;
        case 4:
            if (tempPointF.x() > 0){
                ui->O4_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::green);
            }
            break;
        case 5:
            if (tempPointF.x() > 0){
                ui->O3_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::green);
            }
            break;
        case 6:
            if (tempPointF.x() > 0){
                ui->O2_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::green);
            }
            break;
        case 7:
            if (tempPointF.x() > 0){
                ui->O1_LOC_K->setText( QString::number(tempPointF.x()/10, 'f', 1) + QString(", ") + QString::number(tempPointF.y()/10, 'f', 1)  );
                arena_m->setTerrainTile(xcord, ycord, Qt::GlobalColor::green);
            }
            break;

        }
    }

    QPointF offgrid;
    offgrid.setX(120);
    offgrid.setY(120);

    QPointF offgrid2;
    offgrid2.setX(120);
    offgrid2.setY(120);

    // Add indicators for sensor reported obs and tok
    arena_m->addObstacle("O1", offgrid);
    arena_m->addObstacle("O2", offgrid );
    arena_m->addObstacle("O3", offgrid );
    arena_m->addObstacle("O4", offgrid );

    arena_m->addToken("T1", offgrid2);
    arena_m->addToken("T2", offgrid2);
    arena_m->addToken("T3", offgrid2);
    arena_m->addToken("T4", offgrid2);

    emit sendInitList(initList_m);
}

void MainWindow::totalTokens(int totalTokens)
{
    totalTokens_m = totalTokens;
    ui->total_tokens->setText(QString::number(totalTokens_m));
}

void MainWindow::process_initData(QString line)
{

    QPointF point;
    QPair<QString, QPointF> myPair;
    QString name;

    QList<QString> list = line.split(',');

    name = list.at(0);
    point.setX(list.at(1).toDouble());
    point.setY(list.at(2).toDouble());

    qDebug() << "Name: " << name << "PointX: " << point.x() << "PointY: " << point.y();

    myPair.first = name;
    myPair.second = point;

    initList_m.push_front(myPair);

}

///////////////////////////////////////////////////////////////////////////////
/// COMBO BOXES
///////////////////////////////////////////////////////////////////////////////
void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{
   if(arg1 == "Test 1"){
       arenaFile_m = "/home/pi/ece4534/test1.txt";
       arenaNum_m = "01";
   }

   if(arg1 == "Test 2"){
       arenaFile_m = "/home/pi/ece4534/test2.txt";
       arenaNum_m = "02";
   }

   if(arg1 == "Test 3"){
       arenaFile_m = "/home/pi/ece4534/test3.txt";
       arenaNum_m = "03";
   }

   if(arg1 == "Test 4"){
       arenaFile_m = "/home/pi/ece4534/test4.txt";
       arenaNum_m = "04";
   }

   if(arg1 == "Test 5"){
       arenaFile_m = "/home/pi/ece4534/test5.txt";
       arenaNum_m = "05";
   }

   if(arg1 == "Test 6"){
       arenaFile_m = "/home/pi/ece4534/test6.txt";
       arenaNum_m = "06";
   }

   if(arg1 == "Test 7"){
       arenaFile_m = "/home/pi/ece4534/test7.txt";
       arenaNum_m = "07";
   }

   this->on_clearAccStatButton_clicked();
   this->on_clear_net_stats_clicked();
   arena_m->clearAll();
   initTestCase();


}

///////////////////////////////////////////////////////////////////////////////
/// MENU ITEMS
///////////////////////////////////////////////////////////////////////////////
void MainWindow::on_actionCalibrate_Lead_Rover_triggered()
{
    QByteArray type;
    QByteArray out;

    qDebug() << "SENDING CALIBRATE LEAD ROVER SIGNAL";

    type = "62";                      // CALIBRATE_ROVER
    out = "4142434445464748494A4B4C"; // Filler Data

    emit writeSerialToWorker(type, out);
}

void MainWindow::on_actionCalibrate_Sensors_triggered()
{
    QByteArray type;
    QByteArray out;

    qDebug() << "SENDING CALIBRATE SENSORS SIGNAL";

    type = "61";                      // CALIBRATE_ROVER
    out = "0142434445464748494A4B4C"; // Filler Data

    emit writeSerialToWorker(type, out);
}

void MainWindow::on_actionData_Collect_triggered()
{
    QByteArray type;
    QByteArray out;

    qDebug() << "SENDING SENSOR DATA COLLECT MODE SIGNAL";

    type = "61";                      // CALIBRATE_ROVER
    out = "0242434445464748494A4B4C"; // Filler Data

    emit writeSerialToWorker(type, out);
}

void MainWindow::on_actionClear_Lead_Rover_Path_triggered()
{
    arena_m->clearLeadRoverPath();
}

///////////////////////////////////////////////////////////////////////////////
/// PUSH BUTTONS
///////////////////////////////////////////////////////////////////////////////

void MainWindow::on_pushButton_clicked()
{

    qDebug() << "button clicked";

    QByteArray type;
    QByteArray out;

    type = "78";                      // Toggle LED
    out = "4142434445464748494A4B4C"; // Filler Data

    emit writeSerialToWorker(type, out);
}

void MainWindow::on_pushButton_2_clicked()
{
    ui->textEdit->clear();
}

void MainWindow::on_quitButton_clicked()
{

    serial_thread_m->quit();
    while(serial_thread_m->isRunning());

    guiModel_thread_m->quit();
    while(guiModel_thread_m->isRunning());

    delete ui;

}

void MainWindow::on_clear_net_stats_clicked()
{
    emit clearNetStats();

    QByteArray type;
    QByteArray out;

    type = "75";                      // Clear Netstats
    out = "4142434445464748494A4B4C"; // Filler Data

    emit writeSerialToWorker(type ,out);
}

void MainWindow::on_run_button_clicked()
{
    // clear stuff?
    startGame();

}

void MainWindow::on_clearAccStatButton_clicked()
{
    emit clearAccStats();

    QByteArray type;
    QByteArray out;

    type = "68";                      // Clear Netstats
    out = "4142434445464748494A4B4C"; // Filler Data

    emit writeSerialToWorker(type ,out);
    clearRoverAccStats();
}
