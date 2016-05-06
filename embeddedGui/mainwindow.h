#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QCoreApplication>
#include "arena.h"
#include "serial_worker.h"
#include "guimodel.h"
#include "comm_k.h"
#include <QPair>
#include <QPointF>
#include <QList>
#include <QTime>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:

    void writeToTextEdit(QString in);
    void writeToTextEdit_NL(QString in);
    void receiveNetStatGuiUpdate(int lrMissing, int frMissing, int coMissing, int seMissing,
                                 QString lrStatus, QString frStatus, QString coStatus, QString seStatus,
                                 int lrTotal, int frTotal, int coTotal, int seTotal,
                                 int missingTotal, int messageTotal);
    void updateRoverAcc(double leadRoverTravelError, double leadRoverTravelErrorMax,
                            double leadRoverRotationalError, double leadRoverRotationalErrorMax);
    void updateLeadRover(QPointF inPoint, int inRot);
    void updateObject(QString obj, QPointF inPoint, int inRot);
    void updateFollower(QPointF inPoint, int inRot );
    void updateLeadRoverTokenStats(QString nearest, QPointF leadRoverPosition);
    void startGame();
    void endGame(int tokensRetreived);
    void updateTokenStats(int tokensFound, int tokensRetreived);
    void updateRuntime(QElapsedTimer runtime, QTime othertime);
    void updateSensorStats(double lrAcc, double sensorAcc, double sensorMax);
    void tokenSensorStats(double t1error,double  t2error,double  t3error,double  t4error);
    void obstacleSensorStats(double o1error,double  o2error,double  o3error,double  o4error);
    void initTestCase();
    void totalTokens(int totalTokens);
    void clearLeadRoverTokenStats();
    void clearRoverAccStats();

signals:

    void operate();
    void clearNetStats();
    void clearAccStats();
    void writeSerialToWorker(QByteArray, QByteArray );
    void sendInitList(initialList);
    void startGameToModel();
    void processFoundToken();
    void clearLeadRoverPath();


private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_quitButton_clicked();

    void on_clear_net_stats_clicked();

    void process_initData(QString line);

    void on_clearAccStatButton_clicked();

    void on_comboBox_currentIndexChanged(const QString &arg1);

    void on_run_button_clicked();

    void on_actionCalibrate_Lead_Rover_triggered();

    void on_actionCalibrate_Sensors_triggered();

    void on_actionData_Collect_triggered();

    void on_actionClear_Lead_Rover_Path_triggered();

private:
    Ui::MainWindow *ui;

    QThread *serial_thread_m;

    QThread *guiModel_thread_m;

    QThread *otherThread_m;

    Serial_worker *serial_worker_m;

    GuiModel * gui_model_m;

    Arena * arena_m;

    QList<QPair<QString, QPointF> > initList_m;

    QString arenaFile_m;

    QByteArray arenaNum_m;

    int totalTokens_m;

};

#endif // MAINWINDOW_H
