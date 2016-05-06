#ifndef SERIAL_WORKER_H
#define SERIAL_WORKER_H

#include <QObject>
#include <QMutex>
#include <QtSerialPort/QSerialPort>
#include <QString>
#include <QDebug>
#include <iostream>
#include <QPair>
#include <QPointF>
#include <QList>

#include "comm_k.h"

#define INCOMING_HEADER_SIZE 4

class Serial_worker : public QObject
{
    Q_OBJECT
public:
    explicit Serial_worker(QObject *parent = 0);
    ~Serial_worker();

    void abort();

signals:

    // signal to send string to screen
    void printToTextEdit(QString out);
    void printToTextEdit_NL(QString out);

    // send recieved msg to guimodel
    void sendMsgtoModel(RpiMsg outmsg);

public slots:

    void doWork();
    void readSerial();
    void writeSerial(const QByteArray &type, const QByteArray &data);
    void setInitList( initialList);

private slots:

    void sendInitialdataToPIC();


private:
    QMutex txMutex;
    QSerialPort *somePort;
    QByteArray dataArray;
    char dataBuffer[64];
    qint64 maxSize;
    RpiMsg rpiMsg;
    int itt;
    int data_itt;
    bool done;
    bool doneLock;
    QList<QPair<QString, QPointF> > initList;

};

#endif // SERIAL_WORKER_H
