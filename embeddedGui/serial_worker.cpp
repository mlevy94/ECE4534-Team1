#include "serial_worker.h"


Serial_worker::Serial_worker(QObject *parent) : QObject(parent)
{
    doneLock = false;
    done = true;
    itt = 0;
    data_itt = 0;
}

Serial_worker::~Serial_worker(){

    if(somePort->isOpen()){
        somePort->close();
    }

    delete somePort;

}

void Serial_worker::doWork()
{
   // do stuff
    qDebug() << QString("serial_worker started in new thread");

    emit printToTextEdit_NL(QString("serial: doing work"));

    somePort = new QSerialPort;
    somePort->close();
    somePort->setPortName("/dev/ttyAMA0");
    somePort->setBaudRate(57600);

    if(!somePort->open(QIODevice::ReadWrite)){
        emit printToTextEdit_NL(QString("serial: Can't open port"));
    }
    else{
        emit printToTextEdit_NL(QString("serial: Port opened\n"));
    }

    sendInitialdataToPIC();

    connect(somePort, SIGNAL(readyRead()), this, SLOT(readSerial()));



}

void Serial_worker::readSerial()
{
    int startByteLocation;
    int dataArraySize;

    dataArray += somePort->readAll();

    dataArraySize = dataArray.size();

    startByteLocation = dataArray.indexOf(0xAA);
    dataArray.remove(0, startByteLocation);

    if(startByteLocation >=0){

            if(dataArraySize >= INCOMING_HEADER_SIZE){ //header size?


                    rpiMsg.type = dataArray.at(1);
                    rpiMsg.source = dataArray.at(2);
                    rpiMsg.size = dataArray.at(3);

                    if(dataArray.size() >= INCOMING_HEADER_SIZE + rpiMsg.size){

                        for(itt = 0 ; itt < rpiMsg.size ; itt++){
                            rpiMsg.msg[itt] = dataArray[4 + itt];
                        }

                        dataArray.remove(0, INCOMING_HEADER_SIZE + rpiMsg.size);

                        emit sendMsgtoModel(rpiMsg);

                    }

                }

    }

}

void Serial_worker::writeSerial(const QByteArray &type, const QByteArray &data)
{
    QByteArray temp;
    temp.resize(data.size());

    somePort->write(QByteArray::fromHex("0014"));
    somePort->write(QByteArray::fromHex(type));
    somePort->write(QByteArray::fromHex(data));
    //somePort->write(QByteArray::fromHex("0C"));


    qDebug() << "Sending serial data";
    qDebug() << "Type : " << QByteArray::fromHex(type).toHex();
    qDebug()<< " Data : " << QByteArray::fromHex(data);
    qDebug() << "Size : " << QString::number(data.size() >> 1, 16);
    qDebug() << "Size : " << QByteArray::fromHex("0C");


}

void Serial_worker::sendInitialdataToPIC()
{
    int itt = 0;

    QStringList myOptions;
    myOptions << "T1" << "T2" << "T3" << "T4" << "O1" << "O2" << "O3" << "O4";

    QPair<QString, QPointF> tempPair;
    QString tempString;
    QPointF tempPointF;
    int xcord;
    int ycord;

    QByteArray type = "22"; // This is an initial data message
    QByteArray data = "";
    data.clear();



    for (itt = 0 ; itt < initList.size() ; itt++){

        tempPair = initList.at(itt);

        tempString = tempPair.first;

        switch(myOptions.indexOf(tempString)){

        case 0:
            data.append("22");
            break;
        case 1:
            data.append("21");
            break;
        case 2:
            data.append("20");
            break;
        case 3:
            data.append("19");
            break;
        case 4:
            data.append("18");
            break;
        case 5:
            data.append("17");
            break;
        case 6:
            data.append("16");
            break;
        case 7:
            data.append("15");
            break;

        }

        tempPointF = tempPair.second;
        xcord = tempPointF.x();
        ycord = tempPointF.y();

        data = (data + QByteArray::number(xcord, 16).rightJustified(4,'0') +
                QByteArray::number(ycord, 16).rightJustified(4,'0') ).leftJustified(24, 'A');

        qDebug() << data;

        writeSerial(type, data);

        data.clear();

    }
}

void Serial_worker::setInitList(QList<QPair<QString, QPointF> > initList_in)
{
    initList = initList_in;

    qDebug() << "Got the init list" << initList;
}




