#-------------------------------------------------
#
# Project created by QtCreator 2016-03-09T10:58:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += serialport

TARGET = testGui2
target.files = testGui2
target.path = /home/pi/ece4534
INSTALLS += target

TEMPLATE = app



SOURCES += main.cpp\
        mainwindow.cpp \
    guimodel.cpp \
    roverindicator.cpp \
    arena.cpp \
    serial_worker.cpp

HEADERS  += mainwindow.h \
    guimodel.h \
    comm_k.h \
    roverindicator.h \
    roverghost.h \
    objecttoken.h \
    arena.h \
    serial_worker.h

FORMS    += mainwindow.ui
