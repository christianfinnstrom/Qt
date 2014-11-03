#-------------------------------------------------
#
# Project created by QtCreator 2014-10-23T11:52:30
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = DynamixelControl
CONFIG   += console
CONFIG   -= app_bundle

LIBS += -LC:\Qt\Tools\QtCreator\bin\DynamixelControl -ldynamixel

TEMPLATE = app

INCLUDEPATH += C:\Qt\Tools\QtCreator\bin\DynamixelControl

SOURCES += main.cpp \
    DynamixelControl.cpp

OTHER_FILES += \
    dynamixel.lib \
    DynamixelControl32.dll \
    dynamixel.def

HEADERS += \
    dynamixel_control.h \
    DynamixelControl.h
