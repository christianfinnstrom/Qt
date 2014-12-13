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

# Make sure this path points to the lib-file directory (DynamixelControl folder)
# Example: LIBS += -L"path" -ldynamixel, eg. change "path" to DynamixelControl folder destination
LIBS += -LC:\Users\Christian\Documents\GitHub\Qt\DynamixelControl -ldynamixel

TEMPLATE = app

# make sure this path is correct. It must lead to the DynamixelControl folder where the headers are.
INCLUDEPATH += C:\Users\Christian\Documents\GitHub\Qt\DynamixelControl

SOURCES += main.cpp \
    sensorcontrol.cpp \
    actuatorcontrol.cpp

OTHER_FILES += \
    dynamixel.lib \
    dynamixel.def \
    DynamixelControl32.dll \
    DynamixelControl.pro.user

HEADERS += \
    dynamixel_control.h \
    sensorcontrol.h \
    actuatorcontrol.h
