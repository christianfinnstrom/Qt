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

# make sure this path is correct. It must lead to the DynamixelControl folder.
INCLUDEPATH += C:\Users\Christian\Documents\GitHub\Qt\DynamixelControl

SOURCES += main.cpp \
    DynamixelControl.cpp

OTHER_FILES += \
    dynamixel.lib \
    DynamixelControl32.dll \
    dynamixel.def

HEADERS += \
    dynamixel_control.h \
    DynamixelControl.h
