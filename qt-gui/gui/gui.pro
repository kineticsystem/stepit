#-------------------------------------------------
#
# Project created by QtCreator 2014-11-12T20:08:36
#
#-------------------------------------------------

# The following flags are used to add debug information into the release.
QMAKE_CXXFLAGS_RELEASE += -g
QMAKE_CFLAGS_RELEASE += -g
QMAKE_LFLAGS_RELEASE =

# Compile with GCC c++14 functionalities.
CONFIG += c++17

QT += core
QT += gui
QT += serialport
QT += widgets

greaterThan(QT_MAJOR_VERSION, 5)

TARGET = StepIt
TEMPLATE = app

INCLUDEPATH = serial \
    board
DEPENDPATH = serial \
    board

SOURCES = $$files(*.cpp) \
          $$files(serial/*.cpp) \
          $$files(board/*.cpp)
HEADERS = $$files(*.h) \
          $$files(serial/*.h) \
          $$files(board/*.h)

FORMS   = $$files(*.ui)

RC_FILE = icon.rc

#release: DESTDIR = release
#debug:   DESTDIR = debug
#OBJECTS_DIR = $$DESTDIR
#MOC_DIR = $$DESTDIR
#RCC_DIR = $$DESTDIR
#UI_DIR = $$DESTDIR
