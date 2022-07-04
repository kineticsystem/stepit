# The following flags are used to add debug information into the release.
QMAKE_CXXFLAGS_RELEASE += -g
QMAKE_CFLAGS_RELEASE += -g
QMAKE_LFLAGS_RELEASE =

# Compile with GCC c++17 functionalities.
CONFIG += c++17

greaterThan(QT_MAJOR_VERSION, 5)

TEMPLATE = subdirs
SUBDIRS += gui/gui.pro
