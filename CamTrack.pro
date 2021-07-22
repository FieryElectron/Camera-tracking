TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /usr/local/include/opencv4


LIBS += -L/usr/local/lib

LIBS += -lopencv_core
LIBS += -lopencv_viz
LIBS += -lopencv_calib3d

LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs
