QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
QMAKE_LFLAGS+=-fuse-ld=gold
# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target



unix:!macx: LIBS += -L$$PWD/../../soc-sdk/lib/ -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_video -lopencv_freetype

INCLUDEPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
DEPENDPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include

unix:!macx: LIBS += -L$$PWD/../release/lib/ -ltruckoccupylane -lcommon -lmodelsFactory

INCLUDEPATH += $$PWD/../truckoccupylane
DEPENDPATH += $$PWD/../truckoccupylane

INCLUDEPATH += $$PWD/../common
DEPENDPATH += $$PWD/../common

INCLUDEPATH += $$PWD/../modelsFactory
DEPENDPATH += $$PWD/../modelsFactory

unix:!macx: LIBS += -L$$PWD/../release/lib/ -lbytetrack

INCLUDEPATH += $$PWD/../bytetrack
DEPENDPATH += $$PWD/../bytetrack

unix:!macx: PRE_TARGETDEPS += $$PWD/../release/lib/libbytetrack.a



