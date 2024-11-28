QT -= gui

TEMPLATE = lib
DEFINES += CARNOKEEPDISTANCE_LIBRARY

CONFIG += c++11
CONFIG += plugin
DESTDIR = ../release/lib
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

SOURCES += \
    cardetector.cpp \
    carnokeepdistance.cpp

HEADERS += \
    cardetector.h \
    carnokeepdistance.h

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target


unix:!macx: LIBS += -L$$PWD/../../soc-sdk/lib/ -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_video -lopencv_freetype

INCLUDEPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
DEPENDPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include




unix:!macx: LIBS += -L$$PWD/../release/lib/ -lmodelsFactory -lphonesafetybelt

INCLUDEPATH += $$PWD/../modelsFactory
DEPENDPATH += $$PWD/../modelsFactory

unix:!macx: LIBS += -L$$PWD/../release/lib/ -lbytetrack

INCLUDEPATH += $$PWD/../bytetrack
DEPENDPATH += $$PWD/../bytetrack

unix:!macx: PRE_TARGETDEPS += $$PWD/../release/lib/libbytetrack.a

unix:!macx: LIBS += -L$$PWD/../release/lib/ -lcommon

INCLUDEPATH += $$PWD/../common
DEPENDPATH += $$PWD/../common


QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO



unix:!macx: LIBS += -L$$PWD/../release/lib/ -lphonesafetybelt

INCLUDEPATH += $$PWD/../release/include
DEPENDPATH += $$PWD/../release/include

unix:!macx: LIBS += -L$$PWD/../release/lib/ -ltruckoccupylane

INCLUDEPATH += $$PWD/../truckoccupylane
DEPENDPATH += $$PWD/../truckoccupylane


INCLUDEPATH += $$PWD/../release
DEPENDPATH += $$PWD/../release

INCLUDEPATH += $$PWD/../decodeRadarData
DEPENDPATH += $$PWD/../decodeRadarData