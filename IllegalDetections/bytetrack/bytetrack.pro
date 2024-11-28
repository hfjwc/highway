TEMPLATE = lib
#DEFINES += BYTETRACK_LIBRARY
CONFIG += staticlib
CONFIG -= qt
CONFIG += c++17
DESTDIR = ../release/lib


DEFINES += QT_DEPRECATED_WARNINGS
SOURCES += \
    bytetrack.cpp \
    kalmanfilter.cpp \
    lapjv.cpp \
    strack.cpp

HEADERS += \
    bytetrack.h \
#    bytetrack_global.h \
    kalmanfilter.h \
    lapjv.h \
    strack.h

#BMNNSDK_PATH=/home/lj/bm1684/soc-sdk
#QMAKE_RPATH += $${BMNNSDK_PATH}/lib
#QMAKE_LFLAGS += -Wl,-rpath=$${BMNNSDK_PATH}/lib
#QMAKE_LFLAGS += -L$${BMNNSDK_PATH}/lib -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio
#INCLUDEPATH += $${BMNNSDK_PATH}/include/sophon-opencv_0.8.0/include/opencv4 \
#        $${BMNNSDK_PATH}/include/sophon-ffmpeg_0.8.0/include \
#        $${BMNNSDK_PATH}/include/libsophon-0.4.9/include

#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
unix:!macx: LIBS += -L$$PWD/../../soc-sdk/lib/ -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_video

INCLUDEPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
DEPENDPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include
