TEMPLATE = lib
QT -= gui
QT += core concurrent

CONFIG  += dll
CONFIG  += c++11
CONFIG += plugin
DEFINES += COMMON_LIBRARY
QMAKE_LFLAGS+=-fuse-ld=gold
DESTDIR = ../release/lib
TARGET=common


SOURCES += \
    ff_decode.cpp\
    savelog.cpp\

HEADERS += \
    bmnn_utils.h\
    bm_wrapper.hpp\
    common.h \
    draw_utils.hpp \
    ff_decode.hpp\
    json.hpp\
    utils.hpp\
    savelog.h\
    global.h\

#BMNNSDK_PATH=../soc-sdk
#QMAKE_RPATH += $${BMNNSDK_PATH}/lib
#QMAKE_LFLAGS += -Wl,-rpath=$${BMNNSDK_PATH}/lib
#QMAKE_LFLAGS += -L$${BMNNSDK_PATH}/lib -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio
#INCLUDEPATH += $${BMNNSDK_PATH}/include/sophon-opencv_0.8.0/include/opencv4 \
#        $${BMNNSDK_PATH}/include/sophon-ffmpeg_0.8.0/include \
#        $${BMNNSDK_PATH}/include/libsophon-0.4.9/include

#DEFINES += QT_MESSAGELOGCONTEXT

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../soc-sdk/lib/release/ -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../soc-sdk/lib/debug/ -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio
#else:unix: LIBS += -L$$PWD/../soc-sdk/lib/ -lbmrt

#INCLUDEPATH += $$PWD/../soc-sdk/include/libsophon-0.4.9/include
#DEPENDPATH += $$PWD/../soc-sdk/include/libsophon-0.4.9/include
#INCLUDEPATH += $$PWD/../soc-sdk/include/sophon-ffmpeg_0.8.0/include
#DEPENDPATH += $$PWD/../soc-sdk/include/sophon-ffmpeg_0.8.0/include
#INCLUDEPATH += $$PWD/../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
#DEPENDPATH += $$PWD/../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4

unix:!macx: LIBS += -L$$PWD/../../soc-sdk/lib/ -lbmrt -lbmlib -lbmcv -ldl -lbmvideo -lswresample -lswscale -lavformat -lavutil -lavcodec -lpthread -lbmjpuapi -lbmjpulite -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_video


#unix:!macx: LIBS += -L$$PWD/../release/lib/ -lmodelsFactory

#INCLUDEPATH += $$PWD/../modelsFactory
#DEPENDPATH += $$PWD/../modelsFactory

INCLUDEPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
DEPENDPATH += $$PWD/../../soc-sdk/include/libsophon-0.4.9/include
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-opencv_0.8.0/include/opencv4
INCLUDEPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include
DEPENDPATH += $$PWD/../../soc-sdk/include/sophon-ffmpeg_0.8.0/include



