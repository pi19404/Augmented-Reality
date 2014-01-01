#-------------------------------------------------
#
# Project created by QtCreator 2013-12-22T16:26:25
#
#-------------------------------------------------

QT       -= core gui

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

LIBS += -L/usr/local/lib `pkg-config --libs opencv` -lrt -lm -L/media/windows/pi19404/REPO/Infurnia/ARToolKit/lib -lARgsub   -lAR -lglut -lGLU -lGL -lXi -lXmu -lX11 -lm
LIBS += -L/media/UBUNTU/repository/OpenVisionLibrary/OpenVision/build -lOpenVisionLibrary -lfftw3 -lfftw3f -L/media/UBUNTU/repository/Research/lsd_opencv/lsd_1.6/lib -llsd_opencv

INCLUDEPATH += /usr/local/include /usr/local/include/opencv /usr/local/include/opencv2 /media/windows/pi19404/REPO/Infurnia/ARToolKit/include /media/UBUNTU/repository/OpenVisionLibrary/OpenVision/
INCLUDEPATH +=/media/UBUNTU/repository/OpenVisionLibrary/OpenVision/ImgProc /media/UBUNTU/repository/Research/lsd_opencv/lsd_1.6/include
QMAKE_CXXFLAGS +=-O -I/usr/X11R6/include -g


TARGET = ARTest
TEMPLATE = app


SOURCES += main.cpp \
    /media/UBUNTU/repository/OpenVisionLibrary/OpenVision/others/ACE/ace.cpp \
    ../../ARToolKit/lib/SRC/AR/vTridiag.c \
    ../../ARToolKit/lib/SRC/AR/vInnerP.c \
    ../../ARToolKit/lib/SRC/AR/vHouse.c \
    ../../ARToolKit/lib/SRC/AR/vFree.c \
    ../../ARToolKit/lib/SRC/AR/vDisp.c \
    ../../ARToolKit/lib/SRC/AR/vAlloc.c \
    ../../ARToolKit/lib/SRC/AR/paramGet.c \
    ../../ARToolKit/lib/SRC/AR/paramFile.c \
    ../../ARToolKit/lib/SRC/AR/paramDistortion.c \
    ../../ARToolKit/lib/SRC/AR/paramDisp.c \
    ../../ARToolKit/lib/SRC/AR/paramDecomp.c \
    ../../ARToolKit/lib/SRC/AR/paramChangeSize.c \
    ../../ARToolKit/lib/SRC/AR/mUnit.c \
    ../../ARToolKit/lib/SRC/AR/mTrans.c \
    ../../ARToolKit/lib/SRC/AR/mSelfInv.c \
    ../../ARToolKit/lib/SRC/AR/mPCA.c \
    ../../ARToolKit/lib/SRC/AR/mMul.c \
    ../../ARToolKit/lib/SRC/AR/mInv.c \
    ../../ARToolKit/lib/SRC/AR/mFree.c \
    ../../ARToolKit/lib/SRC/AR/mDup.c \
    ../../ARToolKit/lib/SRC/AR/mDisp.c \
    ../../ARToolKit/lib/SRC/AR/mDet.c \
    ../../ARToolKit/lib/SRC/AR/mAllocUnit.c \
    ../../ARToolKit/lib/SRC/AR/mAllocTrans.c \
    ../../ARToolKit/lib/SRC/AR/mAllocMul.c \
    ../../ARToolKit/lib/SRC/AR/mAllocInv.c \
    ../../ARToolKit/lib/SRC/AR/mAllocDup.c \
    ../../ARToolKit/lib/SRC/AR/mAlloc.c \
    ../../ARToolKit/lib/SRC/AR/arUtil.c \
    ../../ARToolKit/lib/SRC/AR/arLabeling.c \
    ../../ARToolKit/lib/SRC/AR/arGetTransMatCont.c \
    ../../ARToolKit/lib/SRC/AR/arGetTransMat3.c \
    ../../ARToolKit/lib/SRC/AR/arGetTransMat2.c \
    ../../ARToolKit/lib/SRC/AR/arGetTransMat.c \
    ../../ARToolKit/lib/SRC/AR/arGetMarkerInfo.c \
    ../../ARToolKit/lib/SRC/AR/arGetCode.c \
    ../../ARToolKit/lib/SRC/AR/arDetectMarker2.c \
    ../../ARToolKit/lib/SRC/AR/arDetectMarker.c

OTHER_FILES += \
    ../../ARToolKit/lib/SRC/VideoLinuxV4L/ccvt_i386.S

HEADERS += \
    ../../ARToolKit/lib/SRC/VideoLinuxV4L/jpegtorgb.h \
    ../../ARToolKit/lib/SRC/VideoLinuxV4L/ccvt.h


