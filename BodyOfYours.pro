#-------------------------------------------------
#
# Project created by QtCreator 2015-01-29T13:46:19
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

INCLUDEPATH += gui

TARGET = BodyOfYours
TEMPLATE = app


SOURCES += src/main.cpp \
           src/mainwindow.cpp

HEADERS  += src/mainwindow.h

FORMS    += gui/mainwindow.ui


INCLUDEPATH += "D:/Programowanie/Biblioteki/Tidy/pcl-1.7/include/pcl-1.7"
INCLUDEPATH += D:/Programowanie/Biblioteki/VTK5.8.0_vc2010_x86_qt4.8/include/vtk-5.8
INCLUDEPATH += D:/Programowanie/Biblioteki/boost_1.50_msvc2010_x86/include
INCLUDEPATH += D:/Programowanie/Biblioteki/Tidy/Eigen/include


win32:LIBS += -LD:/Programowanie/Biblioteki/vtk-5.8.0_vc2010_x86_qt4.8_build/bin/Debug \
                  -lQVTK -lvtkCommon -lvtkRendering -lvtkFiltering -lvtkGraphics \
                  -lvtkIO -lvtkRendering -lvtksys

win32:LIBS += -LD:/Programowanie/Biblioteki/boost_1.50_msvc2010_x86/lib \
                  -llibboost_thread-vc100-mt-gd-1_50 -llibboost_date_time-vc100-mt-gd-1_50 \
                  -llibboost_system-vc100-mt-gd-1_50 -llibboost_chrono-vc100-mt-gd-1_50 \
                  -llibboost_filesystem-vc100-mt-gd-1_50
win32:LIBS += -L"D:/Programowanie/Biblioteki/Tidy/pcl-1.7/lib" \
                  -lpcl_visualization_debug -lpcl_common_debug -lpcl_io_debug
