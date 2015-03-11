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


SOURCES += src/data/patient.cpp \
           src/data/scan.cpp \
           src/addpatientdialog.cpp \
           src/file_scanner.cpp \
           src/json.cpp \
           src/main.cpp \
           src/mainwindow.cpp \
           src/patient_tree_model.cpp \
           src/reme_scanner_3d.cpp \
           src/reme_sensor_check.cpp \
           src/scanner_3d.cpp \
           src/scanning_3d.cpp \
           src/scanningwindow.cpp \
           src/scans_data_tree.cpp \
           src/scans_tree.cpp \
           src/scans_viewer.cpp \
           src/sensor_viewer.cpp \
           src/scaninfodialog.cpp


HEADERS  += src/data/patient.h \
            src/data/scan.h \
            src/addpatientdialog.h \
            src/file_scanner.h \
            src/json.h \
            src/mainwindow.h \
            src/patient_tree_model.h \
            src/reme_scanner_3d.h \
            src/reme_sensor_check.h \
            src/resources.h \
            src/scanner_3d.h \
            src/scanning_3d.h \
            src/scanningwindow.h \
            src/scans_data_tree.h \
            src/scans_tree.h \
            src/scans_viewer.h \
            src/sensor_viewer.h \
            src/scaninfodialog.h


FORMS    += src/mainwindow.ui \
            src/scanningwindow.ui \
            src/patientinfodialog.ui \
            src/scaninfodialog.ui


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
