#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGridLayout>
#include <osgQt/GraphicsWindowQt>
#include "mygraphicwindowqt.h"
#include <osgQt/QFontImplementation>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

#include <osg/Geode>
#include <osg/Camera>
#include <osg/ShapeDrawable>
#include <osg/Sequence>
#include <osg/PolygonMode>

#include <osgText/Font>
#include <osgText/Text>


#include <QWidget>
#include <QOpenGLContext>
#include <QWindow>
#include <QSurface>
#include <QDebug>

class MainWindow : public QWidget {
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow(){
        int iii=0;
        killTimer(_timerID);
    }

    virtual void paintEvent(QPaintEvent* event)
    {
        //_comViewer->frame();
    }
    virtual void timerEvent(QTimerEvent* event)
    {
        _comViewer->frame();
    }

    void setSceneData(osg::Node* node)
    {
        _viewer->setSceneData(node);
    }
    void setCameraManipulator(osgGA::CameraManipulator* manipulator, bool resetPosition = true)
    {
        _viewer->setCameraManipulator(manipulator, resetPosition);
    }



private:
    osgViewer::Viewer* _viewer;
    osgViewer::CompositeViewer* _comViewer;
    int               _timerID;
    MyGraphicWindowQt* _graphicsWindow;
public slots:
    void onStartTimer();

};

#endif // MAINWINDOW_H
