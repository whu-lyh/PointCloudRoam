#pragma once

#ifndef POINTCLOUDROAMLIB_H
#define POINTCLOUDROAMLIB_H
/*
this lib files is used for qt5	osg3.4+ glog  liblas1.8.1 pcl-1.8.1
*/

#pragma warning(disable: 4273)
#pragma warning(disable: 4819)
#pragma warning(disable: 4138)

#include <iostream>

#include <pcl/kdtree/kdtree_flann.h>
#include <osg/Point>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osg/Light>
#include <osg/Material>
#include <osg/PagedLOD>
#include <osg/MatrixTransform>
#include <osgDB/Input>
#include <osg/PositionAttitudeTransform>
#include <osgGA/OrbitManipulator>
#include <osgUtil/Optimizer>

#include <osgQt/GraphicsWindowQt>
#include <osgQt/QFontImplementation>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

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
#include <QThread>
#include <QTime>
#include <QGridLayout>

#ifdef _DEBUG
#pragma comment(lib, "OpenThreadsd.lib")
#pragma comment(lib, "osgd.lib")
#pragma comment(lib, "osgDBd.lib")
#pragma comment(lib, "osgUtild.lib")
#pragma comment(lib, "osgViewerd.lib")
#pragma comment(lib, "osgGAd.lib")
#pragma comment(lib, "osgQtd.lib")
#pragma comment(lib, "OpenThreadsd.lib")
#pragma comment(lib, "osgTextd.lib")
#pragma comment(lib, "liblas.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_kdtree_debug.lib")

#pragma comment(lib, "glogd.lib")
#pragma comment(lib, "opencv_core347d.lib")

#pragma comment(lib, "Qt5Cored.lib")
#pragma comment(lib, "Qt5Guid.lib")
#pragma comment(lib, "Qt5Widgetsd.lib")
#pragma comment(lib, "Qt5OpenGLd.lib")
#pragma comment(lib, "opengl32.lib")

#else
#pragma comment(lib, "osg.lib")
#pragma comment(lib, "osgDB.lib")
#pragma comment(lib, "osgUtil.lib")
#pragma comment(lib, "osgViewer.lib")
#pragma comment(lib, "osgGA.lib")
#pragma comment(lib, "osgQt.lib")
#pragma comment(lib, "OpenThreads.lib")
#pragma comment(lib, "osgText.lib")
#pragma comment(lib, "liblas.lib")
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")

#pragma comment(lib, "glog.lib")
#pragma comment(lib, "opencv_core347.lib")

#pragma comment(lib, "Qt5Core.lib")
#pragma comment(lib, "Qt5Gui.lib")
#pragma comment(lib, "Qt5Widgets.lib")
#pragma comment(lib, "Qt5OpenGL.lib")
#pragma comment(lib, "opengl32.lib")

#endif

#endif //POINTCLOUDROAMLIB_H