#pragma once

/*
this lib files is used for qt5.13.0	osg3.6.4 liblas pcl-1.8.1
*/

#pragma warning(disable: 4273)
#pragma warning(disable: 4819)

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
#pragma comment(lib, "jsoncpp.lib")
#pragma comment(lib, "liblas.lib")
#pragma comment(lib, "liblas_c.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_kdtree_debug.lib")

#pragma comment(lib, "Qt5Core.lib")
#pragma comment(lib, "Qt5Guid.lib")
#pragma comment(lib, "Qt5Widgetsd.lib")
#pragma comment(lib, "Qt5OpenGLd.lib")

#else
#pragma comment(lib, "osg.lib")
#pragma comment(lib, "osgDB.lib")
#pragma comment(lib, "osgUtil.lib")
#pragma comment(lib, "osgViewer.lib")
#pragma comment(lib, "osgGA.lib")
#pragma comment(lib, "osgQt.lib")
#pragma comment(lib, "OpenThreads.lib")
#pragma comment(lib, "osgText.lib")
#pragma comment(lib, "jsoncpp.lib")
#pragma comment(lib, "liblas.lib")
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")

#pragma comment(lib, "Qt5Core.lib")
#pragma comment(lib, "Qt5Gui.lib")
#pragma comment(lib, "Qt5Widgets.lib")
#pragma comment(lib, "Qt5OpenGL.lib")

#endif