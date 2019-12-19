#pragma once

#pragma warning(disable: 4273)
#pragma warning(disable: 4819)

#ifdef _DEBUG
#pragma comment(lib, "OpenThreadsd.lib")
#pragma comment(lib, "osgd.lib")
#pragma comment(lib, "osgDBd.lib")
#pragma comment(lib, "osgUtild.lib")
#pragma comment(lib, "osgViewerd.lib")
#pragma comment(lib, "osgGAd.lib")
#pragma comment(lib, "osgTextd.lib")
#pragma comment(lib, "jsoncpp.lib")
#pragma comment(lib, "liblas.lib")
#pragma comment(lib, "liblas_c.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_kdtree_debug.lib")
#else
#pragma comment(lib, "osg.lib")
#pragma comment(lib, "osgDB.lib")
#pragma comment(lib, "osgUtil.lib")
#pragma comment(lib, "osgViewer.lib")
#pragma comment(lib, "osgGA.lib")
#pragma comment(lib, "osgText.lib")
#pragma comment(lib, "jsoncpp.lib")
#pragma comment(lib, "liblas.lib")
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")
#endif