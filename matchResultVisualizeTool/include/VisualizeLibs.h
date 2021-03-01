/*
	Copyright (c) 2021 WuHan University. All Rights Reserved
	First create: 2021/02/06
	Detail: extra dependent libraries
	Author: yuhao li
	Email:	yhaoli@whu.edu.cn
*/

#ifndef VISUALIZELIBS_H
#define VISUALIZELIBS_H

#pragma once

#pragma warning(disable: 4273)
#pragma warning(disable: 4819)

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <boost/make_shared.hpp>

#include <glog/logging.h>

#ifdef _DEBUG
#pragma comment(lib, "OpenThreadsd.lib")
#pragma comment(lib, "osgd.lib")
#pragma comment(lib, "osgDBd.lib")
#pragma comment(lib, "osgUtild.lib")
#pragma comment(lib, "osgViewerd.lib")
#pragma comment(lib, "osgGAd.lib")
#pragma comment(lib, "osgTextd.lib")

#pragma comment(lib, "liblas.lib")

#pragma comment(lib, "glogd.lib")

#pragma comment(lib, "yaml-cpp_d.lib")

#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_kdtree_debug.lib")
#else
#pragma comment(lib, "osg.lib")
#pragma comment(lib, "osgDB.lib")
#pragma comment(lib, "osgUtil.lib")
#pragma comment(lib, "osgViewer.lib")
#pragma comment(lib, "osgGA.lib")
#pragma comment(lib, "osgText.lib")

#pragma comment(lib, "liblas.lib")

#pragma comment(lib, "glog.lib")

#pragma comment(lib, "yaml-cpp.lib")

#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")
#endif

#endif //VISUALIZELIBS_H