// Copyright (c) 2021 WuHan University. All Rights Reserved
// @author yhaoli@whu.edu.cn
// @date 2021/01/31
// @brief point cloud io
#pragma once

#include "visualizeLibs.h"

namespace VisualTool {

	struct Point3d
	{
		double x;
		double y;
		double z;
		Point3d(double x0 = 0, double y0 = 0, double z0 = 0)
			: x(x0), y(y0), z(z0)
		{}
	};

	typedef Point3d Offset;

	namespace PointIO
	{
		template <typename T>
		extern bool loadSingleLAS(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Point3d& las_offset);

		template <typename T>
		extern bool saveLAS(const std::string& filepath, const typename pcl::PointCloud<T>::Ptr& cloud,
			const Point3d& offset = Point3d());

		template <typename T>
		extern bool loadPCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud);

		template <typename T>
		extern bool savePCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud,
			const Point3d& offset = Point3d());

	} // namespace PointIO
} // namespace VisualTool

#define TEMPLATE_POINTCLOUD_IO
#include "PointCloudIO.cpp"
