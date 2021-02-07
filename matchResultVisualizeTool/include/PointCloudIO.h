/*
	Copyright (c) 2021 WuHan University. All Rights Reserved
	First create: 2021/01/31
	Detail: point cloud io function
	Author: yuhao li
	Email:	yhaoli@whu.edu.cn
*/

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

		// operator override as global friend function
		friend Point3d operator+(const Point3d &pt1, const Point3d &pt2)
		{
			Point3d p;
			p.x = pt1.x + pt2.x;
			p.y = pt1.y + pt2.y;
			p.z = pt1.z + pt2.z;

			return p;
		}

		friend Point3d operator/(const Point3d &pt1, const int &n)
		{
			Point3d p;
			p.x = pt1.x / n;
			p.y = pt1.y / n;
			p.z = pt1.z / n;

			return p;
		}

		//operator override as member function
		inline Point3d operator +=(const Point3d& pt)
		{
			x += pt.x;
			y += pt.y;
			z += pt.z;
			return *this;
		}

		inline Point3d operator/=(const int &n)
		{
			this->x /= n;
			this->y /= n;
			this->z /= n;

			return *this;
		}
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
#include "../src/PointCloudIO.cpp"
