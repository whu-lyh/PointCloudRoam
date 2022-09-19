/*
	Copyright (c) 2021 WuHan University. All Rights Reserved
	First create: 2021/02/06
	Detail: point cloud node for visualization based on OSG render engine
	Author: yuhao li
	Email:	yhaoli@whu.edu.cn
*/

#ifndef POINTCLOUDNODE_H
#define POINTCLOUDNODE_H

#pragma once

#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

#include "PointCloudIO.h"

namespace VisualTool {
	namespace Node {
		template <typename PointT>
		class pointCloudNode
		{
			typedef boost::shared_ptr<pcl::PointCloud<PointT>> pc_ptr;
		public:
			pointCloudNode();
			~pointCloudNode();
			pointCloudNode(const std::string file_path, const osg::Vec4 node_color = osg::Vec4(1.f, 0.f, 0.f, 1.f));

			osg::ref_ptr<osg::Geode> getGeoNode();
			VisualTool::Offset getOffset();
			void setAveOffset(const VisualTool::Offset &off);
			void setDownSampleInterval(const int gap);
			void setTransformation(const Eigen::Matrix4f &trans);

		private:
			std::string m_sFile_path;
			pc_ptr m_PointCloudPtr;
			VisualTool::Offset m_offset;
			int m_intervel;
			Eigen::Matrix4f m_trans;
			osg::ref_ptr<osg::Geode> m_pGeoNode;
			osg::Vec4 m_v4Color;
		};
	} //Node
}// VisualTool

#define TEMPLATE_POINTCLOUD_NODE
#include "../src/pointCloudNode.cpp"

#endif //POINTCLOUDNODE_H