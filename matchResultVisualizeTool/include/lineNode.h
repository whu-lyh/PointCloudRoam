/*
	Copyright (c) 2021 WuHan University. All Rights Reserved
	First create: 2021/02/06
	Detail: point cloud node for visualization based on OSG render engine
	Author: yuhao li
	Email:	yhaoli@whu.edu.cn
*/

#ifndef LINENODE_H
#define LINENODE_H

#pragma once

#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

#include "PointCloudIO.h"

namespace VisualTool {
	namespace Node {
		class lineNode
		{
		public:
			lineNode();
			~lineNode();
			lineNode(const std::string file_path, const osg::Vec4 pt_color = osg::Vec4(1.f, 1.f, 1.f, 1.f),
				const osg::Vec4 line_color = osg::Vec4(0.f, 0.f, 1.f, 1.f), const float radius = 0.1f, const float width = 0.1f);

			osg::ref_ptr<osg::Geode> getGeoNode();
			void setAveOffset(const VisualTool::Offset &off);

		private:
			std::string m_sFile_path;
			osg::ref_ptr<osg::Geode> m_pGeoNode;
			VisualTool::Offset m_offset;
			osg::Vec4 m_v4ptColor;
			osg::Vec4 m_v4lineColor;
			float m_fradius;
			float m_fwidth;
		};
	} //Node
}// VisualTool

#endif //LINENODE_H