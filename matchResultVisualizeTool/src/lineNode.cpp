
#pragma once
#include "../include/lineNode.h"
#include "../include/PointCloudIO.h"
#include "../include/FileUtility.h"

namespace VisualTool {
	namespace Node {

		lineNode::lineNode()
		{
			m_pGeoNode = new osg::Geode();
		}

		lineNode::~lineNode()
		{
			m_pGeoNode = NULL;
		}

		lineNode::lineNode(const std::string file_path, const osg::Vec4 pt_color, const osg::Vec4 line_color, 
			const float radius, const float width)
			:m_sFile_path(file_path),
			m_v4ptColor(pt_color.r(), pt_color.g(), pt_color.b(), pt_color.a()),
			m_v4lineColor(line_color.r(), line_color.g(), line_color.b(), line_color.a()),
			m_fradius(radius),
			m_fwidth(width)
		{
			m_pGeoNode = new osg::Geode();
		}

		void lineNode::setAveOffset(const VisualTool::Offset &off)
		{
			m_offset = off;
		}

		osg::ref_ptr<osg::Geode> lineNode::getGeoNode()
		{
			if (!VisualTool::FileUtility::FileExist(m_sFile_path))
			{
				LOG(WARNING) << "Match result file doesn't existed! " << m_sFile_path;
				return new osg::Geode();
			}

			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			osg::ref_ptr<osg::Geometry> geom_pt = new osg::Geometry();
			osg::ref_ptr<osg::Geometry> geom_line = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> coords_pt = new osg::Vec3Array();
			osg::ref_ptr<osg::Vec4Array> colors_pt = new osg::Vec4Array();
			osg::ref_ptr<osg::Vec3Array> coords_line = new osg::Vec3Array();
			osg::ref_ptr<osg::Vec4Array> colors_line = new osg::Vec4Array();

			int num_line = 0;
			std::ifstream file;
			file.open(m_sFile_path);
			int pair_count = 0;
			file >> pair_count;
			while (!file.eof())
			{
				++num_line;
				float x1, y1, z1, x2, y2, z2;
				file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;

				x1 = x1 - m_offset.x;
				y1 = y1 - m_offset.y;
				z1 = z1 - m_offset.z;
				x2 = x2 - m_offset.x;
				y2 = y2 - m_offset.y;
				z2 = z2 - m_offset.z;

				coords_pt->push_back(osg::Vec3(x1, y1, z1));
				coords_pt->push_back(osg::Vec3(x2, y2, z2));
				colors_pt->push_back(m_v4ptColor);
				colors_pt->push_back(m_v4ptColor);

				coords_line->push_back(osg::Vec3(x1, y1, z1));
				coords_line->push_back(osg::Vec3(x2, y2, z2));
				colors_line->push_back(m_v4lineColor);
				colors_line->push_back(m_v4lineColor);
			}
			file.close();

			coords_pt->pop_back();
			coords_pt->pop_back();
			colors_pt->pop_back();
			colors_pt->pop_back();
			coords_line->pop_back();
			coords_line->pop_back();
			colors_line->pop_back();
			colors_line->pop_back();
			--num_line;

			for (int i = 0; i < 2 * num_line; ++i)
			{
				osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere((*coords_pt)[i], m_fradius);
				osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = new osg::ShapeDrawable(sphere.get());
				shapeDrawable->setColor((*colors_pt)[i]);
				geode->addDrawable(shapeDrawable.get());
			}

			osg::ref_ptr<osg::LineWidth> LineSize = new osg::LineWidth;
			LineSize->setWidth(m_fwidth);
			geom_line->getOrCreateStateSet()->setAttributeAndModes(LineSize.get(), osg::StateAttribute::ON);
			geom_line->setVertexArray(coords_line.get());
			geom_line->setColorArray(colors_line.get());
			geom_line->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
			for (int i = 0; i < num_line; ++i)
			{
				geom_line->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 2 * i, 2));
			}
			geode->addDrawable(geom_line.get());

			return geode;
		}

	}// namespace Node
} // namespace VisualTool