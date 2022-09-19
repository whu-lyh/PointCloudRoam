
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

		int lineNode::getLineNum()
		{
			return m_iNum;
		}

		void lineNode::setAveOffset(const VisualTool::Offset &off)
		{
			m_offset = off;
		}

		void lineNode::setTransformation(const Eigen::Matrix4f &trans)
		{
			m_trans = trans;
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

			Eigen::Matrix3f r = m_trans.block<3, 3>(0, 0);
			Eigen::Vector3f t = m_trans.topRightCorner<3, 1>();

			int num_line = 0;
			std::ifstream file;
			file.open(m_sFile_path);

			file >> m_iNum;
			while (!file.eof())
			{
				++num_line;
				float x1 = 0.f, y1 = 0.f, z1 = 0.f, x2 = 0.f, y2 = 0.f, z2 = 0.f;
				file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;

				x1 = x1 - m_offset.x;
				y1 = y1 - m_offset.y;
				z1 = z1 - m_offset.z;
				x2 = x2 - m_offset.x;
				y2 = y2 - m_offset.y;
				z2 = z2 - m_offset.z;

				// only src point are transformed
				float x1_trans = 0.f, y1_trans = 0.f, z1_trans = 0.f;
				x1_trans = r(0, 0)*x1 + r(0, 1)*y1 + r(0, 2)*z1 + t[0];
				y1_trans = r(1, 0)*x1 + r(1, 1)*y1 + r(1, 2)*z1 + t[1];
				z1_trans = r(2, 0)*x1 + r(2, 1)*y1 + r(2, 2)*z1 + t[2];
				x1 = x1_trans;
				y1 = y1_trans;
				z1 = z1_trans;

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
				geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
				geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
				geode->addDrawable(shapeDrawable.get());
			}

			osg::ref_ptr<osg::LineWidth> LineSize = new osg::LineWidth;
			LineSize->setWidth(m_fwidth);
			// turn off light render will make the color of line for better visualization
			geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
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

		osg::ref_ptr<osg::Geode> lineNode::getMergedGeoNode(const std::string &file1, const std::string &file2)
		{
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			osg::ref_ptr<osg::Geometry> geom_pt = new osg::Geometry();
			osg::ref_ptr<osg::Geometry> geom_line = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> coords_pt = new osg::Vec3Array();
			osg::ref_ptr<osg::Vec4Array> colors_pt = new osg::Vec4Array();
			osg::ref_ptr<osg::Vec3Array> coords_line = new osg::Vec3Array();
			osg::ref_ptr<osg::Vec4Array> colors_line = new osg::Vec4Array();

			Eigen::Matrix3f r = m_trans.block<3, 3>(0, 0);
			Eigen::Vector3f t = m_trans.topRightCorner<3, 1>();

			osg::Vec4 color_line;
			// file1 is supposed to be smaller size
			std::ifstream ifs1;
			ifs1.open(file1);
			ifs1 >> m_iNum;
			std::vector<float> x_vec1, y_vec1, z_vec1, x_vec2, y_vec2, z_vec2;
			while (!ifs1.eof())
			{
				float x1 = 0.f, y1 = 0.f, z1 = 0.f, x2 = 0.f, y2 = 0.f, z2 = 0.f;
				ifs1 >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;
				x_vec1.emplace_back(x1);
				y_vec1.emplace_back(y1);
				z_vec1.emplace_back(z1);
				x_vec2.emplace_back(x2);
				y_vec2.emplace_back(y2);
				z_vec2.emplace_back(z2);
			}
			ifs1.close();
			// file2 is supposed to be larger size
			int num_line = 0;
			std::ifstream ifs2;
			ifs2.open(file2);
			ifs2 >> m_iNum;

			while (!ifs2.eof())
			{
				float x1 = 0.f, y1 = 0.f, z1 = 0.f, x2 = 0.f, y2 = 0.f, z2 = 0.f;
				ifs2 >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;
				for (int i = 0; i < x_vec1.size(); ++i)
				{
					if (x_vec1[i] == x1 && y_vec1[i] == y1 && z_vec1[i] == z1 &&
						x_vec2[i] == x2 && y_vec2[i] == y2 && z_vec2[i] == z2)
					{
						color_line = osg::Vec4(0.f, 1.f, 0.f, 1.f);
						break;
					}
					else
					{
						color_line = osg::Vec4(0.5f, 0.5f, 0.5f, 1.f);
					}
				}

				x1 = x1 - m_offset.x;
				y1 = y1 - m_offset.y;
				z1 = z1 - m_offset.z;
				x2 = x2 - m_offset.x;
				y2 = y2 - m_offset.y;
				z2 = z2 - m_offset.z;

				// only src point are transformed
				float x1_trans = 0.f, y1_trans = 0.f, z1_trans = 0.f;
				x1_trans = r(0, 0)*x1 + r(0, 1)*y1 + r(0, 2)*z1 + t[0];
				y1_trans = r(1, 0)*x1 + r(1, 1)*y1 + r(1, 2)*z1 + t[1];
				z1_trans = r(2, 0)*x1 + r(2, 1)*y1 + r(2, 2)*z1 + t[2];
				x1 = x1_trans;
				y1 = y1_trans;
				z1 = z1_trans;

				coords_pt->push_back(osg::Vec3(x1, y1, z1));
				coords_pt->push_back(osg::Vec3(x2, y2, z2));
				colors_pt->push_back(m_v4ptColor);
				colors_pt->push_back(m_v4ptColor);

				coords_line->push_back(osg::Vec3(x1, y1, z1));
				coords_line->push_back(osg::Vec3(x2, y2, z2));
				colors_line->push_back(color_line);
				colors_line->push_back(color_line);
			}
			ifs2.close();

			coords_pt->pop_back();
			coords_pt->pop_back();
			colors_pt->pop_back();
			colors_pt->pop_back();
			coords_line->pop_back();
			coords_line->pop_back();
			colors_line->pop_back();
			colors_line->pop_back();

			for (int i = 0; i < 2 * m_iNum; ++i)
			{
				osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere((*coords_pt)[i], m_fradius);
				osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = new osg::ShapeDrawable(sphere.get());
				shapeDrawable->setColor((*colors_pt)[i]);
				geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
				geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
				geode->addDrawable(shapeDrawable.get());
			}

			osg::ref_ptr<osg::LineWidth> LineSize = new osg::LineWidth;
			LineSize->setWidth(m_fwidth);
			//geom_line->getOrCreateStateSet()->setAttributeAndModes(LineSize.get(), osg::StateAttribute::ON);
			// turn off light render will make the color of line for better visualization
			geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
			geom_line->setVertexArray(coords_line.get());
			geom_line->setColorArray(colors_line.get());
			geom_line->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
			for (int i = 0; i < m_iNum; ++i)
			{
				geom_line->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 2 * i, 2));
			}
			geode->addDrawable(geom_line.get());

			return geode;
		}

	}// namespace Node
} // namespace VisualTool