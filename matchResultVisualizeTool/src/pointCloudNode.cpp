#ifdef TEMPLATE_POINTCLOUD_NODE

#pragma once
#include "../include/pointCloudNode.h"
#include "../include/PointCloudIO.h"

namespace VisualTool {
	namespace Node {

		template<typename PointT>
		pointCloudNode<PointT>::pointCloudNode()
		{
			m_PointCloudPtr = boost::make_shared<pcl::PointCloud<PointT>>();
			m_offset = Point3d();
			m_pGeoNode = new osg::Geode();
		}

		template<typename PointT>
		pointCloudNode<PointT>::~pointCloudNode()
		{
			m_PointCloudPtr = NULL;
			m_offset = Point3d();
			m_pGeoNode = NULL;
		}

		template<typename PointT>
		pointCloudNode<PointT>::pointCloudNode(const std::string file_path, const osg::Vec4 node_color)
			:m_sFile_path(file_path),
			m_v4Color(node_color.r(), node_color.g(), node_color.b(), node_color.a())
		{
			m_PointCloudPtr = boost::make_shared<pcl::PointCloud<PointT>>();
			m_offset = Point3d();
			m_pGeoNode = new osg::Geode();
		}

		template<typename PointT>
		VisualTool::Offset pointCloudNode<PointT>::getOffset()
		{ 
			return m_offset; 
		}

		template<typename PointT>
		void pointCloudNode<PointT>::setAveOffset(const VisualTool::Offset &off)
		{
			m_offset = off;
		}

		template<typename PointT>
		osg::ref_ptr<osg::Geode> pointCloudNode<PointT>::getGeoNode()
		{
			VisualTool::Point3d offset_tmp;
			if (!PointIO::loadSingleLAS<PointT>(m_sFile_path, m_PointCloudPtr, offset_tmp))
			{
				LOG(WARNING) << "No point cloud is loaded";
				return new osg::Geode();
			}

			osg::ref_ptr<osg::Geode> geode = NULL;

			if (m_PointCloudPtr->points.size())
			{
				osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
				osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

				for (int i = 0; i < m_PointCloudPtr->points.size(); ++i)
				{// down sampling the origin point cloud
					if (i % 5 == 0)
					{
						auto pt = m_PointCloudPtr->points[i];
						// update node offset
						pt.x = pt.x + offset_tmp.x - m_offset.x;
						pt.y = pt.y + offset_tmp.y - m_offset.y;
						pt.z = pt.z + offset_tmp.z - m_offset.z;
						coords->push_back(osg::Vec3(pt.x, pt.y, pt.z));
						colors->push_back(m_v4Color);
					}
				}

				osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
				geometry->setVertexArray(coords.get());
				geometry->setColorArray(colors.get());
				geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
				geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, coords->size()));

				osg::ref_ptr<osg::StateSet> ss = geometry->getOrCreateStateSet();
				ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
				ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

				geode = new osg::Geode();
				geode->addDrawable(geometry.get());
				geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
			}
			return geode;
		}

	}// namespace Node
} // namespace VisualTool

#endif //TEMPLATE_POINTCLOUD_NODE