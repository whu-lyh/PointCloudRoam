#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif
#ifndef WIN32
#define WIN32
#endif
#include <windows.h>
#include <iostream>
#include <fstream>

#include <glog/logging.h>

#include "visualizeLibs.h"
#include "FileUtility.h"
#include "PointCloudIO.h"

namespace VF = VisualTool::FileUtility;
namespace VP = VisualTool::PointIO;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr      pcXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ>            pcXYZ;

// convert a PCL point cloud 2 osg node
// return: a osg::ref_ptr of osg node
template<typename T>
osg::ref_ptr<osg::Geode> pointCloud2OSG(const typename pcl::PointCloud<T>::Ptr& cloud,
	const osg::Vec4& color);

// convert match result to a osg node
// return: a osg::ref_ptr of osg node
// return 2: end point of correspondence
// return 3: line
osg::ref_ptr<osg::Geode> matchResult2OSG(const std::string& match_file,
	const osg::Vec4& color_pt1,
	const osg::Vec4& color_pt2,
	const osg::Vec4& color_line);

int main(int argc, char** argv)
{
	//if (argc != 2)
	//{
	//	std::cout << "Usage: test.exe Las_path(las1 & las2)" << std::endl;
	//	return -1;
	//}

	std::string las_path = argv[1];
	std::vector<std::string> las_files;
	VF::GetFiles(las_path, ".las", las_files);

	// load las files
	VisualTool::Point3d offset_src, offset_tar;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	VP::loadSingleLAS<pcl::PointXYZ>(las_files[0], cloud_src, offset_src);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	VP::loadSingleLAS<pcl::PointXYZ>(las_files[1], cloud_tar, offset_tar);

	// convert point to osg nodes
	osg::ref_ptr<osg::Geode> geode_src = pointCloud2OSG<pcl::PointXYZ>(cloud_src, osg::Vec4(1.f, 0.f, 0.f, 1.f));
	//osgDB::writeNodeFile(*geode_src, "./point_cloud_src.osg");
	osg::ref_ptr<osg::Geode> geode_tar = pointCloud2OSG<pcl::PointXYZ>(cloud_tar, osg::Vec4(0.f, 0.f, 1.f, 1.f));
	//osgDB::writeNodeFile(*geode_tar, "./point_cloud_tar.osg");
	osg::ref_ptr<osg::Group> root = new osg::Group;
	//osg::ref_ptr<osg::Geode> match_obj1 = createMatchObject("match1.txt", osg::Vec4(1.f, 0.f, 0.f, 1.f), osg::Vec4(0.f, 0.f, 1.f, 1.f), osg::Vec4(0.f, 1.f, 0.f, 1.f));
	osg::ref_ptr<osg::Geode> match_obj_erm = matchResult2OSG("edge_radius_match.txt", osg::Vec4(1.f, 0.f, 0.f, 1.f), osg::Vec4(0.f, 0.f, 1.f, 1.f), osg::Vec4(1.f, 1.f, 0.f, 1.f));
	//osgDB::writeNodeFile(*match_obj_erm, "./match_obj_erm.osg");

	root->addChild(geode_src);
	root->addChild(geode_tar);
	//root->addChild(match_obj1);
	root->addChild(match_obj_erm);

	//get the resolution of the window screen
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE) << "Error, no WindowSystemInterface available, cannot create windows." << std::endl;
		return 0;
	}

	unsigned int winwidth = 1920, winheight = 1280;
	osg::GraphicsContext::ScreenIdentifier main_screen_id;

	main_screen_id.readDISPLAY();
	main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
	wsi->getScreenResolution(main_screen_id, winwidth, winheight);

	osgViewer::Viewer viewer;
	viewer.getCamera()->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 1.f));
	viewer.setSceneData(root.get());
	viewer.home();

	//这里是单屏幕显示
	viewer.setUpViewOnSingleScreen(1);
	//viewer.apply(new osgViewer::SingleScreen(0));

	osgViewer::GraphicsWindow *pWnd = dynamic_cast<osgViewer::GraphicsWindow*>(viewer.getCamera()->getGraphicsContext());
	if (pWnd)
	{
		pWnd->setWindowDecoration(false);
	}

	return viewer.run();

	system("pause");
	return 0;
}

float calculateRMSE(const pcXYZPtr &pc_src, const pcXYZPtr &pc_tar, const Eigen::Matrix4f &src2tar, float& dx, float& dy, float& dz)
{
	dx = dy = dz = 0.f;
	if (pc_src == nullptr || pc_tar == nullptr)
	{
		LOG(ERROR) << "pctar / pcsrc is nullptr!";
		return 0.f;
	}

	if (pc_src->empty() || pc_tar->empty())
	{
		LOG(ERROR) << "pctar / pcsrc is empty!";
		return 0.f;
	}

	pcXYZPtr trans_pc_src(new pcXYZ());
	pcl::transformPointCloud(*pc_src, *trans_pc_src, src2tar);

	srand((int)time(0));
	int sample_N = 1000;
	sample_N = std::min(static_cast<int>(std::min(pc_src->size(), pc_src->size())), sample_N);
	int interval = int(pc_src->size()) / sample_N;
	std::set<int> sample_indices;

	//this sampling method might have some trouble
	for (int i = 0; i*interval < pc_src->size(); ++i)
	{
		sample_indices.insert(i*interval);
	}

	/*
	while (sample_indices.size() < sample_N)
	{
	int sample_tar_id = rand() % transformed_PcTar->size();
	sample_indices.insert(sample_tar_id);
	}*/

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pc_tar);
	int K = 10, count = 0;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	float max_radius = 0.01; //0.1m
	for (std::set<int>::iterator iter = sample_indices.begin(); iter != sample_indices.end(); ++iter)
	{
		const auto& trans_pt_src = trans_pc_src->points[*iter];
		if (kdtree.nearestKSearch(trans_pt_src, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			if (pointNKNSquaredDistance[0] > max_radius)
				continue;

			const auto& pt_src = pc_tar->points[pointIdxNKNSearch[0]];
			dx += (trans_pt_src.x - pt_src.x) * (trans_pt_src.x - pt_src.x);
			dy += (trans_pt_src.y - pt_src.y) * (trans_pt_src.y - pt_src.y);
			dz += (trans_pt_src.z - pt_src.z) * (trans_pt_src.z - pt_src.z);
			++count;
		}
	}
	pcXYZ().swap(*trans_pc_src);

	if (count == 0)
	{
		dx = dy = dz = 0.f;
		return 0.f;
	}

	float RMSE = std::sqrt((dx + dy + dz) / count);
	dx = std::sqrt(dx / count);
	dy = std::sqrt(dy / count);
	dz = std::sqrt(dz / count);

	return RMSE;
}

//float calculateRMSE(const Eigen::Matrix4d& T1w, const Eigen::Matrix4d& T2w, const SectionEdge& edge, float& dx, float& dy, float& dz)
//{
//	Eigen::Matrix3d R1w = T1w.topLeftCorner<3, 3>();
//	Eigen::Vector3d t1w = T1w.topRightCorner<3, 1>();
//
//	Eigen::Matrix3d R2w = T2w.topLeftCorner<3, 3>();
//	Eigen::Vector3d t2w = T2w.topRightCorner<3, 1>();
//
//	double squared_sum = 0.0;
//	dx = dy = dz = 0.0;
//	int count = 0;
//	for (const auto& fp : edge.link.feature_pairs)
//	{
//		Eigen::Vector3d p1 = R1w*fp.first.localSystem_.origin.cast<double>() + t1w;
//		Eigen::Vector3d p2 = R2w*fp.second.localSystem_.origin.cast<double>() + t2w;
//		Eigen::Vector3d delta_p = p1 - p2;
//		if (delta_p.norm() > 2 * edge.link.RMSE)
//			continue;
//		squared_sum += delta_p.squaredNorm();
//		dx += delta_p(0, 0) * delta_p(0, 0);
//		dy += delta_p(1, 0) * delta_p(1, 0);
//		dz += delta_p(2, 0) * delta_p(2, 0);
//		++count;
//	}
//
//	squared_sum = std::sqrt(squared_sum / count);
//	dx = std::sqrt(dx / count);
//	dy = std::sqrt(dy / count);
//	dz = std::sqrt(dz / count);
//
//	return squared_sum;
//}

template<typename T>
osg::ref_ptr<osg::Geode> pointCloud2OSG(const typename pcl::PointCloud<T>::Ptr& cloud, const osg::Vec4& color)
{
	osg::ref_ptr<osg::Geode> geode = NULL;

	if (cloud->points.size())
	{
		osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

		for (int i = 0; i < cloud->points.size(); ++i)
		{
			if (i % 5 == 0)
			{
				auto pt = cloud->points[i];
				coords->push_back(osg::Vec3(pt.x, pt.y, pt.z));
				colors->push_back(color);
			}
		}

		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
		geometry->setVertexArray(coords.get());
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,
			0, coords->size()));

		osg::ref_ptr<osg::StateSet> ss = geometry->getOrCreateStateSet();
		ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
		ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

		geode = new osg::Geode();
		geode->addDrawable(geometry.get());
		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF |
			osg::StateAttribute::OVERRIDE);
	}

	return geode;
}

osg::ref_ptr<osg::Geode> matchResult2OSG(const std::string& match_file, const osg::Vec4& color_pt1,
	const osg::Vec4& color_pt2, const osg::Vec4& color_line)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> geom_pt = new osg::Geometry();
	osg::ref_ptr<osg::Geometry> geom_line = new osg::Geometry();
	osg::ref_ptr<osg::Vec3Array> coords_pt = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors_pt = new osg::Vec4Array();
	osg::ref_ptr<osg::Vec3Array> coords_line = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors_line = new osg::Vec4Array();

	int num_line = 0;
	std::ifstream file;
	file.open(match_file);
	while (!file.eof())
	{
		++num_line;
		float x1, y1, z1, x2, y2, z2;
		file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;

		coords_pt->push_back(osg::Vec3(x1, y1, z1));
		coords_pt->push_back(osg::Vec3(x2, y2, z2));
		colors_pt->push_back(color_pt1);
		colors_pt->push_back(color_pt2);

		coords_line->push_back(osg::Vec3(x1, y1, z1));
		coords_line->push_back(osg::Vec3(x2, y2, z2));
		colors_line->push_back(color_line);
		colors_line->push_back(color_line);
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
		osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere((*coords_pt)[i], 0.1f);
		osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = new osg::ShapeDrawable(sphere.get());
		shapeDrawable->setColor((*colors_pt)[i]);
		geode->addDrawable(shapeDrawable.get());
	}

	osg::ref_ptr<osg::LineWidth> LineSize = new osg::LineWidth;
	LineSize->setWidth(0.1f);
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
