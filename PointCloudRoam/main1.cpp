////范雯行道树提取测试代码。
//
//#define NOMINMAX
//#pragma once
//
//#include <windows.h>
//#include <iostream>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <osg/Point>
//#include <osgViewer/Viewer>
//#include <osgGA/AnimationPathManipulator>
//#include <osg/Light>
//#include <osg/Material>
//#include <osg/PagedLOD>
//#include <osg/MatrixTransform>
//#include <osgDB/Input>
//#include <osgGA/TrackballManipulator>
//#include <osg/PositionAttitudeTransform>
//#include <osgGA/OrbitManipulator>
//#include <osgUtil/Optimizer>
//#include <json/json.h>
//#include "Util.h"
//#include "PointCloudIO.h"
//#include "../lasdb/ReaderWriterLasdb.h"
//
//using namespace std;
//
//struct Point3I
//{
//	uint8_t r;
//	uint8_t g;
//	uint8_t b;
//	Point3I (uint8_t r0 = 255, uint8_t g0 = 255, uint8_t b0 = 255)
//		: r (r0), g (g0), b (b0)
//	{}
//};
//
//float computeRunTime (osg::Vec3 start, osg::Vec3 end);
//
//void loadTraj (const std::string& traj_file, const osg::ref_ptr<osg::AnimationPath>& animation_path, const osg::Vec3d& offset);
//
//osg::Group* loadModel (const std::string& prj_file, pmCommon::project_info& info);
//osg::ref_ptr<osg::Geode> loadPointCloud (const std::string& file_name, osg::Vec3d& offset);
//
//osg::ref_ptr<osg::AnimationPath> creatAnimationPath2 (const osg::Vec3& start_pos, const osg::Vec3& end_pos,
//	float start_angle, float end_angle, float start_time, float end_time);
//
//int main1 ()
//{
//	std::string traj_file = "../demo_data/785.traj";
//
//	//preprocess point cloud
//	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//	Util::Point3d offset;
//	Util::loadSingleLAS<pcl::PointXYZRGB>(las1_path + "/pc1.las", cloud1, offset);
//
//	std::vector<bool> reserved(cloud1->size(), false);
//	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//	std::vector<int> PointIdSearch_cloud;
//	std::vector<float> PointDistanceSearch_cloud;
//	kdtree.setInputCloud(cloud1);
//
//	for (int i = 0; i < cloud1->size(); ++i)
//	{
//		const auto& search_pt = cloud1->points[i];
//		kdtree.radiusSearch(search_pt, 0.15, PointIdSearch_cloud, PointDistanceSearch_cloud);
//		if (PointIdSearch_cloud.size() > 5)
//			reserved[i] = true;
//	}
//
//	float min_z, max_z;
//	min_z = std::numeric_limits<float>::max();
//	max_z = std::numeric_limits<float>::lowest();
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//	for (int i = 0; i < reserved.size(); ++i)
//	{
//		const auto& pt = cloud1->points[i];
//		rgb_cloud->push_back(pt);
//
//		if (pt.z < min_z)
//			min_z = pt.z;
//		if (pt.z > max_z)
//			max_z = pt.z;
//	}
//	Util::saveLAS<pcl::PointXYZRGB>(las1_path + "/pc1_texture.las", rgb_cloud, offset);
//
//	float delta_z = max_z - min_z;
//	std::vector<Point3I> endPointColors;
//	endPointColors.push_back(Point3I(0, 0, 255));
//	endPointColors.push_back(Point3I(0, 255, 255));
//	endPointColors.push_back(Point3I(0, 255, 0));
//	endPointColors.push_back(Point3I(255, 255, 0));
//	endPointColors.push_back(Point3I(255, 0, 0));
//	float sectionLength = delta_z / endPointColors.size();
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hgt_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//	for (int i = 0; i < reserved.size(); ++i)
//	{
//		auto pt = cloud1->points[i];
//
//		Point3I color;
//		int index((pt.z - min_z) / sectionLength);
//		int maxIndex = endPointColors.size() - 1;
//		if (index >= maxIndex)
//		{
//			index = endPointColors.size() - 1;
//			color = endPointColors[index];
//		}
//		else if (index < 0)
//		{
//			index = 0;
//			color = endPointColors[index];
//		}
//		else
//		{
//			double remainder = (pt.z - min_z) - sectionLength * index;
//			double ratio = remainder / sectionLength;
//			color.r = (endPointColors[index + 1].r - endPointColors[index].r)*ratio + endPointColors[index].r;
//			color.g = (endPointColors[index + 1].g - endPointColors[index].g)*ratio + endPointColors[index].g;
//			color.b = (endPointColors[index + 1].b - endPointColors[index].b)*ratio + endPointColors[index].b;
//		}
//
//		pt.r = color.r;
//		pt.g = color.g;
//		pt.b = color.b;
//		hgt_cloud->push_back(pt);
//	}
//	Util::saveLAS<pcl::PointXYZRGB>(las1_path + "/pc1_height.las", hgt_cloud, offset);
//	return 0;*/
//
//	//load point cloud 1
//	osg::Vec3d offset1, offset2;
//	std::string las_filename1 = "../demo_data/785out.las";
//	std::string las_filename2 = "../demo_data/785out_sim.las";
//	osg::ref_ptr<osg::Geode> model1 = loadPointCloud (las_filename1, offset1);
//	osg::ref_ptr<osg::Geode> model2 = loadPointCloud (las_filename2, offset2);
//	pmCommon::project_info info2;
//	//osg::Group* model2 = loadModel(las2_path + "/proj.qtx", info2);
//
//	model1->getOrCreateStateSet ()->setAttribute (new osg::Point (1.5f), osg::StateAttribute::ON);
//	model2->getOrCreateStateSet ()->setAttribute (new osg::Point (1.5f), osg::StateAttribute::ON);
//
//	//viewer
//	osgViewer::Viewer viewer;
//	osg::ref_ptr<osg::Group> root = new osg::Group ();
//	//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
//	osg::StateSet* ss = root->getOrCreateStateSet ();
//	osg::Light *light = new osg::Light;
//	light->setAmbient (osg::Vec4 (0.5f, 0.5f, 0.5f, .25f));
//	ss->setAttribute (light, osg::StateAttribute::ON);
//	ss->setMode (GL_BLEND, osg::StateAttribute::ON);
//	ss->setRenderingHint (osg::StateSet::TRANSPARENT_BIN);
//
//	//add model 1
//	osg::ref_ptr<osg::MatrixTransform> mt1 = new osg::MatrixTransform ();
//	mt1->setMatrix (osg::Matrix::translate (offset1.x (), offset1.y (), offset1.z ()));
//	mt1->addChild (model1);
//	root->addChild (mt1);
//	//load model 2
//	osg::ref_ptr<osg::MatrixTransform> mt2 = new osg::MatrixTransform ();
//	mt2->setMatrix (osg::Matrix::translate (offset2.x (), offset2.y (), offset2.z ()));
//	mt2->addChild (model2);
//	root->addChild (mt2);
//
//	//add model 2
//	//osg::ref_ptr<osg::MatrixTransform> mt2 = new osg::MatrixTransform();
//	//mt2->setMatrix(osg::Matrix::translate(info2.offset.x(), info2.offset.y(), info2.offset.z()));
//	//mt2->addChild(model2);
//	//root->addChild(mt2);
//	//
//	////set animation path for model 2
//	//osg::ref_ptr<osg::AnimationPath> animation_path2 = creatAnimationPath2(info2.offset + osg::Vec3(-1.0, 1.0, 1.0),
//	//	info2.offset, 3.f, 0.f, 30.f, 80.f);
//	//osg::ref_ptr<osg::AnimationPathCallback> apcb = new osg::AnimationPathCallback();
//	//apcb->setAnimationPath(animation_path2.get());
//	//mt2->setUpdateCallback(apcb.get());
//
//	//set animation path manipulator
//	osg::ref_ptr<osgGA::AnimationPathManipulator> animation_path_manipulator = new osgGA::AnimationPathManipulator ();
//	osg::ref_ptr<osg::AnimationPath> animation_path = new osg::AnimationPath ();
//	animation_path->setLoopMode (osg::AnimationPath::LOOP);
//	loadTraj (traj_file, animation_path, osg::Vec3d ());
//	animation_path_manipulator->setAnimationPath (animation_path);
//	viewer.setCameraManipulator (animation_path_manipulator);
//
//	osg::ref_ptr<osg::Camera> camera = viewer.getCamera ();    //3.2后不建议直接new camera
//	camera->setProjectionMatrixAsPerspective (60., 192.0 / 108, .1, 1000.);   //若zfar设成1000，打开大的数据会有问题
//	camera->setClearMask (GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
//	camera->setClearColor (osg::Vec4 (1.f, 1.f, 1.f, 1.f));
//
//	osgUtil::Optimizer optimizer;
//	optimizer.optimize (root.get ());
//
//	viewer.setSceneData (root);
//	viewer.home ();
//
//	viewer.realize ();
//
//	osgViewer::GraphicsWindow *pWnd = dynamic_cast<osgViewer::GraphicsWindow*>(viewer.getCamera ()->getGraphicsContext ());
//	if (pWnd)
//	{
//		//pWnd->setWindowRectangle(10, 10, 1600, 900);
//		pWnd->setWindowDecoration (true);
//	}
//
//	viewer.run ();
//
//	return 0;
//}
//
//void loadTraj (const std::string& traj_file, const osg::ref_ptr<osg::AnimationPath>& animation_path,
//	const osg::Vec3d& offset)
//{
//	//load trajectory file
//	osg::ref_ptr<osg::Vec3Array> route_pts = new osg::Vec3Array ();
//	std::ifstream ifs;
//	ifs.open (traj_file);
//	int num_pts;
//	ifs >> num_pts;
//	while (!ifs.eof ())
//	{
//		double x, y, z;
//		ifs >> x >> y >> z;
//		route_pts->push_back (osg::Vec3d (x, y, z));
//	}
//	ifs.close ();
//	//route_pts->pop_back();
//
//	float time = 0.f;
//	float angle = 0.f;  //绕Z轴旋转，初始方向为Y轴正向（绕X轴旋转90°之后）
//	float roll = M_PI_2;
//	for (auto iter = route_pts->begin (), end = route_pts->end (); iter + 1 != end;)
//	{
//		osg::Vec3 pos (*iter);
//		iter++;
//		if (iter->x () == pos.x ())
//		{
//			angle = 0;
//			if (iter->y () < pos.y ())
//				angle = M_PI;
//		}
//		else if (iter->x () > pos.x ())
//		{
//			angle = M_PI_2 - std::atan ((iter->y () - pos.y ()) / (iter->x () - pos.x ()));
//		}
//		else
//		{
//			angle = -(M_PI_2 + std::atan ((iter->y () - pos.y ()) / (iter->x () - pos.x ())));
//		}
//
//		osg::Quat rotation (osg::Quat (roll, osg::Vec3 (1.f, 0.f, 0.f)) * osg::Quat (-angle, osg::Vec3 (0.f, 0.f, 1.f)));
//		animation_path->insert (time, osg::AnimationPath::ControlPoint (pos - offset, rotation));
//		time += computeRunTime (pos, *iter);
//	}
//}
//
//float computeRunTime (osg::Vec3 start, osg::Vec3 end)
//{
//	float distance = std::sqrtf (std::powf (start.x () - end.x (), 2.f) + std::powf (start.y () - end.y (), 2.f)
//		+ std::powf (start.z () - end.z (), 2.f));
//	float speed_coef = 0.25f;
//	return distance * speed_coef;
//}
//
//osg::Group* loadModel (const std::string &prj_file, pmCommon::project_info& info)
//{
//	std::ifstream ifs;
//	ifs.open (prj_file);
//	Json::Reader reader;
//	Json::Value root;
//	if (!reader.parse (ifs, root, false))
//		return nullptr;
//	boost::filesystem::path fp (prj_file);
//	info.data_folder = fp.parent_path ().string ();
//	std::string cache_folder = root ["cache_folder"].asString ();
//	info.cache_folder = cache_folder;
//	unsigned int format = root ["pointFormat"].asUInt ();
//	if (format & 0x02)
//	{
//		info.has_intensity = true;
//		info.min_intensity = root ["minIntensity"].asUInt ();
//		info.max_intensity = root ["maxIntensity"].asUInt ();
//	}
//	if (format & 0x04)
//	{
//		info.has_texcolor = true;
//	}
//	if (format & 0x08)
//	{
//		info.has_class = true;
//	}
//	double minx = root ["minx"].asDouble ();
//	double miny = root ["miny"].asDouble ();
//	double minz = root ["minz"].asDouble ();
//	double maxx = root ["maxx"].asDouble ();
//	double maxy = root ["maxy"].asDouble ();
//	double maxz = root ["maxz"].asDouble ();
//	info.bound = pmCommon::Bounds3d (minx, miny, minz, maxx, maxy, maxz);
//	info.max_depth = root ["maxdepth"].asInt ();
//	info.point_num = root ["wholePointNum"].asUInt64 ();
//	osg::Vec3d offset (int ((minx + maxx) / 2.0), int ((miny + maxy) / 2.0), int ((minz + maxz) / 2.0));
//	info.offset = offset;
//	info.statistical_min_height = root ["statisticalMinz"].asDouble () - offset._v [2];
//	info.statistical_max_height = root ["statisticalMaxz"].asDouble () - offset._v [2];
//	info.has_trajectroy = root ["has_traj"].asBool ();
//	info.has_panorama = root ["has_pano"].asBool ();
//	ifs.close ();
//
//	osgDB::Registry::instance ()->addFileExtensionAlias ("qtxb", "osgb");
//	osgDB::Input input;
//	osg::Group *group = dynamic_cast<osg::Group*>(input.readNode (info.data_folder + "/" + cache_folder + "/-1.osgb"));
//	return group;
//}
//
//osg::ref_ptr<osg::Geode> loadPointCloud (const std::string& file_name, osg::Vec3d& offset)
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> ();
//	Point3d las_offset;
//	Util::loadSingleLAS<pcl::PointXYZRGB> (file_name, cloud, las_offset);
//	offset = osg::Vec3d (las_offset.x, las_offset.y, las_offset.z);
//
//	osg::ref_ptr<osg::Geode> geode = NULL;
//
//	if (cloud->points.size ())
//	{
//		osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array ();
//		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array ();
//
//		for (auto pt : cloud->points)
//		{
//			coords->push_back (osg::Vec3 (pt.x, pt.y, pt.z));
//			colors->push_back (osg::Vec4 (pt.r / 255.0, pt.g / 255.0, pt.b / 255.0, 1.f));
//		}
//
//		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry ();
//		geometry->setVertexArray (coords.get ());
//		geometry->setColorArray (colors.get ());
//		geometry->setColorBinding (osg::Geometry::BIND_PER_VERTEX);
//		/** Add a primitive set to the geometry. */
//		geometry->addPrimitiveSet (new osg::DrawArrays (osg::PrimitiveSet::POINTS,
//			0, cloud->points.size ()));
//
//		osg::ref_ptr<osg::StateSet> ss = geometry->getOrCreateStateSet ();
//		ss->setMode (GL_RESCALE_NORMAL, osg::StateAttribute::ON);
//		ss->setMode (GL_NORMALIZE, osg::StateAttribute::ON);
//
//		geode = new osg::Geode ();
//		geode->addDrawable (geometry.get ());
//		geode->getOrCreateStateSet ()->setMode (GL_LIGHTING, osg::StateAttribute::OFF |
//			osg::StateAttribute::OVERRIDE);
//		//geode->getOrCreateStateSet()->setAttribute(cloud->points.size(), osg::StateAttribute::ON);
//	}
//
//	return geode;
//}
//
////创建路径
//osg::ref_ptr<osg::AnimationPath> creatAnimationPath2 (const osg::Vec3& start_pos, const osg::Vec3& end_pos,
//	float start_angle, float end_angle, float start_time, float end_time)
//{
//	//创建一个Path对象
//	osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath ();
//	//设置动画模式为循环（LOOP）
//	animationPath->setLoopMode (osg::AnimationPath::NO_LOOPING);
//
//	int num_ctrl_pts = 101;
//	osg::Vec3 delta_pos ((end_pos.x () - start_pos.x ()) / (num_ctrl_pts - 1), (end_pos.y () - start_pos.y ()) / (num_ctrl_pts - 1),
//		(end_pos.z () - start_pos.z ()) / (num_ctrl_pts - 1));
//	float delta_time = (end_time - start_time) / (num_ctrl_pts - 1);
//	float delta_angle = (end_angle - start_angle) / (num_ctrl_pts - 1);
//
//	animationPath->insert (0.0, osg::AnimationPath::ControlPoint (osg::Vec3 (), osg::Quat (osg::inDegrees (start_angle), osg::Z_AXIS)));
//	animationPath->insert (fabs (start_time - delta_time), osg::AnimationPath::ControlPoint (start_pos, osg::Quat (osg::inDegrees (start_angle), osg::Z_AXIS)));
//	for (int i = 0; i < num_ctrl_pts; ++i)
//	{
//		//关键点角度
//		osg::Quat rotation (osg::inDegrees (start_angle + i * delta_angle), osg::Z_AXIS);
//		//插入Path，把关键点与时间压入形成Path
//		osg::Vec3 new_pos (start_pos.x () + i * delta_pos.x (), start_pos.y () + i * delta_pos.y (), start_pos.z () + i * delta_pos.z ());
//		animationPath->insert (start_time + i * delta_time, osg::AnimationPath::ControlPoint (new_pos, rotation));
//	}
//	animationPath->insert (end_time + delta_time, osg::AnimationPath::ControlPoint (osg::Vec3 (), osg::Quat ()));
//
//	//返回Path
//	return animationPath;
//}
