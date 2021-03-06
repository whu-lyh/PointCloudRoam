///*
//This program is used to roam point cloud in a viewer
//author: yhaoli@whu.edu.cn
//time:2019-12-13--9:38:01
//*/
//
//
//#define NOMINMAX
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
//
//
//using namespace std;
//
//struct Point3I
//{
//	uint8_t r;
//	uint8_t g;
//	uint8_t b;
//	Point3I(uint8_t r0 = 255, uint8_t g0 = 255, uint8_t b0 = 255)
//		: r(r0), g(g0), b(b0)
//	{}
//};
//
//float computeRunTime(osg::Vec3 start, osg::Vec3 end);
//
//void loadTraj(const std::string& traj_file, const osg::ref_ptr<osg::AnimationPath>& animation_path, const osg::Vec3d& offset);
//
//osg::ref_ptr<osg::Geode> loadPointCloud(const std::string& file_name, osg::Vec3d& offset);
//
//osg::ref_ptr<osg::AnimationPath> creatAnimationPath2(const osg::Vec3& start_pos, const osg::Vec3& end_pos,
//	float start_angle, float end_angle, float start_time, float end_time);
//
//int main()
//{
//	//define the path of point cloud and trajectory
//	std::string pointfilepath = "F:/shanghai/Lu-block-data/All-in-lu/REFINE";
//	std::string traj_file = "F:/shanghai/Lu-block-data/All-in-lu/small-trajfile.traj";//osg roam only need a few of view points which is different from others
//
//	//preprocess point cloud
//	/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
//	Util::Point3d offset;
//	Util::loadSingleLAS<pcl::PointXYZRGBA>(las1_path + "/pc1.las", cloud1, offset);
//
//	std::vector<bool> reserved(cloud1->size(), false);
//	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
//	std::vector<int> PointIdSearch_cloud;
//	std::vector<float> PointDistanceSearch_cloud;
//	kdtree.setInputCloud(cloud1);
//
//	for (int i = 0; i < cloud1->size(); ++i)
//	{
//	const auto& search_pt = cloud1->points[i];
//	kdtree.radiusSearch(search_pt, 0.15, PointIdSearch_cloud, PointDistanceSearch_cloud);
//	if (PointIdSearch_cloud.size() > 5)
//	reserved[i] = true;
//	}
//
//	float min_z, max_z;
//	min_z = std::numeric_limits<float>::max();
//	max_z = std::numeric_limits<float>::lowest();
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgb_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
//	for (int i = 0; i < reserved.size(); ++i)
//	{
//	const auto& pt = cloud1->points[i];
//	rgb_cloud->push_back(pt);
//
//	if (pt.z < min_z)
//	min_z = pt.z;
//	if (pt.z > max_z)
//	max_z = pt.z;
//	}
//	Util::saveLAS<pcl::PointXYZRGBA>(las1_path + "/pc1_texture.las", rgb_cloud, offset);
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
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr hgt_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
//	for (int i = 0; i < reserved.size(); ++i)
//	{
//	auto pt = cloud1->points[i];
//
//	Point3I color;
//	int index((pt.z - min_z) / sectionLength);
//	int maxIndex = endPointColors.size() - 1;
//	if (index >= maxIndex)
//	{
//	index = endPointColors.size() - 1;
//	color = endPointColors[index];
//	}
//	else if (index < 0)
//	{
//	index = 0;
//	color = endPointColors[index];
//	}
//	else
//	{
//	double remainder = (pt.z - min_z) - sectionLength * index;
//	double ratio = remainder / sectionLength;
//	color.r = (endPointColors[index + 1].r - endPointColors[index].r)*ratio + endPointColors[index].r;
//	color.g = (endPointColors[index + 1].g - endPointColors[index].g)*ratio + endPointColors[index].g;
//	color.b = (endPointColors[index + 1].b - endPointColors[index].b)*ratio + endPointColors[index].b;
//	}
//
//	pt.r = color.r;
//	pt.g = color.g;
//	pt.b = color.b;
//	hgt_cloud->push_back(pt);
//	}
//	Util::saveLAS<pcl::PointXYZRGBA>(las1_path + "/pc1_height.las", hgt_cloud, offset);
//	return 0;*/
//
//	//get all point cloud file and return a list of point cloud file dir+name+ext
//	std::vector<std::string> v_pointnamelist;
//	Util::get_files(pointfilepath,".las", v_pointnamelist);
//
//	//initialize viewer all point will be added as a child
//	osgViewer::Viewer viewer;
//	osg::ref_ptr<osg::Group> root = new osg::Group();
//	//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
//	osg::StateSet* ss = root->getOrCreateStateSet();
//	osg::Light *light = new osg::Light;
//	light->setAmbient(osg::Vec4(0.5f, 0.5f, 0.5f, .25f));
//	ss->setAttribute(light, osg::StateAttribute::ON);
//	ss->setMode(GL_BLEND, osg::StateAttribute::ON);
//	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
//
//	// process all point, meanwhile add them in osg root for roamming all points
//	for (auto pointfilename : v_pointnamelist)
//	{
//		osg::Vec3d offseti;
//		osg::ref_ptr<osg::Geode> modeli = loadPointCloud(pointfilename, offseti);
//		modeli->getOrCreateStateSet()->setAttribute(new osg::Point(1.5f), osg::StateAttribute::ON);
//		//add modeli
//		osg::ref_ptr<osg::MatrixTransform> mti = new osg::MatrixTransform();
//		mti->setMatrix(osg::Matrix::translate(offseti.x(), offseti.y(), offseti.z()));
//		mti->addChild(modeli);
//		root->addChild(mti);
//	}
//
//	//set animation path manipulator
//	osg::ref_ptr<osgGA::AnimationPathManipulator> animation_path_manipulator = new osgGA::AnimationPathManipulator();
//	osg::ref_ptr<osg::AnimationPath> animation_path = new osg::AnimationPath();
//	animation_path->setLoopMode(osg::AnimationPath::LOOP);
//	loadTraj(traj_file, animation_path, osg::Vec3d());
//	animation_path_manipulator->setAnimationPath(animation_path);
//	viewer.setCameraManipulator(animation_path_manipulator);
//
//	osg::ref_ptr<osg::Camera> camera = viewer.getCamera();    //3.2后不建议直接new camera
//	camera->setProjectionMatrixAsPerspective(60., 192.0 / 108, .1, 1000.);   //若zfar设成1000，打开大的数据会有问题
//	camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
//	camera->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 1.f));
//
//	osgUtil::Optimizer optimizer;
//	optimizer.optimize(root.get());
//
//	viewer.setSceneData(root);
//	viewer.home();
//
//	viewer.realize();
//
//	osgViewer::GraphicsWindow *pWnd = dynamic_cast<osgViewer::GraphicsWindow*>(viewer.getCamera()->getGraphicsContext());
//	if (pWnd)
//	{
//		//pWnd->setWindowRectangle(10, 10, 1600, 900);
//		pWnd->setWindowDecoration(true);
//	}
//
//	viewer.run();
//
//	return 0;
//}
//
//void loadTraj(const std::string& traj_file, const osg::ref_ptr<osg::AnimationPath>& animation_path,
//	const osg::Vec3d& offset)
//{
//	//load trajectory file
//	osg::ref_ptr<osg::Vec3Array> route_pts = new osg::Vec3Array();
//	std::ifstream ifs;
//	ifs.open(traj_file);
//	int num_pts;
//	ifs >> num_pts;
//	while (!ifs.eof())
//	{
//		double x, y, z;
//		ifs >> x >> y >> z;
//		route_pts->push_back(osg::Vec3d(x, y, z));
//	}
//	ifs.close();
//	//route_pts->pop_back();
//
//	float time = 0.f;
//	float angle = 0.f;  //绕Z轴旋转，初始方向为Y轴正向（绕X轴旋转90°之后）
//	float roll = M_PI_2;
//	for (auto iter = route_pts->begin(), end = route_pts->end(); iter + 1 != end;)
//	{
//		osg::Vec3 pos(*iter);
//		iter++;
//		if (iter->x() == pos.x())
//		{
//			angle = 0;
//			if (iter->y() < pos.y())
//				angle = M_PI;
//		}
//		else if (iter->x() > pos.x())
//		{
//			angle = M_PI_2 - std::atan((iter->y() - pos.y()) / (iter->x() - pos.x()));
//		}
//		else
//		{
//			angle = -(M_PI_2 + std::atan((iter->y() - pos.y()) / (iter->x() - pos.x())));
//		}
//
//		osg::Quat rotation(osg::Quat(roll, osg::Vec3(1.f, 0.f, 0.f)) * osg::Quat(-angle, osg::Vec3(0.f, 0.f, 1.f)));
//		animation_path->insert(time, osg::AnimationPath::ControlPoint(pos - offset, rotation));
//		time += computeRunTime(pos, *iter);
//	}
//}
//
//float computeRunTime(osg::Vec3 start, osg::Vec3 end)
//{
//	float distance = std::sqrtf(std::powf(start.x() - end.x(), 2.f) + std::powf(start.y() - end.y(), 2.f)
//		+ std::powf(start.z() - end.z(), 2.f));
//	float speed_coef = 0.25f;
//	return distance * speed_coef;
//}
//
//osg::ref_ptr<osg::Geode> loadPointCloud(const std::string& file_name, osg::Vec3d& offset)
//{
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
//	Point3d las_offset;
//	Util::loadSingleLAS<pcl::PointXYZRGBA>(file_name, cloud, las_offset);
//	offset = osg::Vec3d(las_offset.x, las_offset.y, las_offset.z);
//
//	osg::ref_ptr<osg::Geode> geode = NULL;
//
//	if (cloud->points.size())
//	{
//		osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
//		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
//
//		for (auto pt : cloud->points)
//		{
//			coords->push_back(osg::Vec3(pt.x, pt.y, pt.z));
//			//here note that the color should be between 0 and 1
//			colors->push_back(osg::Vec4(pt.r / 255.0, pt.g / 255.0, pt.b / 255.0, 1.f));
//		}
//
//		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
//		geometry->setVertexArray(coords.get());
//		geometry->setColorArray(colors.get());
//		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
//		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,
//			0, cloud->points.size()));
//
//		osg::ref_ptr<osg::StateSet> ss = geometry->getOrCreateStateSet();
//		ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
//		ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
//
//		geode = new osg::Geode();
//		geode->addDrawable(geometry.get());
//		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF |
//			osg::StateAttribute::OVERRIDE);
//		//geode->getOrCreateStateSet()->setAttribute(cloud->points.size(), osg::StateAttribute::ON);
//	}
//
//	return geode;
//}
//
////创建路径
//osg::ref_ptr<osg::AnimationPath> creatAnimationPath2(const osg::Vec3& start_pos, const osg::Vec3& end_pos,
//	float start_angle, float end_angle, float start_time, float end_time)
//{
//	//创建一个Path对象
//	osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath();
//	//设置动画模式为循环（LOOP）
//	animationPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
//
//	int num_ctrl_pts = 101;
//	osg::Vec3 delta_pos((end_pos.x() - start_pos.x()) / (num_ctrl_pts - 1), (end_pos.y() - start_pos.y()) / (num_ctrl_pts - 1),
//		(end_pos.z() - start_pos.z()) / (num_ctrl_pts - 1));
//	float delta_time = (end_time - start_time) / (num_ctrl_pts - 1);
//	float delta_angle = (end_angle - start_angle) / (num_ctrl_pts - 1);
//
//	animationPath->insert(0.0, osg::AnimationPath::ControlPoint(osg::Vec3(), osg::Quat(osg::inDegrees(start_angle), osg::Z_AXIS)));
//	animationPath->insert(fabs(start_time - delta_time), osg::AnimationPath::ControlPoint(start_pos, osg::Quat(osg::inDegrees(start_angle), osg::Z_AXIS)));
//	for (int i = 0; i < num_ctrl_pts; ++i)
//	{
//		//关键点角度
//		osg::Quat rotation(osg::inDegrees(start_angle + i*delta_angle), osg::Z_AXIS);
//		//插入Path，把关键点与时间压入形成Path
//		osg::Vec3 new_pos(start_pos.x() + i*delta_pos.x(), start_pos.y() + i*delta_pos.y(), start_pos.z() + i*delta_pos.z());
//		animationPath->insert(start_time + i*delta_time, osg::AnimationPath::ControlPoint(new_pos, rotation));
//	}
//	animationPath->insert(end_time + delta_time, osg::AnimationPath::ControlPoint(osg::Vec3(), osg::Quat()));
//
//	//返回Path
//	return animationPath;
//}
