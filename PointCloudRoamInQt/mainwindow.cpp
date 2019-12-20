#include "mainwindow.h"
#include <QTimer>

MainWindow::MainWindow()
{
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(osg::DisplaySettings::instance().get());
    traits->width = width();
    traits->height = height();
    traits->doubleBuffer = true;
    _graphicsWindow = new MyGraphicWindowQt(traits.get());
    _graphicsWindow->setMainWindow(this);

    QGridLayout* grid = new QGridLayout;
    grid->setMargin(0);
    grid->addWidget(_graphicsWindow->getGLWidget(), 0, 0);
    setLayout(grid);

	this->setWindowTitle (tr("Point Cloud Roamming"));

	////initialize two viewer and all point will be added as a child
	_comViewer = new osgViewer::CompositeViewer ();
	//_comViewer->setName ("window");

	_viewerorigin = new osgViewer::Viewer();
	_viewerorigin->setThreadingModel (osgViewer::Viewer::DrawThreadPerContext);
	//_viewerorigin->setName ("_pointfilepathorigin");
	////_viewerrefine = new osgViewer::Viewer();
	////_viewerrefine->setThreadingModel (osgViewer::Viewer::DrawThreadPerContext);
	//
	{
	////_comViewer->setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);
	////	//_comViewer->setThreadingModel(osgViewer::CompositeViewer::DrawThreadPerContext);
		_comViewer->setThreadingModel (osgViewer::CompositeViewer::CullThreadPerCameraDrawThreadPerContext);
	//_comViewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
		QTimer::singleShot (10, this, SLOT (onStartTimer ()));//不要即可启动定时器，否则窗体还未创建，容易帧循环时出错
	}

	osg::Camera* camera = _viewerorigin->getCamera ();
	camera->setGraphicsContext (_graphicsWindow);
	camera->setViewport (new osg::Viewport (0, 0, width (), height ()));
	camera->setClearColor (osg::Vec4f (1, 1, 1, 1));

	_comViewer->addView (_viewerorigin);
	////_comViewer->addView (_viewerrefine);
	_comViewer->realize();//一定要在此实现，否则不能将qopenglcontext移动到图形线程
}

void MainWindow::onStartTimer()
{
    _timerID=startTimer(10);
}

void MainWindow::loadTraj (const std::string& traj_file, const osg::ref_ptr<osg::AnimationPath>& animation_path,
	const osg::Vec3d& offset)
{
	//load trajectory file
	osg::ref_ptr<osg::Vec3Array> route_pts = new osg::Vec3Array ();
	std::ifstream ifs;
	ifs.open (traj_file);
	int num_pts;
	ifs >> num_pts;
	while (!ifs.eof ())
	{
		double x, y, z;
		ifs >> x >> y >> z;
		route_pts->push_back (osg::Vec3d (x, y, z));
	}
	ifs.close ();
	//route_pts->pop_back();

	float time = 0.f;
	float angle = 0.f;  //绕Z轴旋转，初始方向为Y轴正向（绕X轴旋转90°之后）
	float roll = M_PI_2;
	for (auto iter = route_pts->begin (), end = route_pts->end (); iter + 1 != end;)
	{
		osg::Vec3 pos (*iter);
		iter++;
		if (iter->x () == pos.x ())
		{
			angle = 0;
			if (iter->y () < pos.y ())
				angle = M_PI;
		}
		else if (iter->x () > pos.x ())
		{
			angle = M_PI_2 - std::atan ((iter->y () - pos.y ()) / (iter->x () - pos.x ()));
		}
		else
		{
			angle = -(M_PI_2 + std::atan ((iter->y () - pos.y ()) / (iter->x () - pos.x ())));
		}

		osg::Quat rotation (osg::Quat (roll, osg::Vec3 (1.f, 0.f, 0.f)) * osg::Quat (-angle, osg::Vec3 (0.f, 0.f, 1.f)));
		animation_path->insert (time, osg::AnimationPath::ControlPoint (pos - offset, rotation));
		time += computeRunTime (pos, *iter);
	}
}

float MainWindow::computeRunTime (osg::Vec3 start, osg::Vec3 end)
{
	float distance = std::sqrtf (std::powf (start.x () - end.x (), 2.f) + std::powf (start.y () - end.y (), 2.f)
		+ std::powf (start.z () - end.z (), 2.f));
	float speed_coef = 0.25f;
	return distance * speed_coef;
}

osg::ref_ptr<osg::Geode> MainWindow::loadPointCloud (const std::string& file_name, osg::Vec3d& offset)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> ();
	Point3d las_offset;
	Util::loadSingleLAS<pcl::PointXYZRGB> (file_name, cloud, las_offset);
	offset = osg::Vec3d (las_offset.x, las_offset.y, las_offset.z);

	osg::ref_ptr<osg::Geode> geode = NULL;

	if (cloud->points.size ())
	{
		osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array ();
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array ();

		for (auto pt : cloud->points)
		{
			coords->push_back (osg::Vec3 (pt.x, pt.y, pt.z));
			//here note that the color should be between 0 and 1
			colors->push_back (osg::Vec4 (pt.r / 255.0, pt.g / 255.0, pt.b / 255.0, 1.f));
		}

		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry ();
		geometry->setVertexArray (coords.get ());
		geometry->setColorArray (colors.get ());
		geometry->setColorBinding (osg::Geometry::BIND_PER_VERTEX);
		geometry->addPrimitiveSet (new osg::DrawArrays (osg::PrimitiveSet::POINTS,
			0, cloud->points.size ()));

		osg::ref_ptr<osg::StateSet> ss = geometry->getOrCreateStateSet ();
		ss->setMode (GL_RESCALE_NORMAL, osg::StateAttribute::ON);
		ss->setMode (GL_NORMALIZE, osg::StateAttribute::ON);

		geode = new osg::Geode ();
		geode->addDrawable (geometry.get ());
		geode->getOrCreateStateSet ()->setMode (GL_LIGHTING, osg::StateAttribute::OFF |
			osg::StateAttribute::OVERRIDE);
		//geode->getOrCreateStateSet()->setAttribute(cloud->points.size(), osg::StateAttribute::ON);
	}
	return geode;
}

//创建路径
osg::ref_ptr<osg::AnimationPath> MainWindow::creatAnimationPath2 (const osg::Vec3& start_pos, const osg::Vec3& end_pos,
	float start_angle, float end_angle, float start_time, float end_time)
{
	//创建一个Path对象
	osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath ();
	//设置动画模式为循环（LOOP）
	animationPath->setLoopMode (osg::AnimationPath::NO_LOOPING);

	int num_ctrl_pts = 101;
	osg::Vec3 delta_pos ((end_pos.x () - start_pos.x ()) / (num_ctrl_pts - 1), (end_pos.y () - start_pos.y ()) / (num_ctrl_pts - 1),
		(end_pos.z () - start_pos.z ()) / (num_ctrl_pts - 1));
	float delta_time = (end_time - start_time) / (num_ctrl_pts - 1);
	float delta_angle = (end_angle - start_angle) / (num_ctrl_pts - 1);

	animationPath->insert (0.0, osg::AnimationPath::ControlPoint (osg::Vec3 (), osg::Quat (osg::inDegrees (start_angle), osg::Z_AXIS)));
	animationPath->insert (fabs (start_time - delta_time), osg::AnimationPath::ControlPoint (start_pos, osg::Quat (osg::inDegrees (start_angle), osg::Z_AXIS)));
	for (int i = 0; i < num_ctrl_pts; ++i)
	{
		//关键点角度
		osg::Quat rotation (osg::inDegrees (start_angle + i * delta_angle), osg::Z_AXIS);
		//插入Path，把关键点与时间压入形成Path
		osg::Vec3 new_pos (start_pos.x () + i * delta_pos.x (), start_pos.y () + i * delta_pos.y (), start_pos.z () + i * delta_pos.z ());
		animationPath->insert (start_time + i * delta_time, osg::AnimationPath::ControlPoint (new_pos, rotation));
	}
	animationPath->insert (end_time + delta_time, osg::AnimationPath::ControlPoint (osg::Vec3 (), osg::Quat ()));

	//返回Path
	return animationPath;
}

void MainWindow::createOriginView ()
{
	//get all point cloud file and return a list of point cloud file dir+name+ext
	std::vector<std::string> v_pointnamelistorigin;
	Util::get_files (getOriginPointFile (), ".las", v_pointnamelistorigin);

	//create a group to contains all SceneData
	osg::ref_ptr<osg::Group> root = new osg::Group ();
	//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
	osg::StateSet* ss = root->getOrCreateStateSet ();
	osg::Light *light = new osg::Light;
	light->setAmbient (osg::Vec4 (0.5f, 0.5f, 0.5f, .25f));
	ss->setAttribute (light, osg::StateAttribute::ON);
	ss->setMode (GL_BLEND, osg::StateAttribute::ON);
	ss->setRenderingHint (osg::StateSet::TRANSPARENT_BIN);

	// process all point, meanwhile add them in osg root for roamming all points
	for (auto pointfilename : v_pointnamelistorigin)
	{
		osg::Vec3d offseti;
		osg::ref_ptr<osg::Geode> modeli = loadPointCloud (pointfilename, offseti);
		modeli->getOrCreateStateSet ()->setAttribute (new osg::Point (1.5f), osg::StateAttribute::ON);
		//add modeli
		osg::ref_ptr<osg::MatrixTransform> mti = new osg::MatrixTransform ();
		mti->setMatrix (osg::Matrix::translate (offseti.x (), offseti.y (), offseti.z ()));
		mti->addChild (modeli);
		root->addChild (mti);
	}

	_viewerorigin->setThreadingModel (osgViewer::Viewer::DrawThreadPerContext);
	_viewerorigin->setName ("origin point cloud");
	//_comViewer->addView (_viewerorigin);
	_viewerorigin->setUpViewInWindow (100, 100, 1000, 563);

	//set animation path manipulator
	osg::ref_ptr<osgGA::AnimationPathManipulator> animation_path_manipulator = new osgGA::AnimationPathManipulator ();
	osg::ref_ptr<osg::AnimationPath> animation_path = new osg::AnimationPath ();
	animation_path->setLoopMode (osg::AnimationPath::LOOP);
	loadTraj (getTrajPointFile (), animation_path, osg::Vec3d ());
	animation_path_manipulator->setAnimationPath (animation_path);
	_viewerorigin->setCameraManipulator (animation_path_manipulator);

	osg::ref_ptr<osg::Camera> camera = _viewerorigin->getCamera ();    //3.2后不建议直接new camera
	camera->setName ("camera origin");
	camera->setProjectionMatrixAsPerspective (60., 192.0 / 108, .1, 1000.);   //若zfar设成1000，打开大的数据会有问题
	camera->setClearMask (GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	camera->setClearColor (osg::Vec4 (1.f, 1.f, 1.f, 0));

	//if you wanted to roam point cloud these line should be commented,theses lines are used for show point cloud and it's correspond color
	{
		//_viewerorigin->setCameraManipulator (new osgGA::TrackballManipulator);

		//// add the state manipulator
		//osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator;
		//statesetManipulator->setStateSet (_viewerorigin->getCamera ()->getOrCreateStateSet ());

		//_viewerorigin->addEventHandler (statesetManipulator.get ());
	}

	osgUtil::Optimizer optimizer;
	optimizer.optimize (root.get ());
	// set the scene to render
	_viewerorigin->setSceneData (root);
	_viewerorigin->home ();
	_viewerorigin->addEventHandler (new osgViewer::StatsHandler);

	osgViewer::GraphicsWindow *pWnd = dynamic_cast<osgViewer::GraphicsWindow*>(_viewerorigin->getCamera ()->getGraphicsContext ());
	if (pWnd)
	{
		pWnd->setWindowRectangle (100, 100, 1000, 563);
		pWnd->setWindowDecoration (true);
	}
}
void MainWindow::createRefineView ()
{
	//get all point cloud file and return a list of point cloud file dir+name+ext
	std::vector<std::string>  v_pointnamelistrefine;
	Util::get_files (getRefinePointFile (), ".las", v_pointnamelistrefine);

	//create a group to contains all SceneData
	osg::ref_ptr<osg::Group> root = new osg::Group ();
	//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
	osg::StateSet* ss = root->getOrCreateStateSet ();
	osg::Light *light = new osg::Light;
	light->setAmbient (osg::Vec4 (0.5f, 0.5f, 0.5f, .25f));
	ss->setAttribute (light, osg::StateAttribute::ON);
	ss->setMode (GL_BLEND, osg::StateAttribute::ON);
	ss->setRenderingHint (osg::StateSet::TRANSPARENT_BIN);

	// process all point, meanwhile add them in osg root for roamming all points
	for (auto pointfilename : v_pointnamelistrefine)
	{
		osg::Vec3d offseti;
		osg::ref_ptr<osg::Geode> modeli = loadPointCloud (pointfilename, offseti);
		modeli->getOrCreateStateSet ()->setAttribute (new osg::Point (1.5f), osg::StateAttribute::ON);
		//add modeli
		osg::ref_ptr<osg::MatrixTransform> mti = new osg::MatrixTransform ();
		mti->setMatrix (osg::Matrix::translate (offseti.x (), offseti.y (), offseti.z ()));
		mti->addChild (modeli);
		root->addChild (mti);
	}

	_viewerrefine->setName ("refine point cloud");
	//_comViewer->addView (_viewerrefine);
	_viewerrefine->setUpViewInWindow (1100, 100, 1000, 563);
	//set animation path manipulator
	osg::ref_ptr<osgGA::AnimationPathManipulator> animation_path_manipulator = new osgGA::AnimationPathManipulator ();
	osg::ref_ptr<osg::AnimationPath> animation_path = new osg::AnimationPath ();
	animation_path->setLoopMode (osg::AnimationPath::LOOP);
	loadTraj (getTrajPointFile (), animation_path, osg::Vec3d ());
	animation_path_manipulator->setAnimationPath (animation_path);
	_viewerrefine->setCameraManipulator (animation_path_manipulator);

	osg::ref_ptr<osg::Camera> camera = _viewerrefine->getCamera ();    //3.2后不建议直接new camera
	camera->setName ("camera refine");
	camera->setProjectionMatrixAsPerspective (60., 192.0 / 108, .1, 1000.);   //若zfar设成1000，打开大的数据会有问题
	camera->setClearMask (GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	camera->setClearColor (osg::Vec4 (1.f, 1.f, 1.f, 1.f));

	//if you wanted to roam point cloud these line should be commented,theses lines are used for show point cloud and it's correspond color
	{
		//these line correspond to the uper code line that are commented in viewer origin
		/*viewerrefine->setCameraManipulator (new osgGA::TrackballManipulator);

		viewerrefine->addEventHandler (new osgViewer::StatsHandler);*/
	}

	osgUtil::Optimizer optimizer;
	optimizer.optimize (root.get ());

	_viewerrefine->setSceneData (root);
	_viewerrefine->home ();
	_viewerrefine->addEventHandler (new osgViewer::StatsHandler);

	osgViewer::GraphicsWindow *pWnd = dynamic_cast<osgViewer::GraphicsWindow*>(_viewerrefine->getCamera ()->getGraphicsContext ());
	if (pWnd)
	{
		pWnd->setWindowRectangle (1100, 100, 1000, 563);
		pWnd->setWindowDecoration (true);
	}
}