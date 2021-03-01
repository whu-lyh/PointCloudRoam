#include "mainwindow.h"
#include "shapewindow.h"
#include <QTimer>
#include <QApplication>

MainWindow::MainWindow()
{
	//glog setting
	FLAGS_logtostderr = true;
	google::FlushLogFiles(google::GLOG_INFO);
	google::InitGoogleLogging("PointCloudRoam");
	google::SetLogFilenameExtension("log_");
	FLAGS_log_dir = "./";
	FLAGS_logbufsecs = 0;
	//set the precision of the out data 4 accurate number behind dot
	LOG(INFO) << std::setiosflags(std::ios::fixed) << std::setprecision(4);

	//get the resolution of the window screen
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE) << "Error, no WindowSystemInterface available, cannot create windows." << std::endl;
		//return 1;
	}

	unsigned int winwidth = width(), winheight = height();
	osg::GraphicsContext::ScreenIdentifier main_screen_id;

	main_screen_id.readDISPLAY();
	main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
	wsi->getScreenResolution(main_screen_id, winwidth, winheight);

	osg::ref_ptr<osg::GraphicsContext::Traits> traits1 = new osg::GraphicsContext::Traits(osg::DisplaySettings::instance().get());
	traits1->width = 0.5*width();
	traits1->height = 0.5*height();
	traits1->windowDecoration = true;
	traits1->doubleBuffer = true;
	_graphicsWindoworigin = new MyGraphicWindowQt(traits1.get());
	_graphicsWindoworigin->setMainWindow(this);

	osg::ref_ptr<osg::GraphicsContext::Traits> traits2 = new osg::GraphicsContext::Traits(osg::DisplaySettings::instance().get());
	traits2->width = 0.5*width();
	traits2->height = 0.5*height();
	traits2->windowDecoration = true;//doesn't know what's difference
	traits2->doubleBuffer = true;
	_graphicsWindowrefine = new MyGraphicWindowQt(traits2.get());
	_graphicsWindowrefine->setMainWindow(this);

	QGridLayout* grid = new QGridLayout;
	grid->setMargin(0);//set the distance between widget and the boundary
	grid->setSpacing(10);//set the distance between the widget which distribute up and down
	grid->addWidget(_graphicsWindoworigin->getGLWidget(), 0, 0);
	grid->addWidget(_graphicsWindowrefine->getGLWidget(), 0, 1);//addWidget,控件名，行，列，占用行数，占用列数，对齐方式
	this->setLayout(grid);

	this->setWindowTitle(tr("Point Cloud Roamming"));
	connect(this, SIGNAL(activateShapewindow()), this, SLOT(showshape()));

	////initialize two viewer and all point will be added as a child
	_comViewer = new osgViewer::CompositeViewer();
	_comViewer->setName("window");

	_viewerorigin = new osgViewer::Viewer();
	_viewerorigin->setThreadingModel(osgViewer::Viewer::DrawThreadPerContext);
	_viewerorigin->setName("origin");
	_viewerrefine = new osgViewer::Viewer();
	_viewerrefine->setThreadingModel(osgViewer::Viewer::DrawThreadPerContext);
	_viewerrefine->setName("refine");

	{
		//_comViewer->setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);
		//OSG 将为每一个图形设备上下文（GraphicsContext）创建一个图形线程，以实现并行的渲染工作。
		//如果有多个 CPU 的话，那么系统将尝试把线程分别放在不同的 CPU上运行，不过每一帧结束前都会强制同步所有的线程。
		//_comViewer->setThreadingModel(osgViewer::CompositeViewer::DrawThreadPerContext);
		//这一线程模型同样会为每个 GraphicsContext 创建线程，并分配
		//到不同的 CPU 上。十分值得注意的是，这种模式会在当前帧的所有线程完成工作之前，开始下一帧。
		_comViewer->setThreadingModel(osgViewer::CompositeViewer::CullThreadPerCameraDrawThreadPerContext);
		//这一线程模型将为每个 GraphicsContext和每个摄像机创建线程，这种模式同样不会等待前一次的渲染结束，而是返回仿真循环并再
		//次开始执行 frame 函数。如果您使用四核甚至更高的系统配置，那么使用这一线程模型将最大限度地发挥多 CPU的处理能力。
		//_comViewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
		//OSG不会创建任何新线程来完成场景的筛选和渲染，因而也不会对渲染效率的提高有任何助益。它适合任何配置下使用。
		QTimer::singleShot(10, this, SLOT(onStartTimer()));//不要即可启动定时器，否则窗体还未创建，容易帧循环时出错
	}

	osg::ref_ptr<osg::Camera> cameraorigin = _viewerorigin->getCamera();
	cameraorigin->setGraphicsContext(_graphicsWindoworigin);
	if (this->getPerspectiveProjectStatus())
	{//explore more projection mode
		cameraorigin->setProjectionMatrixAsPerspective(60., 192.0 / 108, .1, 1000.);   //若zfar设成1000，打开大的数据会有问题
	}
	cameraorigin->setViewport(new osg::Viewport(0, 0, width(), height()));
	cameraorigin->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	cameraorigin->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 0));
	GLenum bufferorigin = traits1->doubleBuffer ? GL_BACK : GL_FRONT;
	cameraorigin->setDrawBuffer(bufferorigin);
	cameraorigin->setReadBuffer(bufferorigin);

	osg::ref_ptr<osg::Camera> camerarefine = _viewerrefine->getCamera();
	camerarefine->setGraphicsContext(_graphicsWindowrefine);
	if (this->getPerspectiveProjectStatus())
	{
		camerarefine->setProjectionMatrixAsPerspective(60., 192.0 / 108, .1, 1000.);//如果采用这种透视投影的方式将会使得视角中的物体近处比较稀疏
	}
	camerarefine->setViewport(new osg::Viewport(0, 0, width(), height()));
	camerarefine->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	camerarefine->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 0));
	GLenum bufferrefine = traits2->doubleBuffer ? GL_BACK : GL_FRONT;
	camerarefine->setDrawBuffer(bufferrefine);
	camerarefine->setReadBuffer(bufferrefine);

	//there are still many things that i don't know the usage and function
	// add the state manipulator
	_viewerorigin->addEventHandler(new osgGA::StateSetManipulator(_viewerorigin->getCamera()->getOrCreateStateSet()));
	// add the stats handler ///key S & W but meet a trouble
	_viewerorigin->addEventHandler(new osgViewer::StatsHandler);
	// add the record camera path handler
	_viewerorigin->addEventHandler(new osgViewer::RecordCameraPathHandler);
	// add the thread model handler
	_viewerorigin->addEventHandler(new osgViewer::ThreadingHandler);
	// add the window size toggle handler ///key F but meet a trouble
	_viewerorigin->addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler ///key S & W but meet a trouble
	_viewerrefine->addEventHandler(new osgViewer::StatsHandler);
	// add the record camera path handler
	_viewerrefine->addEventHandler(new osgViewer::RecordCameraPathHandler);
	// add the thread model handler
	_viewerrefine->addEventHandler(new osgViewer::ThreadingHandler);
	// add the window size toggle handler ///key F but meet a trouble
	_viewerrefine->addEventHandler(new osgViewer::WindowSizeHandler);

	_comViewer->addView(_viewerorigin);
	_comViewer->addView(_viewerrefine);
	_comViewer->realize();//一定要在此实现，否则不能将qopenglcontext移动到图形线程
}

//void waitASecound () {}
	//每隔10ms就调用frame()来绘制一帧，而这个定时器是在构造函数的时候就开始调用了，
	//没有等待QT中OpenGL环境的生成。在这里我把定时器的部分给改进了一下，
	//等待OSG的环境初始化完成在启动定时器，这个警告就没有了
void MainWindow::onStartTimer()
{
	QWidget::show();//Modefication for warning about "QOpenGLContext::swapBuffers() called with non-exposed window, behavior is undefined"
	_timerID = startTimer(10);
	//the up line is not working for error of "QObject::startTimer: Timers can only be used with threads started with QThread"
}

bool MainWindow::loadTraj(const std::string& traj_file, const osg::ref_ptr<osg::AnimationPath>& animation_path,
	const osg::Vec3d& offset)
{
	//load trajectory file
	osg::ref_ptr<osg::Vec3Array> route_pts = new osg::Vec3Array();
	std::ifstream ifs;
	ifs.open(traj_file);
	if (!ifs)
	{
		LOG(ERROR) << "The traj file open fails, Please check it. " << traj_file;
		return false;
	}
	int num_pts;
	ifs >> num_pts;
	int tmpi = 0;
	while (tmpi < num_pts)
	{
		double x, y, z;
		ifs >> x >> y >> z;

		if (typeid(y) != typeid(double) || typeid(x) != typeid(double) || typeid(z) != typeid(double) || !ifs)
		{
			LOG(ERROR) << "The traj file format is incorrect. Please check it" << std::endl;
			return false;
		}

		route_pts->push_back(osg::Vec3d(x, y, z));
		++tmpi;
	}
	ifs.close();

	//route_pts->pop_back();
	/*The up line is required while if the ending of the traj file have a space line which is empty but have a placeholder charactor
	the last line should be pop out or delete manully,
	I think this misrotation is caused due to the last number which are initiallized automatically to be zero and  further leading the rotation is disordered
	In the future, if the point cloud roamming work is needed ,the .traj file should be attached more attention and emphasized carefully
	*/

	float time = 0.f;
	float angle = 0.f;  //初始方向为Y轴正向（绕X轴旋转90°之后），之后就沿着z轴转
	float heading; //roam direction
	std::string direction = this->getViewPortDirection();
	if (direction == "y" || direction == "Y")
		heading = M_PI_2;  //绕x轴旋转90度, in anticlokcwise direction which is equal to y positive axis direction
	else if (direction == "-z" || direction == "-Z")
		heading = 0;  //绕x轴旋转90度, in anticlokcwise direction which is equal to y positive axis direction
	else if (direction == "y-down-45")
	{
		heading = M_PI_4;
	}
	else {
		//default direction
		heading = M_PI_2;  //绕x轴旋转90度, in anticlokcwise direction which is equal to y positive axis direction
		LOG(INFO) << "Using default direction along y axis";
	}

	/*
	The osg coordinate definition as left-hand coordinate which is right-front-up(x-y-z) and
	the OpenGL coordinate definition also as left-hand coordinate which is right-up-behind(x-y-z), z axis is pointer to outside the screen
	The default viewport is along down direction, so heading is rotate to be equal to  the y positive direction, rotate axis is x axis
	*/

	//The following should know the osg's coordinate's difinition
	for (auto iter = route_pts->begin(), end = route_pts->end(); iter + 1 != end;)
	{
		osg::Vec3 pos(*iter);
		iter++;
		if (iter->x() == pos.x())
		{
			angle = 0; //just be the same direction as default direction and need no more rotation
			if (iter->y() < pos.y())
				angle = M_PI; // if the y coordinate is decrease meaning that the object is backforward moving
		}
		else if (iter->x() > pos.x())
		{
			angle = M_PI_2 - std::atan((iter->y() - pos.y()) / (iter->x() - pos.x())); //calculate the angle to rotate around the z axis
		}
		else
		{
			angle = -(M_PI_2 + std::atan((iter->y() - pos.y()) / (iter->x() - pos.x()))); //negative direction
		}

		//here i need to learn more about quaternions to handle the rotation of the heading change while roamming
		//one quaternion is a rotation
		///***	
		///Q quaternion * P quaternion means that first conduct a rotation P,Second conduct a rotation Q, means a fusion about two rotations with a multiplication operation
		///***
		//current direction is opengl's direction so here should change the coordinate to the osg's coordinate
		osg::Quat rotation(osg::Quat(heading, osg::Vec3(1.f, 0.f, 0.f)) * osg::Quat(-angle, osg::Vec3(0.f, 0.f, 1.f))); //first rotate around z axis(0,0,1) with -angle degree, second rotate around x axis with heading degree
		//osg::Quat(angle to rotate, rotate around axis);
		//add each ControlPoint(a.k.a. viewpoint ==> position rotation scale) to the path
		//position ==> a 3D point osg::Vec3
		//rotation ==> a quaternion (osg::Quat) to control the view angle (a quaternion for 3d rotation is a multiplation conducted on 4D manifold) see more detail in bilibili
		//scale ==> default to be const osg::Vec3d& scale(1.0,1.0,1.0)
		animation_path->insert(time, osg::AnimationPath::ControlPoint(pos - offset, rotation));
		time += computeRunTime(pos, *iter);
	}
	return true;
}

float MainWindow::computeRunTime(osg::Vec3 start, osg::Vec3 end)
{
	float distance = std::sqrtf(std::powf(start.x() - end.x(), 2.f) + std::powf(start.y() - end.y(), 2.f)
		+ std::powf(start.z() - end.z(), 2.f));
	float speed_coef = this->getRoamSpeed(); //larger ==> slower
	return distance * speed_coef;
}

osg::ref_ptr<osg::Geode> MainWindow::loadPointCloud(const std::string& file_name, osg::Vec3d& offset)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	Point3d las_offset;
	Util::loadSingleLAS<pcl::PointXYZRGBA>(file_name, cloud, las_offset);
	offset = osg::Vec3d(las_offset.x, las_offset.y, las_offset.z);

	osg::ref_ptr<osg::Geode> geode = NULL;

	/*Note about color
	 * The RGBA information is available either as separate r, g, b, or as a
	* packed uint32_t rgba value. To pack it, use:
	*
	* \code
	* int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
	* \endcode
	*
	* To unpack it use:
	*
	* \code
	* int rgb = ...;
	* uint8_t r = (rgb >> 16) & 0x0000ff;
	* uint8_t g = (rgb >> 8)  & 0x0000ff;
	* uint8_t b = (rgb)     & 0x0000ff;
	* \endcode
	*/

	if (cloud->points.size())
	{
		osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

		for (auto pt : cloud->points)
		{
			coords->push_back(osg::Vec3(pt.x, pt.y, pt.z));
			//here note that the color should be between 0 and 1
			if (pt.r == 0.0 && pt.g == 0.0 && pt.b == 0.0 && pt.rgba != 0.0)
			{
				pt.r = (((int)pt.rgb) >> 16) & 0x0000ff6;
				pt.g = (((int)pt.rgb) >> 8) & 0x0000ff6;
				pt.b = ((int)pt.rgb) & 0x0000ff6;
				colors->push_back(osg::Vec4(pt.r, pt.g, pt.b, 1.f));
			}
			else
			{
				colors->push_back(osg::Vec4(pt.r / 255.0, pt.g / 255.0, pt.b / 255.0, 1.f));
			}
		}

		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
		geometry->setVertexArray(coords.get());
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,
			0, cloud->points.size()));

		osg::ref_ptr<osg::StateSet> ss = geometry->getOrCreateStateSet();
		ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
		ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

		geode = new osg::Geode();
		geode->addDrawable(geometry.get());
		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF |
			osg::StateAttribute::OVERRIDE);
		//geode->getOrCreateStateSet()->setAttribute(cloud->points.size(), osg::StateAttribute::ON);
	}
	return geode;
}

//创建路径
osg::ref_ptr<osg::AnimationPath> MainWindow::creatAnimationPath2(const osg::Vec3& start_pos, const osg::Vec3& end_pos,
	float start_angle, float end_angle, float start_time, float end_time)
{
	//创建一个Path对象
	osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath();
	//设置动画模式为循环（LOOP）
	animationPath->setLoopMode(osg::AnimationPath::NO_LOOPING);

	int num_ctrl_pts = 101;
	osg::Vec3 delta_pos((end_pos.x() - start_pos.x()) / (num_ctrl_pts - 1), (end_pos.y() - start_pos.y()) / (num_ctrl_pts - 1),
		(end_pos.z() - start_pos.z()) / (num_ctrl_pts - 1));
	float delta_time = (end_time - start_time) / (num_ctrl_pts - 1);
	float delta_angle = (end_angle - start_angle) / (num_ctrl_pts - 1);

	animationPath->insert(0.0, osg::AnimationPath::ControlPoint(osg::Vec3(), osg::Quat(osg::inDegrees(start_angle), osg::Z_AXIS)));
	animationPath->insert(fabs(start_time - delta_time), osg::AnimationPath::ControlPoint(start_pos, osg::Quat(osg::inDegrees(start_angle), osg::Z_AXIS)));
	for (int i = 0; i < num_ctrl_pts; ++i)
	{
		//关键点角度
		osg::Quat rotation(osg::inDegrees(start_angle + i * delta_angle), osg::Z_AXIS);
		//插入Path，把关键点与时间压入形成Path
		osg::Vec3 new_pos(start_pos.x() + i * delta_pos.x(), start_pos.y() + i * delta_pos.y(), start_pos.z() + i * delta_pos.z());
		animationPath->insert(start_time + i * delta_time, osg::AnimationPath::ControlPoint(new_pos, rotation));
	}
	animationPath->insert(end_time + delta_time, osg::AnimationPath::ControlPoint(osg::Vec3(), osg::Quat()));

	//返回Path
	return animationPath;
}

void MainWindow::showshape()
{
	_shapewindow = new ShapeWindow();

	//create a group to contains all SceneData
	osg::ref_ptr<osg::Group> rootreshape = new osg::Group();
	{
		osg::ref_ptr<osg::Node> nodeshape = osgDB::readNodeFile(this->getShapeFilepath());  //this registry using the osg plugin osgdb_shp.dll in PATH
		//for modefining some attribute of the point or line
		//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
		osg::StateSet* ssshape = rootreshape->getOrCreateStateSet();
		//set point's size
		//osg::ref_ptr<osg::Point> pointrefine = new osg::Point ();
		//pointrefine->setSize ( 20.0 );//didn't work
		//ssshape->setAttribute ( pointrefine );
		osg::Light *lightshape = new osg::Light;
		lightshape->setAmbient(osg::Vec4(0.5f, 0.5f, 0.5f, .25f));
		ssshape->setAttribute(lightshape, osg::StateAttribute::ON);
		ssshape->setMode(GL_BLEND, osg::StateAttribute::ON);
		ssshape->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		rootreshape->addChild(nodeshape);

		osgUtil::Optimizer optimizershape;
		optimizershape.optimize(rootreshape.get());
	}

	_shapewindow->setShapeSceneData(rootreshape.get());
	_shapewindow->setShapeCameraManipulator(new osgGA::TrackballManipulator);
	_shapewindow->show();
}