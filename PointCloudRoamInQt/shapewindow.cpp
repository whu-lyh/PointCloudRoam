#include "shapewindow.h"

ShapeWindow::ShapeWindow ()
{
	this->setWindowTitle ( "Trajectory Shape File" );

	_viewershape = new osgViewer::Viewer;

	_viewershape->setThreadingModel ( osgViewer::Viewer::CullThreadPerCameraDrawThreadPerContext );
	QTimer::singleShot ( 10, this, SLOT ( onStartTimer () ) );
	// disable the default setting of viewer.done() by pressing Escape.
	_viewershape->setKeyEventSetsDone ( 0 );
	_viewershape->setName ( "Shape window" );

	_graphicsWindowShape = this->createGraphicsWindow ( 900, 100, 320, 240, "Shape Window", true );

	osg::Camera* camerashape = _viewershape->getCamera ();
	camerashape->setGraphicsContext ( _graphicsWindowShape );
	const osg::GraphicsContext::Traits* traitsShp = _graphicsWindowShape->getTraits ();
	camerashape->setClearColor ( osg::Vec4 ( 1.f, 1.f, 1.f, 0 ) );
	camerashape->setClearMask ( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );

	GLenum bufferShape = traitsShp->doubleBuffer ? GL_BACK : GL_FRONT;
	camerashape->setDrawBuffer ( bufferShape );
	camerashape->setReadBuffer ( bufferShape );

	camerashape->setViewport ( new osg::Viewport ( 0, 0, traitsShp->width, traitsShp->height ) );
	camerashape->setProjectionMatrixAsPerspective ( 60.0f, static_cast<double>( traitsShp->width ) / static_cast<double>( traitsShp->height ), 1.0f, 1000.0f );
	_viewershape->addEventHandler ( new osgViewer::StatsHandler );

	_viewershape->realize ();

	connect ( &_timer, SIGNAL ( timeout () ), this, SLOT ( update () ) );
	_timer.start ( 15 );
}

MyGraphicWindowQt* ShapeWindow::createGraphicsWindow ( int x, int y, int w, int h, const std::string& name, bool windowDecoration )
{
	osg::DisplaySettings* ds = osg::DisplaySettings::instance ().get ();
	osg::ref_ptr<osg::GraphicsContext::Traits> traitsShp = new osg::GraphicsContext::Traits;
	traitsShp->windowName = name;
	traitsShp->windowDecoration = windowDecoration;
	traitsShp->x = x;
	traitsShp->y = y;
	traitsShp->width = w;
	traitsShp->height = h;
	traitsShp->doubleBuffer = true;
	traitsShp->alpha = ds->getMinimumNumAlphaBits ();
	traitsShp->stencil = ds->getMinimumNumStencilBits ();
	traitsShp->sampleBuffers = ds->getMultiSamples ();
	traitsShp->samples = ds->getNumMultiSamples ();
	return new MyGraphicWindowQt ( traitsShp.get () );
}

//每隔10ms就调用frame()来绘制一帧，而这个定时器是在构造函数的时候就开始调用了，
//没有等待QT中OpenGL环境的生成。在这里我把定时器的部分给改进了一下，
//等待OSG的环境初始化完成在启动定时器，这个警告就没有了
void ShapeWindow::onStartTimer ()
{
	QWidget::show ();//Modefication for warning about "QOpenGLContext::swapBuffers() called with non-exposed window, behavior is undefined"
	_timerID = startTimer ( 10 );
	//the up line is not working for error of "QObject::startTimer: Timers can only be used with threads started with QThread"
}
