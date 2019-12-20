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

    _viewer=new osgViewer::Viewer;
    _viewer->setThreadingModel(osgViewer::Viewer::DrawThreadPerContext);
    _comViewer=new osgViewer::CompositeViewer;
    {
        //_comViewer->setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);
        //_comViewer->setThreadingModel(osgViewer::CompositeViewer::DrawThreadPerContext);
        _comViewer->setThreadingModel(osgViewer::CompositeViewer::CullThreadPerCameraDrawThreadPerContext);
        //_comViewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
        QTimer::singleShot(10,this,SLOT(onStartTimer()));//不要即可启动定时器，否则窗体还未创建，容易帧循环时出错
    }

    osg::Camera* camera = _viewer->getCamera();
    camera->setGraphicsContext(_graphicsWindow);
    camera->setViewport(new osg::Viewport(0, 0, width(), height()));

    _comViewer->addView(_viewer);
    _comViewer->realize();//一定要在此实现，否则不能将qopenglcontext移动到图形线程
}

void MainWindow::onStartTimer()
{
    _timerID=startTimer(10);
}
