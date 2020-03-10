#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "PointCloudRoamLibs.h"
#include "mygraphicwindowqt.h"
#include "Util.h"
#include <glog/logging.h>
#include "PointCloudIO.h"

class MainWindow : public QWidget {
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow(){
		google::ShutdownGoogleLogging ();
        killTimer(_timerID);
    }

    virtual void paintEvent(QPaintEvent* event)
    {
        //_comViewer->frame();
    }
	//timer event
    virtual void timerEvent(QTimerEvent* event)
    {
        _comViewer->frame();
    }

	void setSceneData (osg::Node* node)
	{
		_viewerorigin->setSceneData (node);
	}

	void setSceneData (osg::Node* nodeorigin, osg::Node* noderefine)
	{
		_viewerorigin->setSceneData (nodeorigin);
		_viewerrefine->setSceneData (noderefine);
	}

	void setCameraManipulator (osgGA::CameraManipulator* manipulator, bool resetPosition = true)
	{
		_viewerorigin->setCameraManipulator (manipulator, resetPosition);
	}

	void setCameraManipulator (osgGA::CameraManipulator* manipulatororigin, osgGA::CameraManipulator* manipulatorrefine, bool resetPositionorigin = true, bool resetPositionrefine = true)
	{
		_viewerorigin->setCameraManipulator (manipulatororigin, resetPositionorigin);
		_viewerrefine->setCameraManipulator (manipulatorrefine, resetPositionrefine);
	}

	bool loadTraj (const std::string& traj_file,
		const osg::ref_ptr<osg::AnimationPath>& animation_path,
		const osg::Vec3d& offset);

	osg::ref_ptr<osg::Geode> loadPointCloud (const std::string& file_name, osg::Vec3d& offset);

	float computeRunTime (osg::Vec3 start, osg::Vec3 end);

	//创建路径
	osg::ref_ptr<osg::AnimationPath> creatAnimationPath2 (const osg::Vec3& start_pos, 
		const osg::Vec3& end_pos,
		float start_angle, 
		float end_angle, 
		float start_time, 
		float end_time);

private:
	osgViewer::Viewer *_viewer, *_viewerrefine, *_viewerorigin;
	osgViewer::CompositeViewer *_comViewer;
    int               _timerID;
    MyGraphicWindowQt* _graphicsWindoworigin;
	MyGraphicWindowQt* _graphicsWindowrefine;

signals:
	void waitASecound ();

public slots:
    void onStartTimer();
};

#endif // MAINWINDOW_H
