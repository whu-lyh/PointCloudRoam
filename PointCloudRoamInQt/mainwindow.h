﻿#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "PointCloudRoamLibs.h"
#include "mygraphicwindowqt.h"
#include "shapewindow.h"
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

	inline std::string getViewPortDirection () { return _viewportdirection; }
	inline void setViewPortDirection ( std::string direction ) { _viewportdirection = direction; }

	inline float getRoamSpeed () { return _roamspeed; }
	inline void setRoamSpeed ( float speed ) { _roamspeed = speed; }

	inline float getPerspectiveProjectStatus () { return _perspective; }
	inline void setPerspectiveProjectStatus ( float openperspective ) { _perspective = openperspective; }

	inline bool getShapeStatus () { return ( ( _openShapeWindow == "true" ) || ( _openShapeWindow == "TRUE" ) || ( _openShapeWindow == "True" ) ) ? true : false; }
	inline void setShapeStatus ( std::string openshape ) { _openShapeWindow = openshape; }

	inline void setShapeFilepath ( std::string path ) { _shapeFilepath = path; }
	inline std::string getShapeFilepath () { return _shapeFilepath; }

private:
	osgViewer::Viewer *_viewer, *_viewerrefine, *_viewerorigin;
	osgViewer::CompositeViewer *_comViewer;
    int               _timerID;
    MyGraphicWindowQt* _graphicsWindoworigin;
	MyGraphicWindowQt* _graphicsWindowrefine;

	std::string _viewportdirection;
	float _roamspeed = 0.25;
	float _perspective = 0.f;

	std::string _openShapeWindow;
	std::string _shapeFilepath;
	ShapeWindow *_shapewindow;

signals:
	void waitASecound ();
	void activateShapewindow ();

public slots:
    void onStartTimer();
	void showshape ();
};

#endif // MAINWINDOW_H
