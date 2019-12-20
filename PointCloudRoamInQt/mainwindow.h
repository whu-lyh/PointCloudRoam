#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "PointCloudRoamLibs.h"
#include "mygraphicwindowqt.h"
#include "Util.h"
#include "PointCloudIO.h"

class MainWindow : public QWidget {
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow(){
        int iii=0;
        killTimer(_timerID);
    }

    virtual void paintEvent(QPaintEvent* event)
    {
        //_comViewer->frame();
    }
    virtual void timerEvent(QTimerEvent* event)
    {
        _comViewer->frame();
    }

	void setSceneData (osg::Node* node)
	{
		_viewerorigin->setSceneData (node);
	}
	void setCameraManipulator (osgGA::CameraManipulator* manipulator, bool resetPosition = true)
	{
		_viewerorigin->setCameraManipulator (manipulator, resetPosition);
	}
	inline void setOriginPointFile (std::string unparsed) { _pointfilepathorigin = unparsed; }
	inline void setRefinePointFile (std::string unparsed) { _pointfilepathrefine = unparsed; }
	inline void setTrajFile (std::string unparsed) { _trajFile = unparsed; }

	inline std::string getOriginPointFile () { return _pointfilepathorigin; }
	inline std::string getRefinePointFile () { return _pointfilepathrefine; }
	inline std::string getTrajPointFile () { return _trajFile; }

	void loadTraj (const std::string& traj_file,
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

	void createOriginView ();
	void createRefineView ();

private:
	osgViewer::Viewer *_viewer, *_viewerrefine, *_viewerorigin;
	osgViewer::CompositeViewer *_comViewer;
    int               _timerID;
    MyGraphicWindowQt* _graphicsWindow;

	std::string _pointfilepathorigin;
	std::string _pointfilepathrefine;
	std::string _trajFile;

public slots:
    void onStartTimer();
};

#endif // MAINWINDOW_H
