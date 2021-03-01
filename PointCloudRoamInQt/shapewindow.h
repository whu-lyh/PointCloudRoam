#pragma once

#ifndef SHAPEWINDOW_H
#define SHAPEWINDOW_H

#include <QTimer>
#include "PointCloudRoamLibs.h"
#include "mygraphicwindowqt.h"

class ShapeWindow:public QWidget
{
	Q_OBJECT

public:
	ShapeWindow();

	~ShapeWindow() { killTimer(_timerID); }

	MyGraphicWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);

	void setShapeSceneData(osg::Node* nodeshape)
	{
		_viewershape->setSceneData(nodeshape);
	}

	void setShapeCameraManipulator(osgGA::CameraManipulator* manipulator, bool resetPosition = true)
	{
		_viewershape->setCameraManipulator(manipulator, resetPosition);
	}

	virtual void paintEvent(QPaintEvent* event)
	{
		_viewershape->frame();
	}

public slots:
	void onStartTimer();

protected:
	QTimer _timer;

private:
	int _timerID;
	osgViewer::Viewer *_viewershape;

	MyGraphicWindowQt* _graphicsWindowShape;
};
#endif // SHAPEWINDOW_H