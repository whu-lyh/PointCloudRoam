/*
this program is used to roam point cloud in qt using multi thread method
*/
#pragma once

#define NOMINMAX

#include <windows.h>// this line have to be at the first line of the whole header files

#include <QApplication>

#include "mainwindow.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

	// The qt window
	MainWindow widget;

	//osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface ();
	//if (!wsi)
	//{
	//	osg::notify (osg::NOTICE) << "Error, no WindowSystemInterface available, cannot create windows." << std::endl;
	//	return 1;
	//}

	//unsigned int width, height;
	//osg::GraphicsContext::ScreenIdentifier main_screen_id;

	//main_screen_id.readDISPLAY ();
	//main_screen_id.setUndefinedScreenDetailsToDefaultScreen ();
	//wsi->getScreenResolution (main_screen_id, width, height);

	//define the path of point cloud and trajectory
	std::string pointfilepathorigin = "F:/shanghai/Lu-block-data/All-in-lu/ORIGIN";
	std::string pointfilepathrefine = "F:/shanghai/Lu-block-data/All-in-lu/REFINE";
	std::string traj_file = "F:/shanghai/Lu-block-data/All-in-lu/small-trajfile.traj";//osg roam only need a few of view points which is different from others

	widget.setOriginPointFile (pointfilepathorigin);
	widget.setRefinePointFile (pointfilepathrefine);
	widget.setTrajFile (traj_file);
	
	widget.createOriginView ();
	widget.createRefineView ();

	widget.show();

    return app.exec();
}