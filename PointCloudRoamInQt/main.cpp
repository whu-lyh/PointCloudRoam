/*
this program is used to roam point cloud in qt using multi thread method
and exploit the yaml file to locate the point cloud and trajectory file
*/
#pragma once
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>// this line have to be at the first line of the whole header files

#include <QApplication>

#include "mainwindow.h"
#include "shapewindow.h"
#include "Config.h"

struct DataInfo
{
	std::string  pointfilepathorigin;
	std::string  pointfilepathrefine;
	std::string traj_file;
};

struct RoamInfo
{
	std::string  viewportdirection;
	float speed;
	float doperspective;
};

struct ShapeInfo
{
	std::string doshowshape;
	std::string shapefilepath;
};

struct ConfigParameter
{
	DataInfo dataInfo;
	RoamInfo roaminfo;
	ShapeInfo shapeinfo;
};

//param setting
void parametersSetting ( ConfigParameter& config_param );

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

	//std::string config_file = argv [1];

	//std::string config_file = "F:/20190818_OldTown/20190818_OldTown.yaml";
	std::string config_file = "E:/Vs15WorkSpace/PointCloudRoam/PointCloudRoamInQt/Configuration.yaml";
	if ( !Util::Config::setParameterFile ( config_file ) )
	{
		LOG ( ERROR ) << "The configuration file is not existed! plz check it." << std::endl;
		return 0;
	}
		
	ConfigParameter config_param;
	parametersSetting ( config_param );

	// The qt window
	MainWindow widget;
	widget.setViewPortDirection ( config_param.roaminfo.viewportdirection );
	widget.setRoamSpeed ( config_param.roaminfo.speed );
	widget.setPerspectiveProjectStatus ( config_param.roaminfo.doperspective );
	//Shape window
	widget.setShapeStatus ( config_param.shapeinfo.doshowshape );
	if (widget.getShapeStatus())
	{
		widget.setShapeFilepath ( config_param.shapeinfo.shapefilepath );
		emit widget.activateShapewindow ();
	}

	//make sure that the dictories are exist
	if (Util::file_exist( config_param.dataInfo.pointfilepathorigin )&& Util::file_exist( config_param.dataInfo.pointfilepathrefine))
	{
		Util::ensure_dir ( config_param.dataInfo.pointfilepathorigin );
		Util::ensure_dir ( config_param.dataInfo.pointfilepathrefine );
	}

	//get all point cloud file and return a list of point cloud file dir+name+ext
	std::vector<std::string> v_pointnamelistorigin, v_pointnamelistrefine;
	Util::get_files ( config_param.dataInfo.pointfilepathorigin, ".las", v_pointnamelistorigin);
	Util::get_files ( config_param.dataInfo.pointfilepathrefine, ".las", v_pointnamelistrefine);
	if (v_pointnamelistorigin.size () == 0 || v_pointnamelistrefine.size () == 0)
	{
		LOG ( ERROR ) << "please check the directory of the input data" << std::endl;
		return 0;
	}

	osg::ElapsedTime elapsedTime;

	//origin viewer
	//create a group to contains all SceneData
	osg::ref_ptr<osg::Group> rootorigin = new osg::Group ();
	{
		//for modefining some attribute of the point or line
		//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
		osg::StateSet* ssorigin = rootorigin->getOrCreateStateSet ();
		//set point's size
		osg::ref_ptr<osg::Point> pointorigin = new osg::Point ();
		pointorigin->setSize (2);
		ssorigin->setAttribute (pointorigin);
		osg::Light *lightorigin = new osg::Light;
		lightorigin->setAmbient (osg::Vec4 (0.5f, 0.5f, 0.5f, .25f));
		ssorigin->setAttribute (lightorigin, osg::StateAttribute::ON);
		ssorigin->setMode (GL_BLEND, osg::StateAttribute::ON);
		ssorigin->setRenderingHint (osg::StateSet::TRANSPARENT_BIN);

		// process all point, meanwhile add them in osg root for roamming all points
		for (auto pointfilename : v_pointnamelistorigin)
		{
			osg::Vec3d offseti;
			osg::ref_ptr<osg::Geode> modeli = widget.loadPointCloud (pointfilename, offseti);
			modeli->getOrCreateStateSet ()->setAttribute (new osg::Point (1.5f), osg::StateAttribute::ON);
			//add modeli
			osg::ref_ptr<osg::MatrixTransform> mti = new osg::MatrixTransform ();
			mti->setMatrix (osg::Matrix::translate (offseti.x (), offseti.y (), offseti.z ()));
			mti->addChild (modeli);
			rootorigin->addChild (mti);
		}
	}

	//refine viewer
	//create a group to contains all SceneData
	osg::ref_ptr<osg::Group> rootrefine = new osg::Group ();
	{
		//for modefining some attribute of the point or line
		//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
		osg::StateSet* ssrefine = rootrefine->getOrCreateStateSet ();
		//set point's size
		osg::ref_ptr<osg::Point> pointrefine = new osg::Point ();
		pointrefine->setSize (20.0);//didn't work
		ssrefine->setAttribute (pointrefine);
		osg::Light *lightrefine = new osg::Light;
		lightrefine->setAmbient (osg::Vec4 (0.5f, 0.5f, 0.5f, .25f));
		ssrefine->setAttribute (lightrefine, osg::StateAttribute::ON);
		ssrefine->setMode (GL_BLEND, osg::StateAttribute::ON);
		ssrefine->setRenderingHint (osg::StateSet::TRANSPARENT_BIN);

		// process all point, meanwhile add them in osg root for roamming all points
		for (auto pointfilename : v_pointnamelistrefine)
		{
			osg::Vec3d offseti;
			osg::ref_ptr<osg::Geode> modeli = widget.loadPointCloud (pointfilename, offseti);
			modeli->getOrCreateStateSet ()->setAttribute (new osg::Point (1.5f), osg::StateAttribute::ON);
			//add modeli
			osg::ref_ptr<osg::MatrixTransform> mti = new osg::MatrixTransform ();
			mti->setMatrix (osg::Matrix::translate (offseti.x (), offseti.y (), offseti.z ()));
			mti->addChild (modeli);
			rootrefine->addChild (mti);
		}
	}

	double loadTime = elapsedTime.elapsedTime_m ();
	LOG ( INFO ) << "Load time " << loadTime << "ms";

	//set animation path manipulator
	osg::ref_ptr<osgGA::AnimationPathManipulator> animation_path_manipulator = new osgGA::AnimationPathManipulator ();
	osg::ref_ptr<osg::AnimationPath> animation_path = new osg::AnimationPath ();
	animation_path->setLoopMode (osg::AnimationPath::LOOP);
	if ( ! widget.loadTraj ( config_param.dataInfo.traj_file, animation_path, osg::Vec3d () ) )//middle point will interplated
	{
		LOG ( ERROR ) << "The traj file fails to be loaded";
		return 0;
	}
	animation_path_manipulator->setAnimationPath (animation_path);

	// optimize the scene graph, remove redundant nodes and state etc.
	osgUtil::Optimizer optimizer1;
	optimizer1.optimize (rootorigin.get ());

	osgUtil::Optimizer optimizer2;
	optimizer2.optimize (rootrefine.get ());

	widget.setSceneData (rootorigin.get (), rootrefine.get ());
	widget.setCameraManipulator (animation_path_manipulator, animation_path_manipulator);

	widget.setGeometry (100, 100, 2000, 618);
	widget.show();

    return app.exec();
}

void parametersSetting ( ConfigParameter& config_param )
{
	//Data info point cloud directories
	config_param.dataInfo.pointfilepathorigin = Util::Config::get<std::string> ( "PointFilepathOrigin" );
	config_param.dataInfo.pointfilepathrefine = Util::Config::get<std::string> ( "PointFilepathRefine" );
	config_param.dataInfo.traj_file = Util::Config::get<std::string> ( "TrajectoryFile" );

	//Roam info parameters
	config_param.roaminfo.viewportdirection = Util::Config::get<std::string> ( "ViewportDirection" );
	config_param.roaminfo.speed = Util::Config::get<float> ( "RoamSpeed" );
	config_param.roaminfo.doperspective = Util::Config::get<float> ( "OpenPerspective" );

	//Shape inro
	config_param.shapeinfo.doshowshape = Util::Config::get<std::string> ( "OpenShpWindow" );
	config_param.shapeinfo.shapefilepath = Util::Config::get<std::string> ( "ShapeFilepath" );

	//print params
	LOG ( INFO ) << "------------------------------Parameters------------------------------" << std::endl;
	LOG ( INFO ) << "#Data info" ;
	LOG ( INFO ) << "PointFilepath Origin: " << config_param.dataInfo.pointfilepathorigin;
	LOG ( INFO ) << "PointFilepath Refine: " << config_param.dataInfo.pointfilepathrefine;
	LOG ( INFO ) << "Trajectory File: " << config_param.dataInfo.traj_file;
	LOG ( INFO ) << "#Roam info";
	LOG ( INFO ) << "Roam Direction: " << config_param.roaminfo.viewportdirection;
	LOG ( INFO ) << "Roam Speed: " << config_param.roaminfo.speed;
	LOG ( INFO ) << "Open perspective projection: " << config_param.roaminfo.doperspective;
	LOG ( INFO ) << "#Shape info";
	LOG ( INFO ) << "Show shape file: " << config_param.shapeinfo.doshowshape;
	LOG ( INFO ) << "ShapeFilepath: " << config_param.shapeinfo.shapefilepath;
	LOG ( INFO ) << std::endl;
}
