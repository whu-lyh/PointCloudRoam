/*
this program is used to roam point cloud in qt using multi thread method
and exploit the yaml file to locate the point cloud and trajectory file
*/
#pragma once
#define NOMINMAX

#include <windows.h>// this line have to be at the first line of the whole header files

#include <QApplication>

#include "mainwindow.h"
#include "Config.h"

struct DataInfo
{
	std::string  pointfilepathorigin;
	std::string  pointfilepathrefine;
	std::string traj_file;
};

struct ConfigParameter
{
	DataInfo dataInfo;
};

//param setting
void parametersSetting ( ConfigParameter& config_param );

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

	std::string config_file = argv [1];

	//std::string config_file = "F:/20190818_OldTown/20190818_OldTown.yaml";
	if ( !Util::Config::setParameterFile ( config_file ) )
	{
		std::cout << "The configuration file is not existed! plz check it." << std::endl;
		return -1;
	}
		
	ConfigParameter config_param;
	parametersSetting ( config_param );

	// The qt window
	MainWindow widget;

	 //define the path of point cloud and trajectory
	std::string pointfilepathorigin = Util::Config::get<std::string> ( "PointFilepathOrigin" );
	std::string pointfilepathrefine = Util::Config::get<std::string> ( "PointFilepathRefine" );
	std::string traj_file = Util::Config::get<std::string> ( "TrajectoryFile" );

	//make sure that the dictories are exist
	if (Util::file_exist(pointfilepathorigin)&& Util::file_exist(pointfilepathrefine))
	{
		Util::ensure_dir (pointfilepathorigin);
		Util::ensure_dir (pointfilepathrefine);
	}

	//get all point cloud file and return a list of point cloud file dir+name+ext
	std::vector<std::string> v_pointnamelistorigin, v_pointnamelistrefine;
	Util::get_files (pointfilepathorigin, ".las", v_pointnamelistorigin);
	Util::get_files (pointfilepathrefine, ".las", v_pointnamelistrefine);
	if (v_pointnamelistorigin.size () == 0 || v_pointnamelistrefine.size () == 0)
	{
		std::cout << "please check the directory of the input data" << std::endl;
		return -1;
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
	std::cout << "Load time " << loadTime << "ms" << std::endl;

	//set animation path manipulator
	osg::ref_ptr<osgGA::AnimationPathManipulator> animation_path_manipulator = new osgGA::AnimationPathManipulator ();
	osg::ref_ptr<osg::AnimationPath> animation_path = new osg::AnimationPath ();
	animation_path->setLoopMode (osg::AnimationPath::LOOP);
	widget.loadTraj (traj_file, animation_path, osg::Vec3d ());//middle point will interplated
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
	//Data info
	std::string str1 = Util::Config::get<std::string> ( "PointFilepathOrigin" );
	std::string str2 = Util::Config::get<std::string> ( "PointFilepathRefine" );
	std::string strTraj = Util::Config::get<std::string> ( "TrajectoryFile" );

	//point cloud directories
	config_param.dataInfo.pointfilepathorigin = Util::Config::get<std::string> ( "PointFilepathOrigin" );
	config_param.dataInfo.pointfilepathrefine = Util::Config::get<std::string> ( "PointFilepathRefine" );
	config_param.dataInfo.pointfilepathrefine = Util::Config::get<std::string> ( "TrajectoryFile" );

	//print params
	std::cout << "------------------------------Parameters------------------------------";
	std::cout << "#Data info" << std::endl;
	std::cout << "PointFilepathOrigin: " << str1 << std::endl;
	std::cout << "PointFilepathRefine: " << str2 << std::endl;
	std::cout << "Trajectory File: " << str2 << std::endl;
	std::cout << std::endl;
}
