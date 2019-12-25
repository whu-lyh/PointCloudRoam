﻿/*
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

	//define the path of point cloud and trajectory
	///dataset shanghai lu
	//std::string pointfilepathorigin = "F:/shanghai/Lu-block-data/All-in-lu/ORIGIN";
	//std::string pointfilepathrefine = "F:/shanghai/Lu-block-data/All-in-lu/REFINE";
	//std::string traj_file = "F:/shanghai/Lu-block-data/All-in-lu/small-trajfile.traj";//osg roam only need a few of view points which is different from others

	///dataset wuhan cehuiyuan
	std::string pointfilepathorigin = "F:/Data/wuhan/origin";
	std::string pointfilepathrefine = "F:/Data/wuhan/refine";
	std::string traj_file = "F:/Data/wuhan/small-trajfile.traj";//osg roam only need a few of view points which is different from others

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
	}

	osg::ElapsedTime elapsedTime;

	//origin viewer
	//create a group to contains all SceneData
	osg::ref_ptr<osg::Group> rootorigin = new osg::Group ();
	{
		//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
		osg::StateSet* ssorigin = rootorigin->getOrCreateStateSet ();
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
		//root->getOrCreateStateSet()->setAttribute(new osg::Point(1.f), osg::StateAttribute::ON);
		osg::StateSet* ssrefine = rootrefine->getOrCreateStateSet ();
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