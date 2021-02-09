/*
	Copyright (c) 2021 WuHan University. All Rights Reserved
	First create: 2021/02/08
	Detail: user keyboard handler for manipulating different match result
	Author: yuhao li
	Email:	yhaoli@whu.edu.cn
*/

#ifndef USERKEYBOARDHANDLER_H
#define USERKEYBOARDHANDLER_H

#pragma once

#include <vector>
#include <string>

#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgGA/GUIEventHandler>

namespace VisualTool {

class UserKeyBoardHandler:public osgGA::GUIEventHandler
{
public:

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
		if (!viewer)return false;

		switch (ea.getEventType())
		{
		case osgGA::GUIEventAdapter::KEYDOWN:
		{
			int num_nodes = viewer->getSceneData()->asSwitch()->getNumChildren();
			if (ea.getKey() == 0xFFBE && num_nodes > 2)
			{
				viewer->getSceneData()->asGroup()->getChild(2)->setNodeMask(0);
			}
			if (ea.getKey() == 0xFFBF && num_nodes > 3)
			{
				viewer->getSceneData()->asGroup()->getChild(3)->setNodeMask(0);
			}
			if (ea.getKey() == 0xFFC0 && num_nodes > 4)
			{
				viewer->getSceneData()->asGroup()->getChild(4)->setNodeMask(0);
			}
			if (ea.getKey() == 0xFFC1 && num_nodes > 5)
			{
				viewer->getSceneData()->asGroup()->getChild(5)->setNodeMask(0);
			}
			if (ea.getKey() == 0xFFC2 && num_nodes > 6)
			{
				viewer->getSceneData()->asGroup()->getChild(6)->setNodeMask(0);
			}
			if (ea.getKey() == 0xFFC3 && num_nodes > 7)
			{
				viewer->getSceneData()->asGroup()->getChild(7)->setNodeMask(0);
			}
			if (ea.getKey() == 0xFFC4 && num_nodes > 8)
			{
				viewer->getSceneData()->asGroup()->getChild(8)->setNodeMask(0);
			}
			if (ea.getKey() == 0xFFC5 && num_nodes > 9)
			{
				viewer->getSceneData()->asGroup()->getChild(9)->setNodeMask(0);
			}
			break;
		}
		case osgGA::GUIEventAdapter::PUSH: // mouse right single click
			if (ea.getButton() == 4) // OSG key code related to mouse push
			{// hide all nodes
				int num_nodes = viewer->getSceneData()->asSwitch()->getNumChildren();
				for (int i = 2; i < num_nodes; i++)
				{
					viewer->getSceneData()->asSwitch()->getChild(i)->setNodeMask(0);
				}
			}
			break;
		case osgGA::GUIEventAdapter::DOUBLECLICK: // mouse left double click
			if (ea.getButton() == 1) // OSG key code related to mouse DOUBLECLICK
			{// show all nodes
				int num_nodes = viewer->getSceneData()->asSwitch()->getNumChildren();
				for (int i = 2; i < num_nodes; i++)
				{
					viewer->getSceneData()->asSwitch()->getChild(i)->setNodeMask(1);
				}
			}
			break;
		default:
			break;
		}
		return false;
	}
};

} // namespace VisualTool

#endif //USERKEYBOARDHANDLER_H