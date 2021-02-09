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
			if (ea.getKey() == 0xFF51)
			{
				viewer->getSceneData()->asGroup()->getChild(1)->setNodeMask(0);
				viewer->getSceneData()->asGroup()->getChild(0)->setNodeMask(1);
			}
			if (ea.getKey() == 0xFF53)
			{
				viewer->getSceneData()->asGroup()->getChild(0)->setNodeMask(0);
				viewer->getSceneData()->asGroup()->getChild(1)->setNodeMask(1);
			}
			break;
		}
		case osgGA::GUIEventAdapter::PUSH:
			if (ea.getButton() == 4)
			{
				viewer->getSceneData()->asGroup()->getChild(0)->setNodeMask(0);
				viewer->getSceneData()->asGroup()->getChild(1)->setNodeMask(0);
			}
			break;
		case osgGA::GUIEventAdapter::DOUBLECLICK:
			if (ea.getButton() == 1)
			{
				viewer->getSceneData()->asGroup()->getChild(0)->setNodeMask(1);
				viewer->getSceneData()->asGroup()->getChild(1)->setNodeMask(1);
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