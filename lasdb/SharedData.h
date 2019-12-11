#pragma once

#include "stdafx.h"
#include "utility.h"

#include <vector>
#include <osg/Vec3>
#include <osg/Array>

namespace pmCommon{

    struct ShareData
	{
		enum DisplayMode
		{
			ASSIGNCOLOR = 0,
			INTENSITY,
			TEXTURECOLOR,
			HEIGHT,
			CLASS
		};

		unsigned int	minIntensity;
		unsigned int	maxIntensity;
		DisplayMode		displayMode;
		osg::Vec3d		assignColor;
		double			minHeight;
		double			maxHeight;
		osg::Vec3d		offset;
		std::vector<osg::Vec3f>	classColorMap;
		std::vector<bool>		switchMode;

		ShareData()
		{
			displayMode = ASSIGNCOLOR;
			minHeight = 0;
			maxHeight = 0;
			minIntensity = 0;
			maxIntensity = 0;
			assignColor = osg::Vec3d(1., 1., 1.);
			offset = osg::Vec3d(0., 0., 0.);

			//default class color map and switch mode
            classColorMap.resize(OBJECTCLASS_NUM);
            std::copy(DEFAULT_CLASS_COLORS, DEFAULT_CLASS_COLORS + OBJECTCLASS_NUM, classColorMap.begin());
            switchMode.resize(OBJECTCLASS_NUM, false);
		}
	};

}