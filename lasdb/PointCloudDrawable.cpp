#include "stdafx.h"
#include "PointCloudDrawable.h"

using namespace pmCommon;

PointCloudGeometry::PointCloudGeometry()
    : _intensities(nullptr), _points(nullptr), _texColors(nullptr), _assignColors(nullptr), _reserves(nullptr)
{
}

PointCloudGeometry::PointCloudGeometry(const PointCloudGeometry& pointcloud, const osg::CopyOp& copyop /*= osg::CopyOp::SHALLOW_COPY*/)
{
    _intensities = pointcloud._intensities;
    _points = pointcloud._points;
    _texColors = pointcloud._texColors;
    _bounds = pointcloud._bounds;
	_assignColors = pointcloud._assignColors;
}

void PointCloudGeometry::setPointCloud(osg::Vec3Array* points, const Bounds3d& bounds,
	osg::Vec3 offset, unsigned short* intensities /*= nullptr*/, osg::Vec3Array* texColors /*= nullptr*/, OBJECTCLASS* c, double* reserves)
{
    _points = points;
    _bounds = bounds;
    _texColors = texColors;
    _intensities = intensities;
    _offset = offset;
	_cs = c;
    _reserves = reserves;

    setVertexArray(_points);
    addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, _points->size()));

	if (_texColors != nullptr)
	{
		int count = _points->size();
		osg::ref_ptr<osg::Vec3Array> currentColors = new osg::Vec3Array(count);
		for (int i = 0; i < count; ++i)
		{
			(*currentColors)[i] = _texColors->at(i);
		}
		setColorArray(currentColors);
        setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

PointCloudGeometry::~PointCloudGeometry(void)
{
    if (_intensities != nullptr)
        delete _intensities;
	if (_cs != nullptr)
		delete _cs;
    if (_reserves != nullptr)
        delete _reserves;
}

void PointCloudGeometry::refreshDisplayMode(const pmCommon::ShareData& data)
{
	switch (data.displayMode)
	{
    case ShareData::HEIGHT:
        //setColorsByHeight(data);
		break;
    case ShareData::INTENSITY:
        //setColorsByIntensity(data);
        break;
    case ShareData::ASSIGNCOLOR:
        //setColorsByAssignColor(data);
        break;
    case ShareData::TEXTURECOLOR:
        setColorsByTexture(data);
        break;
	case ShareData::CLASS:
		//setColorByClass(data);
		break;
    default:
        return;
	}
}

void PointCloudGeometry::setColorsByTexture(const pmCommon::ShareData& data)
{
	//if (_texColors == nullptr)
	//	return;
	int count = _points->size();
	osg::ref_ptr<osg::Vec3Array> currentColors = new osg::Vec3Array(count);
    if (_texColors.valid())
    {
        for (int i = 0; i < count; ++i)
        {
            (*currentColors)[i] = _texColors->at(i);
        }
    }
    else
    {
        osg::Vec3 default_color(1.f, 1.f, 1.f);
        for (int i = 0; i < count; ++i)
        {
            (*currentColors)[i] = default_color;    //若没有纹理则默认白色
        }
    }
    setColorArray(currentColors);
	setColorBinding(osg::Geometry::BIND_PER_VERTEX);
}

void PointCloudGeometry::refreshSwitchMode(const pmCommon::ShareData& data)
{
	if (_cs == nullptr)
		return;
	removePrimitiveSet(0, this->getNumPrimitiveSets());
	const std::vector<bool>& sm = data.switchMode;
	osg::ref_ptr<osg::DrawElementsUInt> de = new osg::DrawElementsUInt(GL_POINTS);
	for (int i = 0, count = _points->size(); i < count; ++i)
	{
		if (sm[_cs[i]] == true)
		{
			de->push_back(i);
			/*switch(data.displayMode)
			{
			case ShareData::ASSIGNCOLOR:
			{
			colors->push_back(data.assignColor);
			break;
			}
			case ShareData::HEIGHT:
			case ShareData::INTENSITY:
			case ShareData::TEXTURECOLOR:
			{
			colors->push_back((*currentColors)[i]);
			break;
			}
			case ShareData::CLASS:
			{

			break;
			}
			}*/
		}
	}
	addPrimitiveSet(de);
}
