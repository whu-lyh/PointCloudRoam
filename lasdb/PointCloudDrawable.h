#pragma once

#include <osg/Drawable>
#include <osg/RenderInfo>
#include <osg/Object>
#include <osg/BufferObject>
#include <osg/Geometry>

#include "utility.h"
#include "SharedData.h"

#ifdef LASDB_EXPORTS
#define LASDB_API __declspec(dllexport)
#else
#define LASDB_API __declspec(dllimport)
#endif

class LASDB_API PointCloudGeometry :
    public osg::Geometry
{
public:
    PointCloudGeometry();
    PointCloudGeometry(const PointCloudGeometry& pointcloud, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    void setPointCloud(osg::Vec3Array* points, const pmCommon::Bounds3d& bounds, osg::Vec3 offset,
        unsigned short* intensities = nullptr, osg::Vec3Array* texColors = nullptr, pmCommon::OBJECTCLASS* c = nullptr, double *reserves = nullptr);
    osg::Vec3Array* getPointCloud() const { return _points; }
    osg::Vec3Array* getPointCloudTexColors() const { return _texColors;  }
    void refreshDisplayMode(const pmCommon::ShareData& data);
    void refreshSwitchMode(const pmCommon::ShareData& data);
    osg::Vec3d	getOffset() const { return _offset; }
    pmCommon::Bounds3d	getBound() const { return _bounds; }
	unsigned short*	getIntensities() const { return _intensities; }
    pmCommon::OBJECTCLASS* getObjectClass() const { return _cs; }
    double* getReserves() const { return _reserves; }
	
protected:
    ~PointCloudGeometry(void);

private:
    void setColorsByTexture(const pmCommon::ShareData& data);
    osg::ref_ptr<osg::Vec3Array>    _points;
    osg::ref_ptr<osg::Vec3Array>    _texColors;
	osg::ref_ptr<osg::Vec3Array>	_assignColors;
    pmCommon::Bounds3d			    _bounds;
    unsigned short*		            _intensities;
    osg::Vec3                       _offset;
    pmCommon::OBJECTCLASS*			_cs;
    double*                         _reserves;
};
