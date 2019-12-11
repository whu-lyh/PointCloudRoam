#pragma once

#include "stdafx.h"
#include <osg/Vec3f>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <vector>

#include <cmath>
#include <fstream>

namespace pmCommon{

    const int OBJECTCLASS_NUM = 16;

    enum OBJECTCLASS
    {
        NONE = -1,		//未启用类别
        UNKNOWN = 0,	//未知类别
        GROUND,			//地面;
        BUILDING,       //建筑物;
        UTILITYPOLE,    //电线杆;
        TRAFFICSIGN,    //交通标志牌;
        TREE,           //树;
        STREETLAMP,     //路灯;
        ENCLOSURE,      //围墙;
        CAR,            //汽车;
        ROAD,
        ROADMARKING,		//交通标线;	
        UNKNOWN_POLE,
        POWERLINE,
        CURB,
		BUSH,
		UNKNOWN_PLANE
    };

    const osg::Vec3f DEFAULT_CLASS_COLORS[] = {
        //未分类;
        { 0.5f, 0.5f, 0.5f },
        //地面
        { 0.94f, 0.89f, 0.69f },
        //建筑物;
        { 0.3f, 0.74f, 0.77f },
        //电线杆;
        { 0.92f, 0.81f, 0.0f },
        //标牌;
        { 0.90f, 0.15f, 0.1f },
        //树木;
        { 0.56f, 0.76f, 0.12f },
        //路灯;
        { 1.0f, 0.5f, 0.0f },
        //围墙;
        { 0.65f, 0.87f, 0.93f },
        //汽车;
        { 0.72f, 0.5f, 0.34f },
        //road
        { 0.0f, 0.0f, 1.0f },
        //ROADMARKING
        { 1.0f, 1.0f, 1.0f },
        //UNKNOWN_POLE
        { 0.0f, 0.0f, 0.0f },
        //POWERLINE
        { 1.0f, 0.68f, 0.79f },
        //CURB
        { 1.0f, 1.f, 1.f},
		//BUSH;
		{ 0.56f, 0.76f, 0.12f },
		//UNKNOWN_PLANE;
		{ 0.3f, 0.76f, 0.87f }
    };

    enum PointFormat    //move from osgPC, modification by zhao[2016.01.13]
    {
        pointFormat00 = 0x00000000,	//unknown
        pointFormat01 = 0x00000001,	//only x y z
        pointFormat02 = 0x00000003,	//x y z i
        pointFormat03 = 0x00000007,	//x y z i r g b
        pointFormat04 = 0x00000005,	//x y z r g b
        pointFormat05 = 0x00000009	//x y z c
    };

    struct Point3d
    {
        double	x;
        double	y;
        double	z;

        double reserve;

        Point3d()
        {
            x = y = z = 0.0;
            this->reserve = std::nan("");
        }

        Point3d(double x, double y, double z, double reserve = std::nan(""))
        {
            this->x = x;
            this->y = y;
            this->z = z;
            this->reserve = reserve;
        }

        virtual ~Point3d(){}

        virtual void output(std::ostream& out)
        {
            (static_cast<const Point3d*>(this))->output(out);
        }

        virtual void output(std::ostream& out) const
        {
            float fx = x;
            float fy = y;
            float fz = z;
            out.write(reinterpret_cast<const char*>(&fx), sizeof(float));
            out.write(reinterpret_cast<const char*>(&fy), sizeof(float));
            out.write(reinterpret_cast<const char*>(&fz), sizeof(float));
            out.write(reinterpret_cast<const char*>(&reserve), sizeof(reserve));
        }

        void relative(const Point3d& offset)
        {
            x -= (int)offset.x;
            y -= (int)offset.y;
            z -= (int)offset.z;
        }
    };

    struct IntensityPoint3d : Point3d
    {
        unsigned short i;

        IntensityPoint3d()
            : Point3d()
        {
            this->i = -1;
        }

        IntensityPoint3d(double x, double y, double z, unsigned short i, double reserve = std::nan(""))
            : Point3d(x, y, z, reserve)
        {
            this->i = i;
        }

        virtual ~IntensityPoint3d() {}

        virtual void output(std::ostream& out)
        {
            (static_cast<const IntensityPoint3d*>(this))->output(out);
        }

        virtual void output(std::ostream& out) const
        {
            Point3d::output(out);
            out.write(reinterpret_cast<const char*>(&i), sizeof(unsigned short));
        }
    };

    //! 带颜色的点云
    /*!
    带颜色而不带强度的点云类型
    *注意：没有将ColorPoint3d作为ColorIntensityPoint3d的基类，主要是避免多重继承
    *而且xyzirgb的格式继承自xyzi的格式也更为合理
    */
    struct ColorPoint3d : Point3d
    {
        unsigned short r;
        unsigned short g;
        unsigned short b;

        ColorPoint3d()
            : Point3d()
        {
            r = g = b = 255;
        }

        ColorPoint3d(double x, double y, double z, unsigned short r,
            unsigned short g, unsigned short b, double reserve = std::nan(""))
            : Point3d(x, y, z, reserve)
        {
            this->r = r;
            this->g = g;
            this->b = b;
        }

        virtual ~ColorPoint3d() {}

        virtual void output(std::ostream& out)
        {
            (static_cast<const ColorPoint3d*>(this))->output(out);
        }

        virtual void output(std::ostream& out) const
        {
            Point3d::output(out);
            out.write(reinterpret_cast<const char*>(&r), sizeof(unsigned short));
            out.write(reinterpret_cast<const char*>(&g), sizeof(unsigned short));
            out.write(reinterpret_cast<const char*>(&b), sizeof(unsigned short));
        }
    };

    struct ColorIntensityPoint3d : IntensityPoint3d
    {
        unsigned short r;
        unsigned short g;
        unsigned short b;

        ColorIntensityPoint3d()
            : IntensityPoint3d()
        {
            this->r = this->g = this->b = 255;
        }

        ColorIntensityPoint3d(double x, double y, double z, unsigned short i,
            unsigned short r, unsigned short g, unsigned short b, double reserve = std::nan(""))
            : IntensityPoint3d(x, y, z, i, reserve)
        {
            this->r = r;
            this->g = g;
            this->b = b;
        }

        virtual void output(std::ostream& out)
        {
            (static_cast<const ColorIntensityPoint3d*>(this))->output(out);
        }

        virtual void output(std::ostream& out) const
        {
            IntensityPoint3d::output(out);
            out.write(reinterpret_cast<const char*>(&r), sizeof(unsigned short));
            out.write(reinterpret_cast<const char*>(&g), sizeof(unsigned short));
            out.write(reinterpret_cast<const char*>(&b), sizeof(unsigned short));
        }
    };

    /**
    * @brief ClassPoint3d	: 具有类别的点云，类别由OBJECTCLASS枚举指定，并在显示时按照类别赋色
    */
    struct ClassPoint3d : public Point3d
    {
        OBJECTCLASS c;

    public:
        ClassPoint3d()
            : Point3d()
        {
            this->c = UNKNOWN;
        }

        ClassPoint3d(double x, double y, double z, OBJECTCLASS c, double reserve = std::nan(""))
            : Point3d(x, y, z, reserve)
        {
            this->c = c;
        }

        virtual ~ClassPoint3d() {}


        virtual void output(std::ostream& out)
        {
            (static_cast<const ClassPoint3d*>(this))->output(out);
        }

        virtual void output(std::ostream& out) const
        {
            Point3d::output(out);
            out.write(reinterpret_cast<const char*>(&c), sizeof(OBJECTCLASS));
        }
    };

    struct Point2d
    {
        double x;
        double y;

        void relative(const Point2d& offset)
        {
            x -= (int)offset.x;
            y -= (int)offset.y;
        }
    };

    struct Bounds
    {
        double minx;
        double miny;
        double maxx;
        double maxy;

        Bounds()
        {
            minx = miny = DBL_MAX;
            maxx = maxy = -DBL_MAX;
        }

        Bounds(double minx, double miny, double maxx, double maxy)
        {
            this->minx = minx;
            this->maxx = maxx;
            this->miny = miny;
            this->maxy = maxy;
        }

        Point2d center()
        {
            Point2d pt_center = { (minx + maxx) / 2, (miny + maxy) / 2 };
            return pt_center;
        }

        double width() const
        {
            return maxx - minx;
        }

        double height() const
        {
            return maxy - miny;
        }

        double area() const
        {
            return width() * height();
        }

        bool contains(const Point3d& pt) const
        {
            return (pt.x >= minx && pt.x <= maxx) && (pt.y >= miny && pt.y <= maxy);
        }

        bool contains(const Point2d& pt) const
        {
            return (pt.x >= minx && pt.x <= maxx) && (pt.y >= miny && pt.y <= maxy);
        }

        bool contains(double x, double y) const
        {
            return (x >= minx && x <= maxx) && (y >= miny && y <= maxy);
        }
    };

    struct Bounds3d
    {
        double minx;
        double miny;
        double minz;
        double maxx;
        double maxy;
        double maxz;

        Bounds3d()
        {
            reset();
        }

        Bounds3d(double minx, double miny, double minz, double maxx, double maxy, double maxz)
        {
            this->minx = minx;
            this->maxx = maxx;
            this->miny = miny;
            this->maxy = maxy;
            this->minz = minz;
            this->maxz = maxz;
        }

        void reset()
        {
            minx = miny = minz = DBL_MAX;
            maxx = maxy = maxz = -DBL_MAX;
        }

        Point3d center()
        {
            return Point3d((minx + maxx) / 2, (miny + maxy) / 2, (minz + maxz) / 2);
        }

        double length()
        {
            return maxx - minx;
        }

        double width()
        {
            return maxy - miny;
        }

        double height()
        {
            return maxz - minz;
        }

        void expand(const Point3d& pt)
        {
            if (pt.x < minx) minx = pt.x;
            if (pt.x > maxx) maxx = pt.x;
            if (pt.y < miny) miny = pt.y;
            if (pt.y > maxy) maxy = pt.y;
            if (pt.z < minz) minz = pt.z;
            if (pt.z > maxz) maxz = pt.z;
        }

        void expand(double x, double y, double z)
        {
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
            if (z < minz) minz = z;
            if (z > maxz) maxz = z;
        }

        void combine(const Bounds3d& bound)
        {
            if (minx > bound.minx) minx = bound.minx;
            if (maxx < bound.maxx) maxx = bound.maxx;
            if (miny > bound.miny) miny = bound.miny;
            if (maxy < bound.maxy) maxy = bound.maxy;
            if (minz > bound.minz) minz = bound.minz;
            if (maxz < bound.maxz) maxz = bound.maxz;
        }

        bool contains(const Point3d& pt)
        {
            return (pt.x >= (minx - 1e-4) && pt.x <= (maxx + 1e-4) && pt.y >= (miny - 1e-4) && pt.y <= (maxy + 1e-4) && pt.z >= (minz - 1e-4) && pt.z <= (maxz + 1e-4));
        }

        operator Bounds()
        {
            return Bounds(minx, miny, maxx, maxy);
        }
    };

    //非内联报错
    //extern OSG_EXPORT std::ostream& operator <<(std::ostream& fs, const osgPC::Point3d& pt);

	struct VoteScore
	{
		float angle1;//相邻两条线段的夹角;
		float angle2;//连线和该直线的夹角;
		float length;//该直线的长度;
		float pt_line_dis;//点的直线的距离（较大的值）;
		float pt_pt_dis;//相邻直线相邻端点的距离;
		int   scores;//得分;
		bool is_in_expel_zone;
		bool start_has_connected;
		bool end_has_connected;
		VoteScore()
		{
			scores = 0;//得分;
			is_in_expel_zone = false;
			start_has_connected = false;
			end_has_connected = false;
		}
		int line_id;
	};

	struct  PhysicalLine
	{
		pcl::ModelCoefficients::Ptr coefficients_line;
		pcl::PointXYZ start_pt;
		pcl::PointXYZ end_pt;
		float length;
		int line_id;
		VoteScore similarity;
		bool is_redundant;
		PhysicalLine()
		{
			is_redundant = false;
		}
	};
	typedef  std::vector<PhysicalLine, Eigen::aligned_allocator<PhysicalLine>>  VectorPhysicalLine;

	struct PolyLine
	{
		VectorPhysicalLine polyline_v;
		int polyline_id;
		pcl::PointCloud<pcl::PointXYZ>::Ptr sequence_points;
		double length;
		double dertax;
		double dertay;
	};
	typedef  std::vector<PolyLine, Eigen::aligned_allocator<PolyLine>>  VectorPolyLine;


	struct InterestingObject
	{
		int obj_id;
		OBJECTCLASS obj_sort;
		std::vector<int> point_id;
		std::vector<int> seg_id;
		float height;
		float length;
		float width;
		VectorPolyLine polylines_v;
		InterestingObject()
		{
			height = 0.0;
			width = 0.0;
			length = 0.0;
			obj_sort = NONE;
		}
		float min_z;
		pcl::PointXYZ min_z_pt;
		pcl::PointXYZ ground_pt;
		float max_z;
		float height_differece_to_ground;
		int relationship_with_road;

		float percentage_of_ball_points;
		float percentage_of_vertical_plane_points;
		float percentage_of_vertical_pole_points;
		float HeightDifferenceBeteewnGeometricalCenterAndBarycenter;
		float height_ground;
	};

	typedef  std::vector<InterestingObject, Eigen::aligned_allocator<InterestingObject>> VectorInterestingObject;

	struct project_info
	{
		bool	has_intensity;
		bool	has_texcolor;
		bool	has_class;
		bool	has_trajectroy;
		bool	has_panorama;
		int		max_depth;
		double	statistical_min_height;
		double	statistical_max_height;
		unsigned short	min_intensity;
		unsigned short	max_intensity;
		std::string	data_folder;
		std::string cache_folder;
		osg::Vec3d offset;
		Bounds3d bound;
		unsigned long long point_num;

		project_info()
		{
			has_texcolor = false;
			has_intensity = false;
			has_class = false;
			has_trajectroy = false;
			has_panorama = false;
			max_depth = 1;
			statistical_min_height = DBL_MAX;
			statistical_max_height = -DBL_MAX;
			point_num = 0;
		}
	};

}//end namespace osgPC
