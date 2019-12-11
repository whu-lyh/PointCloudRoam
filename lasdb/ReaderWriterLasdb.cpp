#include "StdAfx.h"
#include "ReaderWriterLasdb.h"

#include <cstdlib>
#include <cstdio>
#include <cmath>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/Array>
#include <osg/PrimitiveSet>
#include <osg/Switch>

#include <boost/filesystem.hpp>


ReaderWriterLasdb::ReaderWriterLasdb(void)
{
	supportsExtension("lasdb", "las cache file");
}


ReaderWriterLasdb::~ReaderWriterLasdb(void)
{
}

const char* ReaderWriterLasdb::className() const
{
	return "ReaderWriterLasdb";
}

const char* ReaderWriterLasdb::libraryName() const
{
#ifdef _DEBUG
	return "lasdbd";
#else
	return "lasdb";
#endif
}

osgDB::ReaderWriter::ReadResult ReaderWriterLasdb::readNode( const std::string& filename,const Options* opt ) const
{
	std::string ext = osgDB::getFileExtension(filename);
	if (!acceptsExtension(ext)) 
		return ReaderWriter::ReadResult::FILE_NOT_HANDLED;
	std::string file = osgDB::findDataFile(filename, opt);
	std::cout << "开始加载数据...file: "<< filename << std::endl;
	if (file.empty())
		return ReadResult::FILE_NOT_FOUND;

    FILE *fin = fopen(filename.c_str(), "rb");
    if (!feof(fin))
    {
        fclose(fin);
        osgDB::ReaderWriter::ReadResult rs = readFile(filename);
        boost::filesystem::path path(filename);
        rs.getNode()->setName(std::string("leafgeode-") + path.stem().string());
        return rs;
    }
	else
	{
        fclose(fin);
		return ReadResult::FILE_NOT_HANDLED;
	}
}

osgDB::ReaderWriter::ReadResult ReaderWriterLasdb::readNode( std::istream& ifs,const Options* /*=NULL */ ) const
{
	if (ifs)
	{
		osg::Node* node = readFile(ifs);
		if (node != nullptr)
			return node;
		return ReadResult::FILE_NOT_HANDLED;
	}
	else
		return ReadResult::FILE_NOT_FOUND;
}

osg::Node* ReaderWriterLasdb::readFile( std::istream& ifs ) const
{
    unsigned int point_ver = 0;
    ifs.read(reinterpret_cast<char*>(&point_ver), sizeof(point_ver));
    if (point_ver | 0x00)
    {
        return nullptr;
    }
	pmCommon::Bounds3d bounds;
    unsigned int point_num = 0;
    ifs.read(reinterpret_cast<char*>(&point_num), sizeof(point_num));
    osg::Vec3Array *vertices = new osg::Vec3Array;
    osg::Vec3 pt;
    unsigned short intensity;
    unsigned short r, g, b;
    pmCommon::OBJECTCLASS c;
    unsigned short *intensities = nullptr;
    if (point_ver & 0x02)
    {
        intensities = new unsigned short[point_num];
    }
    osg::Vec3Array *colors = nullptr;
    if (point_ver & 0x04)
    {
        colors = new osg::Vec3Array;
    }
    pmCommon::OBJECTCLASS *classes = nullptr;
    if (point_ver & 0x08)
    {
        classes = new pmCommon::OBJECTCLASS[point_num];
    }
    double *reserves = nullptr;
    double reserve = 0;
    bool has_reserve = false;
    ifs.read(reinterpret_cast<char*>(&pt._v), sizeof(pt._v));
    ifs.read(reinterpret_cast<char*>(&reserve), sizeof(reserve));
    if (!std::isnan(reserve))
    {
        has_reserve = true;
        reserves = new double[point_num];
    }
    ifs.seekg(-sizeof(pt._v) - sizeof(double), std::ios::cur);
    for (int j = 0; j < point_num; ++j)
    {
        ifs.read(reinterpret_cast<char*>(&pt._v), sizeof(pt._v));
        vertices->push_back(pt);
        bounds.expand(pt.x(), pt.y(), pt.z());
        ifs.read(reinterpret_cast<char*>(&reserve), sizeof(reserve));
        if (has_reserve)
            reserves[j] = reserve;
        if (point_ver & 0x02)
        {
            ifs.read(reinterpret_cast<char*>(&intensity), sizeof(unsigned short));
            intensities[j] = intensity;
        }
        if (point_ver & 0x04)
        {
            ifs.read(reinterpret_cast<char*>(&r), sizeof(unsigned short));
            ifs.read(reinterpret_cast<char*>(&g), sizeof(unsigned short));
            ifs.read(reinterpret_cast<char*>(&b), sizeof(unsigned short));
            colors->push_back(osg::Vec3(r / 255.f, g / 255.f, b / 255.f));
        }
        if (point_ver & 0x08)
        {
            ifs.read(reinterpret_cast<char*>(&c), sizeof(pmCommon::OBJECTCLASS));
            classes[j] = c;
        }
    }
    osg::Geode *geode = new osg::Geode;
    PointCloudGeometry *pcg = new PointCloudGeometry;
    pcg->setPointCloud(vertices, bounds, osg::Vec3(), intensities, colors, classes, reserves);
    geode->addDrawable(pcg);
    std::cout << "Load cache." << std::endl;
    return geode;
}

osg::Node* ReaderWriterLasdb::readFile(const std::string& filename) const
{
    FILE *fin = fopen(filename.c_str(), "rb");
    unsigned int point_ver = 0;
    fread(&point_ver, sizeof(point_ver), 1, fin);
    if (point_ver & 0x00)
    {
        return nullptr;
    }
	pmCommon::Bounds3d bounds;
    unsigned int point_num = 0;
    fread(&point_num, sizeof(point_num), 1, fin);
    osg::Vec3Array *vertices = new osg::Vec3Array;
    osg::Vec3 pt;
    unsigned short intensity;
    unsigned short r, g, b;
    pmCommon::OBJECTCLASS c;
    unsigned short *intensities = nullptr;
    if (point_ver & 0x02)
    {
        intensities = new unsigned short[point_num];
    }
    osg::Vec3Array *colors = nullptr;
    if (point_ver & 0x04)
    {
        colors = new osg::Vec3Array;
    }
    pmCommon::OBJECTCLASS *classes = nullptr;
    if (point_ver & 0x08)
    {
        classes = new pmCommon::OBJECTCLASS[point_num];
    }
    fread(&pt._v, sizeof(pt._v), 1, fin);
    double reserve = 0;
    fread(&reserve, sizeof(reserve), 1, fin);
    bool has_reserve = false;
    double *reserves = nullptr;
	//jianping [2016 1 22] 判断彩色是否是16位
    if (!std::isnan(reserve))//是一个数字
    {
        has_reserve = true;
        reserves = new double[point_num];
	}
	else
	{
		fseek(fin, -sizeof(double), SEEK_CUR);
	}
	if (intensities)//有强度
	{
		fread(&intensity, sizeof(intensity), 1, fin);
	}
	float color_index = 255;//转换成osg颜色系数
	if (colors)//有颜色
	{
		fread(&r, sizeof(r), 1, fin);
		fread(&g, sizeof(g), 1, fin);
		fread(&b, sizeof(b), 1, fin);
		if (r>255||g>255||b>255)
		{
			color_index = 255 * 255;
		}
	}
	//将文件指针偏移到起始点
	fseek(fin, -sizeof(pt._v), SEEK_CUR);
	if (has_reserve)
	{
		fseek(fin,-sizeof(double), SEEK_CUR);
	}
	if (intensities)
	{
		fseek(fin, -sizeof(intensity), SEEK_CUR);
	}
	if (colors)
	{
		fseek(fin, -sizeof(r)*3, SEEK_CUR);
	}
	
    for (int j = 0; j < point_num; ++j)
    {
        fread(&pt._v, sizeof(pt._v), 1, fin);
		vertices->push_back(pt);
        bounds.expand(pt.x(), pt.y(), pt.z());
        fread(&reserve, sizeof(double), 1, fin);
        if (has_reserve)
            reserves[j] = reserve;
        if (point_ver & 0x02)
        {
            fread(&intensity, sizeof(unsigned short), 1, fin);
            intensities[j] = intensity;
        }
        if (point_ver & 0x04)
        {
            fread(&r, sizeof(unsigned short), 1, fin);
            fread(&g, sizeof(unsigned short), 1, fin);
            fread(&b, sizeof(unsigned short), 1, fin);
			colors->push_back(osg::Vec3(r / color_index, g / color_index, b / color_index));
        }
        if (point_ver & 0x08)
        {
            fread(&c, sizeof(pmCommon::OBJECTCLASS), 1, fin);
            classes[j] = c;
        }
    }
    osg::Geode *geode = new osg::Geode;
    PointCloudGeometry *pcg = new PointCloudGeometry;
	/*osg::Vec3 offset;
	if (filename.find("pc1") != std::string::npos)
		offset = osg::Vec3();
	else if (filename.find("pc2") != std::string::npos)
		offset = osg::Vec3();*/
    pcg->setPointCloud(vertices, bounds, osg::Vec3(), intensities, colors, classes, reserves);
    geode->addDrawable(pcg);
    std::cout << "Load cache." << std::endl;
    fclose(fin);
    return geode;
}
