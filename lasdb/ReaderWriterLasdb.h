#pragma once

#ifdef LASDB_EXPORTS
#define LASDB_API __declspec(dllexport)
#else
#define LASDB_API __declspec(dllimport)
#endif

#include <osgDB/ReaderWriter>
#include <osgDB/Registry>

#include "PointCloudDrawable.h"
#include "utility.h"

class LASDB_API ReaderWriterLasdb :
	public osgDB::ReaderWriter
{
public:
	ReaderWriterLasdb(void);
	~ReaderWriterLasdb(void);

	virtual const char* className() const;

	virtual const char* libraryName() const;

	virtual ReadResult readNode( const std::string& ,const Options* =NULL ) const;

	virtual ReadResult readNode( std::istream& ,const Options* =NULL ) const;

private:
	osg::Node*	readFile(std::istream& ifs) const;
    osg::Node*  readFile(const std::string& filename) const;
};

REGISTER_OSGPLUGIN(lasdb, ReaderWriterLasdb)