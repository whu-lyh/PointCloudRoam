// Copyright (c) 2018 WuHan University. All Rights Reserved
// @author xhzou_whu@foxmail.com
// @date 2018/10/04
// @brief utility
#pragma once

#include <exception>
#include <vector>
#include <list>
#include <string>
#include <float.h>
#include <stdlib.h>
#include <fstream>
#include <liblas/liblas.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "PointCloudRoamLibs.h"

namespace Util
{
	typedef std::list<std::string> filelist;

	extern void ensure_dir(const std::string &dir);

	extern void format_path(std::string &path);

	extern std::string make_path(const std::string &dir, const std::string &file);

	extern std::string replace_ext(const std::string &file, const std::string &ext);

	extern std::string get_ext(const std::string &file);

	extern std::string get_name(const std::string &file);

	extern std::string get_name_without_ext(const std::string &file);

	extern std::string get_path(const std::string &file);

	extern std::string get_parent(const std::string &path);

	extern void get_files(const std::string& dir, const std::string& ext, filelist& list);

	extern void get_files(const std::string& dir, const std::string& ext, std::vector<std::string>& files);

	extern bool file_exist(const std::string& file);

	extern bool is_directory(const std::string &dir);

	extern bool copy_file(const std::string& to_dir, const std::string& file, bool overlay = false);

	extern void rename(const std::string& new_name, const std::string& old_name);

	extern void remove_file(const std::string& filename);

}// Util
