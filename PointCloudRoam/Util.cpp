#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include "Util.h"
#include <boost/version.hpp>
#include <boost/filesystem.hpp>

using namespace boost;

#ifdef WIN32
#define DIR_INTERVAL '\\'
#else
#define DIR_INTERVAL '/'
#endif

namespace Util
{
	const float Epsilon = std::numeric_limits<float>::epsilon();

	void ensure_dir(const std::string & dir)
	{
		filesystem::path fullpath(dir);
		if (!filesystem::exists(fullpath))
		{
			filesystem::create_directory(fullpath);
		}
	}

	//路径字符串添加/或\结尾
	void format_path(std::string & path)
	{
		size_t size = path.size();
		if (size == 0 || path[size - 1] != DIR_INTERVAL)
			path.push_back(DIR_INTERVAL);
	}

	//根据路径和文件名生成完整路径
	std::string make_path(const std::string & dir, const std::string & file)
	{
		std::string path = dir;
		format_path(path);
		return path + file;
	}

	//替换文件扩展名
	std::string replace_ext(const std::string & file, const std::string & ext)
	{
		size_t pos = file.rfind('.');
		return file.substr(0, pos) + ext;
	}

	//文件扩展名
	std::string get_ext(const std::string & file)
	{
		size_t pos = file.rfind('.');
		return (pos != std::string::npos) ? file.substr(pos) : std::string();
	}

	//文件名
	std::string get_name(const std::string & file)
	{
		boost::filesystem::path p(file);
#if BOOST_VERSION > 104000
		return p.filename().string();
#else
		return p.filename();
#endif
	}

	std::string get_name_without_ext(const std::string &file)
	{
		std::string name = get_name(file);
		std::string ext = get_ext(file);
		return name.substr(0, name.length() - ext.length());
	}

	//文件父目录
	std::string get_path(const std::string &file)
	{
		boost::filesystem::path p(file);
#if BOOST_VERSION > 104000
		return p.parent_path().string();
#else
		return p.parent_path();
#endif
	}

	std::string get_parent(const std::string &path)
	{
		boost::filesystem::path p(path);
#if BOOST_VERSION > 104000
		return p.parent_path().string();
#else
		return p.parent_path();
#endif
	}

	//获得文件夹下所有的文件
	void get_files(const std::string& dir, const std::string& ext, filelist& list)
	{
		filesystem::path fullPath(filesystem::initial_path());
		fullPath = filesystem::system_complete(filesystem::path(dir));

		if (!filesystem::exists(fullPath) || !filesystem::is_directory(fullPath))
		{
			return;
		}

		filesystem::directory_iterator end_iter;
		for (filesystem::directory_iterator file_itr(fullPath); file_itr != end_iter; ++file_itr)
		{
			if (!filesystem::is_directory(*file_itr) && (filesystem::extension(*file_itr) == ext || ext == ""))
			{
#if BOOST_VERSION > 104000
				std::string str = make_path(dir, file_itr->path().filename().string());
#else
				std::string str = make_path(dir, file_itr->path().filename());
#endif
				list.push_back(str);
		}
	}
}

	void get_files(const std::string& dir, const std::string& ext, std::vector<std::string>& files)
	{
		filesystem::path fullPath(filesystem::initial_path());
		fullPath = filesystem::system_complete(filesystem::path(dir));

		if (!filesystem::exists(fullPath) || !filesystem::is_directory(fullPath))
		{
			return;
		}

		filesystem::directory_iterator end_iter;
		for (filesystem::directory_iterator file_itr(fullPath); file_itr != end_iter; ++file_itr)
		{
			if (!filesystem::is_directory(*file_itr) && (filesystem::extension(*file_itr) == ext || ext == ""))
			{
#if BOOST_VERSION > 104000
				std::string str = make_path(dir, file_itr->path().filename().string());
#else
				std::string str = make_path(dir, file_itr->path().filename());
#endif
				files.push_back(str);
		}
	}
}

	bool file_exist(const std::string& file)
	{
		filesystem::path fullpath(file);
		return filesystem::exists(fullpath);
	}

	bool is_directory(const std::string &dir)
	{
		filesystem::path fullPath(filesystem::initial_path());
		fullPath = filesystem::system_complete(filesystem::path(dir));

		if (filesystem::exists(fullPath) && filesystem::is_directory(fullPath))
		{
			return true;
		}
		return false;
	}

	bool copy_file(const std::string& to_dir, const std::string& file, bool overlay)
	{
		filesystem::path fullpath(to_dir);
		if (!filesystem::exists(fullpath))
			return false;

		if (file_exist(file))
		{
			std::string file_name = get_name(file);
			if (!overlay)
			{
				if (file_exist(to_dir + "/" + file_name))
					return false;
			}

			filesystem::copy_file(filesystem::path(file), filesystem::path(to_dir + "/" + file_name));
			return true;
		}

		return false;
	}

	void rename(const std::string& new_name, const std::string& old_name)
	{
		filesystem::rename(filesystem::path(old_name), filesystem::path(new_name));
	}

	void remove_file(const std::string& filename)
	{
		if (file_exist(filename))
		{
			boost::filesystem::remove(boost::filesystem::path(filename));
			std::cout << "'" << filename << "' is removed!" << std::endl;
		}
		else
		{
			std::cerr << filename << " doesn't exist!" << std::endl;
		}
	}

}// namespace Util
