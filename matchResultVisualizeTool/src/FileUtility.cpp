#pragma once

#include <boost/version.hpp>
#include <boost/filesystem.hpp>

#include "../include/FileUtility.h"

namespace VisualTool {
namespace FileUtility {
using namespace boost;

#ifdef WIN32
#define DIR_INTERVAL '\\'
#else
#define DIR_INTERVAL '/'
#endif

bool CopyFile(const std::string& to_dir, const std::string& file, bool overlay) {
	filesystem::path fullpath(to_dir);
	if (!filesystem::exists(fullpath))
		return false;
	if (FileExist(file)) {
		std::string file_name = GetName(file);
		if (!overlay) {
			if (FileExist(to_dir + DIR_INTERVAL + file_name))
				return false;
		}
		filesystem::copy_file(filesystem::path(file),
			filesystem::path(to_dir + DIR_INTERVAL + file_name));
		return true;
	}
	return false;
}

bool EnsureDir(const std::string &dir) {
	bool bSuccess = true;
	filesystem::path fullpath(dir);
	filesystem::path parent_path = fullpath.parent_path();
	if (!filesystem::exists(parent_path)) {
		bSuccess |= EnsureDir(parent_path.string());
		bSuccess |= filesystem::create_directory(fullpath);
	} else if (!filesystem::exists(fullpath)) {
		bSuccess |= filesystem::create_directory(fullpath);
	}
	return bSuccess;
}

bool FileExist(const std::string& file) {
	filesystem::path fullpath(file);
	return filesystem::exists(fullpath) && GetExt(file) != "";
}

void FormatPath(std::string &path) {
	size_t size = path.size();
	if (size == 0 || path[size - 1] != DIR_INTERVAL) {
		path.push_back(DIR_INTERVAL);
	}
}

std::string GetExt(const std::string &file) {
	size_t pos = file.rfind('.');
	return (pos != std::string::npos) ? file.substr(pos) : std::string();
}

std::string GetName(const std::string &file) {
	boost::filesystem::path p(file);
#if BOOST_VERSION > 104000
	return p.filename().string();
#else
	return p.filename();
#endif
}

std::string GetNameWithoutExt(const std::string &file) {
	std::string name = GetName(file);
	std::string ext = GetExt(file);
	return name.substr(0, name.length() - ext.length());
}

std::string GetParent(const std::string &path) {
	boost::filesystem::path p(path);
#if BOOST_VERSION > 104000
	return p.parent_path().string();
#else
	return p.parent_path();
#endif
}

std::string GetPath(const std::string &file) {
	return GetParent(file);
}

template <class Container>
void TemplatedGetFiles(const std::string& dir,
	const std::string& ext,
	Container& container) {
	filesystem::path fullPath(filesystem::initial_path());
	fullPath = filesystem::system_complete(filesystem::path(dir));

	if (!filesystem::exists(fullPath) || !filesystem::is_directory(fullPath)) {
		return;
	}

	filesystem::directory_iterator end_iter;
	filesystem::directory_iterator file_itr(fullPath);
	for (; file_itr != end_iter; ++file_itr) {
		if (!filesystem::is_directory(*file_itr) &&
			(filesystem::extension(*file_itr) == ext || ext == "")) {
#if BOOST_VERSION > 104000
			std::string str = MakePath(dir, file_itr->path().filename().string());
#else
			std::string str = MakePath(dir, file_itr->path().filename());
#endif
			container.push_back(str);
		}
	}
}

void GetFiles(const std::string& dir, const std::string& ext, filelist& files) {
	TemplatedGetFiles(dir, ext, files);
}

void GetFiles(const std::string& dir, const std::string& ext, filevec& files) {
	TemplatedGetFiles(dir, ext, files);
}

bool IsDirectory(const std::string &dir) {
	filesystem::path fullPath(filesystem::initial_path());
	fullPath = filesystem::system_complete(filesystem::path(dir));

	if (filesystem::exists(fullPath) && filesystem::is_directory(fullPath)) {
		return true;
	}
	return false;
}

std::string MakePath(const std::string &dir, const std::string &file) {
	std::string path = dir;
	FormatPath(path);
	return path + file;
}

void Rename(const std::string& new_name, const std::string& old_name) {
	filesystem::rename(filesystem::path(old_name), filesystem::path(new_name));
}

std::string ReplaceExt(const std::string &file, const std::string &ext) {
	size_t pos = file.rfind('.');
	return file.substr(0, pos) + ext;
}

void GetSubDirs(const std::string& dir, filelist& list) {
	GetFilesOrDirs(dir, false, "", list);
}

void GetFilesOrDirs(const std::string& dir, const bool get_files,
	const std::string& ext, filelist& list) {
	filesystem::path fullPath(filesystem::initial_path());
	fullPath = filesystem::system_complete(filesystem::path(dir));

	if (!filesystem::exists(fullPath) || !filesystem::is_directory(fullPath)) {
		return;
	}

	filesystem::directory_iterator end_iter;
	for (filesystem::directory_iterator file_itr(fullPath); file_itr != end_iter; ++file_itr) {
		if ((get_files && !filesystem::is_directory(*file_itr) &&
			(filesystem::extension(*file_itr) == ext || ext == "")) ||
			(!get_files && filesystem::is_directory(*file_itr))) {
#if BOOST_VERSION > 104000
			std::string str = MakePath(dir, file_itr->path().filename().string());
#else
			std::string str = make_path(dir, file_itr->path().filename());
#endif
			list.push_back(str);
		}
	}
}
} // namespace FileUtility
}  // namespace VisualTool
