#pragma once

#include <float.h>
#include <stdlib.h>

#include <vector>
#include <list>
#include <string>

#include "visualizeLibs.h"

namespace VisualTool {

typedef std::list<std::string> filelist;
typedef std::vector<std::string> filevec;

namespace FileUtility {
bool CopyFile(const std::string& to_dir,
	const std::string& file, bool overlay = false);

bool EnsureDir(const std::string &dir);

bool FileExist(const std::string& file);

void FormatPath(std::string &path);

std::string GetExt(const std::string &file);

std::string GetName(const std::string &file);

std::string GetNameWithoutExt(const std::string &file);

std::string GetParent(const std::string &path);

std::string GetPath(const std::string &file);

void GetFiles(const std::string& dir,
	const std::string& ext, filelist& files);

void GetFiles(const std::string& dir,
	const std::string& ext, filevec& files);

bool IsDirectory(const std::string &dir);

std::string MakePath(const std::string &dir,
	const std::string &file);

void Rename(const std::string& new_name,
	const std::string& old_name);

std::string ReplaceExt(const std::string &file,
	const std::string &ext);

void GetSubDirs(const std::string& dir, filelist& list);

void GetFilesOrDirs(const std::string& dir,
	const bool get_files, const std::string& ext, filelist& list);
}  // namespace FileUtility
}  // namespace VisualTool
