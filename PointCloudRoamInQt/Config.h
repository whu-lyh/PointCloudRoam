// Copyright (c) 2018 WuHan University. All Rights Reserved
// @author xhzou_whu@foxmail.com
// @date 2018/10/04
// @brief point cloud section
#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core.hpp>
#include <glog/logging.h>

#ifdef UTILITY_EXPORT
#define UTILITY_API __declspec(dllexport)
#else
#define UTILITY_API __declspec(dllimport)
#endif

namespace Util
{
	//class: singleton class for parameter configuration
	class UTILITY_API Config
	{
	private:
		static boost::shared_ptr<Config> config_;
		cv::FileStorage file_;
		Config() {}   // private constructor make a singleton

	public:
		~Config()
		{
			if (file_.isOpened())
				file_.release();
		}

		// set a new parameter file
		static bool setParameterFile(const std::string& filename)
		{
			if (config_ == nullptr)
				config_ = boost::shared_ptr<Config>(new Config);
			config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
			if (config_->file_.isOpened() == false)
			{
				LOG(ERROR) << "parameter file " << filename << " does not exist.";
				config_->file_.release();
				return false;
			}
			return true;
		}

		// access parameters
		template <typename T>
		static T get(const std::string& key)
		{
			return T(config_->file_[key]);
		}

		// set parameters (type of value is double / string)
		template <typename T>
		void set(const std::string& key, T value)
		{
			config_->file_.write(key, value);
		}
	};

	boost::shared_ptr<Config> Config::config_ = nullptr;

}// namespace Util
