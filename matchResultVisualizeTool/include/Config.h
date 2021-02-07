/*
	Copyright (c) 2021 WuHan University. All Rights Reserved
	First create: 2021/02/06
	Detail: configuration file load func based on yaml-cpp instead of opencv filestorage
	Author: yuhao li
	Email:	yhaoli@whu.edu.cn
*/
#pragma once

#include <yaml-cpp/yaml.h>

#include <boost/make_shared.hpp>

#include <glog/logging.h>

namespace VisualTool {
namespace FileUtility {
	//class: singleton class for parameter configuration
	class Config
	{
	private:
		static boost::shared_ptr<Config> config_;
		YAML::Node node_;
		Config() {}   // private constructor make a singleton

	public:
		~Config()
		{}

		// set a new parameter file
		static bool setParameterFile(const std::string& filename)
		{
			if (config_ == nullptr)
				config_ = boost::shared_ptr<Config>(new Config);
			//config_->node_ = 
			YAML::LoadFile("E:/Vs15WorkSpace/PointCloudRoam/matchResultVisualizeTool/config/Configuration-example.yaml");
			if (!config_->node_.IsDefined())
			{
				LOG(ERROR) << "parameter file " << filename << " does not exist.";
				return false;
			}
			return true;
		}

		// access parameters
		template <typename T>
		static T get(const std::string& key)
		{
			return config_->node_[key].as<T>();
		}

		// set parameters (type of value is double / string)
		template <typename T>
		void set(const std::string& key, T value)
		{
			//config_->file_.write(key, value);
		}
	};

	boost::shared_ptr<Config> Config::config_ = nullptr;

}// namespace FileUtility
}// namespace VisualTool
