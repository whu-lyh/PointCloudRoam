#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif
#ifndef WIN32
#define WIN32
#endif
#include <windows.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <future>

#include <glog/logging.h>

#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

#include <yaml-cpp/yaml.h>

#include "../include/visualizeLibs.h"
#include "../include/FileUtility.h"
#include "../include/pointCloudNode.h"
#include "../include/lineNode.h"
#include "../include/PointCloudIO.h"

namespace VF = VisualTool::FileUtility;
namespace VP = VisualTool::PointIO;
namespace VN = VisualTool::Node;

int saveNodes(std::vector<osg::ref_ptr<osg::Geode>> &node_ptr_vec, std::vector<std::string> &files, std::promise<int> &pro_obj)
{
	int node_size = node_ptr_vec.size();
	int file_size = files.size();
	int ret = 0;

	if (node_size != file_size)
	{
		LOG(WARNING) << "File names size is not equal to node size";
		return 0;
	}

	if (node_size == 0 || file_size == 0)
	{
		LOG(WARNING) << "File doesn't existed";
		return 0;
	}

	std::string base_path = VF::GetParent(files[0]);
	base_path += "/out_node_file";
	if (!VF::EnsureDir(base_path))
	{
		LOG(ERROR) << "Fail to create the out put dir";
	}

	for (int i = 0; i < files.size(); i++, ret++)
	{
		std::string node_name = VF::GetNameWithoutExt(files[i]);
		std::string node_full_out_name = base_path + "/" + node_name + ".osg";
		if (node_ptr_vec[i])
		{
			if (!osgDB::writeNodeFile(*node_ptr_vec[i], node_full_out_name))
			{
				LOG(WARNING) << "Current node fail to be saved: " << node_full_out_name;
			}
		}	
	}

	// set return value in promise
	pro_obj.set_value(ret);
	return ret;
}

/*
依赖库尽可能少：pcl，boost，osg，glog，yaml-cpp
通过键盘来调整显示的是那个哪个匹配关系
并通过osg上的界面来显示匹配对数量，匹配文件名字
*/
int main(int argc, char** argv)
{
	//glog setting
	FLAGS_logtostderr = true;
	google::FlushLogFiles(google::GLOG_INFO);
	google::InitGoogleLogging("matchResultVisualizeTool");
	google::SetLogFilenameExtension("log_");
	FLAGS_log_dir = "../logs";
	FLAGS_logbufsecs = 0;
	//set the precision of the out data 4 accurate number behind dot
	LOG(INFO) << std::setiosflags(std::ios::fixed) << std::setprecision(4);

	if (argc < 2)
	{
		LOG(ERROR) << "Usage: matchResultVisualizeTool.exe configuration-file.yaml" << std::endl;
		google::FlushLogFiles(google::GLOG_ERROR);
		google::ShutdownGoogleLogging();
		system("pause");
		return 0;
	}

	// ger configuration parameter file
	std::string config_file = argv [1];
	if (!VF::FileExist(config_file))
	{
		LOG(ERROR) << "The configuration file is not existed! plz check it." << std::endl;
		google::FlushLogFiles(google::GLOG_ERROR);
		google::ShutdownGoogleLogging();
		system("pause");
		return 0;
	}
		
	YAML::Node config;
	try {
		config = YAML::LoadFile(config_file);
	}
	catch (const YAML::ParserException& ex) {
		LOG(ERROR) << ex.what() << std::endl;
		google::FlushLogFiles(google::GLOG_ERROR);
		google::ShutdownGoogleLogging();
		system("pause");
		return 0;
	}

	if (!config.IsDefined())
	{
		LOG(ERROR) << "Fail to load the configuration file! plz check it." << std::endl;
		google::FlushLogFiles(google::GLOG_ERROR);
		google::ShutdownGoogleLogging();
		system("pause");
		return 0;
	}

	std::string las_patch1_path = config["PointCloudPatch1"].as<std::string>();
	std::vector<std::string> las_patch1_files;
	VF::GetFiles(las_patch1_path, ".las", las_patch1_files);
	int n_patch1 = (int)las_patch1_files.size();

	// load las files in batch
	VisualTool::Point3d offset_src;

	// color setting
	osg::Vec4 color_src;
	if (config["ColorPatch1"].size() != 4)
	{
		color_src = osg::Vec4(1.f, 0.f, 0.f, 1.f);
	}
	else
	{
		color_src[0] = config["ColorPatch1"]["r"].as<float>();
		color_src[1] = config["ColorPatch1"]["g"].as<float>();
		color_src[2] = config["ColorPatch1"]["b"].as<float>();
		color_src[3] = config["ColorPatch1"]["a"].as<float>();
	}
	
	std::vector<osg::ref_ptr<osg::Geode>> nodeptr_vec_src(n_patch1);
	for (int i = 0; i < n_patch1; i++)
	{
		VN::pointCloudNode<pcl::PointXYZ> pc_node(las_patch1_files[i], color_src);
		nodeptr_vec_src[i] = pc_node.getGeoNode();
		offset_src = offset_src + pc_node.getOffset();
		//or like this
		//offset_src += pc_node.getOffset();
	}
	
	// get average offset of patch1
	offset_src /= n_patch1;

	std::string las_patch2_path = config["PointCloudPatch2"].as<std::string>();
	std::vector<std::string> las_patch2_files;
	VF::GetFiles(las_patch2_path, ".las", las_patch2_files);
	int n_patch2 = (int)las_patch2_files.size();

	// load las files in batch
	VisualTool::Point3d offset_tar;

	// color setting
	osg::Vec4 color_tar;
	if (!config["ColorPatch2"].IsSequence())
	{
		color_tar = osg::Vec4(0.f, 1.f, 0.f, 1.f);
	}
	else
	{
		std::vector<float> color_vec = config["ColorPatch2"].as<std::vector<float>>();
		color_tar = osg::Vec4(color_vec[0], color_vec[1], color_vec[2], color_vec[3]);
	}

	std::vector<osg::ref_ptr<osg::Geode>> nodeptr_vec_tar(n_patch2);
	for (int i = 0; i < n_patch2; i++)
	{
		VN::pointCloudNode<pcl::PointXYZ> pc_node(las_patch2_files[i], color_tar);
		nodeptr_vec_tar[i] = pc_node.getGeoNode();
		offset_tar = offset_tar + pc_node.getOffset();
		//or like this
		//offset_src += pc_node.getOffset();
	}

	// get average offset of patch2
	offset_tar /= n_patch2;

	float pt_size = config["PointSize"].as<float>();
	float line_width = config["LineWidth"].as<float>();

	osg::Vec4 color_line, color_pt;
	if (!config["PointColor"].IsSequence())
	{
		color_pt = osg::Vec4(1.f, 1.f, 1.f, 1.f);
	}
	else
	{
		std::vector<float> color_vec = config["PointColor"].as<std::vector<float>>();
		color_pt = osg::Vec4(color_vec[0], color_vec[1], color_vec[2], color_vec[3]);
	}
	if (!config["LineColor"].IsSequence())
	{
		color_line = osg::Vec4(0.f, 0.f, 1.f, 1.f);
	}
	else
	{
		std::vector<float> color_vec = config["LineColor"].as<std::vector<float>>();
		color_line = osg::Vec4(color_vec[0], color_vec[1], color_vec[2], color_vec[3]);
	}

	std::string base_path = config["MatchResultPath"].as<std::string>();
	std::vector<std::string> match_files;
	VF::GetFiles(base_path, ".mrf", match_files);
	int n_match = (int)match_files.size();

	std::vector<osg::ref_ptr<osg::Geode>> nodeptr_vec_match(n_match);
	for (int i = 0; i < n_match; i++)
	{
		VN::lineNode line_node(match_files[i], color_pt, color_line, pt_size, line_width);
		nodeptr_vec_match[i] = line_node.getGeoNode();
	}

	osg::ref_ptr<osg::Group> root = new osg::Group;
	for (auto node: nodeptr_vec_src)
	{
		root->addChild(node);
	}
	for (auto node : nodeptr_vec_tar)
	{
		root->addChild(node);
	}
	for (auto node : nodeptr_vec_match)
	{
		root->addChild(node);
	}

	// check whether to save nodes
	std::thread *saveNode_thread_1, *saveNode_thread_2, *saveNode_thread_3;
	std::promise<int> saveNode_promise_obj_1, saveNode_promise_obj_2, saveNode_promise_obj_3;
	std::future<int> saveNode_future_1 = saveNode_promise_obj_1.get_future();
	std::future<int> saveNode_future_2 = saveNode_promise_obj_2.get_future();
	std::future<int> saveNode_future_3 = saveNode_promise_obj_3.get_future();
	if (std::stoi(config["SaveNodes"].as<std::string>()) == 1)
	{
		LOG(INFO) << "Begin to save nodes, current thread id is: " << std::this_thread::get_id();
		// in sub thread
		saveNode_thread_1 = new std::thread(saveNodes, std::ref(nodeptr_vec_src), std::ref(las_patch1_files), std::ref(saveNode_promise_obj_1));
		saveNode_thread_2 = new std::thread(saveNodes, std::ref(nodeptr_vec_tar), std::ref(las_patch2_files), std::ref(saveNode_promise_obj_2));
		saveNode_thread_3 = new std::thread(saveNodes, std::ref(nodeptr_vec_match), std::ref(match_files), std::ref(saveNode_promise_obj_3));
		
		// using async but could not get thread id and multi-time return number
		//saveNode_future = std::async(saveNodes, std::ref(nodeptr_vec_src), std::ref(las_patch1_files));
		
		LOG(INFO) << "Save nodes in new sub thread, thread id is: " << saveNode_thread_1->get_id();
		LOG(INFO) << "Save nodes in new sub thread, thread id is: " << saveNode_thread_2->get_id();
		LOG(INFO) << "Save nodes in new sub thread, thread id is: " << saveNode_thread_3->get_id();
	}

	//get the resolution of the window screen
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE) << "Error, no WindowSystemInterface available, cannot create windows." << std::endl;
		return 0;
	}

	unsigned int win_width = 0, win_height = 0;
	if (!config["WindowSize"].IsSequence() || config["WindowSize"].size() != 2)
	{
		LOG(WARNING) << "No window size is set!, the osg window will be set as default size of 1920x1080";
		win_width = 1920;
		win_height = 1080;
	}
	else
	{
		win_width = config["WindowSize"][0].as<int>();
		win_height = config["WindowSize"][1].as<int>();
	}

	osg::GraphicsContext::ScreenIdentifier main_screen_id;

	main_screen_id.readDISPLAY();
	main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
	wsi->getScreenResolution(main_screen_id, win_width, win_height);

	osgViewer::Viewer viewer;
	viewer.getCamera()->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 1.f));
	viewer.setSceneData(root.get());
	viewer.home();

	//这里是单屏幕显示
	viewer.setUpViewOnSingleScreen(1);

	osgViewer::GraphicsWindow *pWnd = dynamic_cast<osgViewer::GraphicsWindow*>(viewer.getCamera()->getGraphicsContext());
	if (pWnd)
	{
		pWnd->setWindowDecoration(false);
	}

	//check whether the sub saveNode_thread is still existed or not
	if (std::stoi(config["SaveNodes"].as<std::string>()) == 1)
	{
		if (saveNode_future_1.get() == n_patch1 && saveNode_future_2.get() == n_patch2 && saveNode_future_3.get() == n_match)
		{
			LOG(INFO) << "Whole node files are successfully saved!";
		}
	}

	viewer.realize();
	viewer.run();

	system("pause");
	return 0;
}
