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

#include <glog/logging.h>

#include <opencv2/core/core.hpp>

#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

#include "../include/Config.h"
#include "../include/visualizeLibs.h"
#include "../include/FileUtility.h"
#include "../include/pointCloudNode.h"
#include "../include/lineNode.h"
#include "../include/PointCloudIO.h"

namespace VF = VisualTool::FileUtility;
namespace VP = VisualTool::PointIO;
namespace VN = VisualTool::Node;

osg::Vec4 toOSGVec4(const cv::Mat &colors_)
{
	osg::Vec4 v(colors_.at<float>(0), colors_.at<float>(1), colors_.at<float>(2), 1.0f);
	return v;
}

osg::Vec4 toOSGVec4(const std::vector<float> &colors_)
{
	if (colors_.size() < 4)
	{
		return osg::Vec4(1.f, 1.f, 1.f, 1.f);
	}

	osg::Vec4 v(colors_[0], colors_[1], colors_[2], colors_[3]);
	return v;
}

int saveNodes(std::vector<osg::ref_ptr<osg::Geode>> &node_ptr_vec, std::vector<std::string> &files)
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

	std::string base_path = VF::Config::get<std::string>("MatchResultPath");
	base_path += "/out_node_file";
	if (!VF::EnsureDir(base_path))
	{
		LOG(ERROR) << "Fail to create the out put dir";
	}

	for (int i = 0; i < files.size(); i++, ret++)
	{
		std::string node_name = VF::GetNameWithoutExt(files[i]);
		std::string node_full_out_name = base_path + "/" + node_name + ".osg";
		if (!osgDB::writeNodeFile(*node_ptr_vec[i], node_full_out_name))
		{
			LOG(WARNING) << "Current node fail to be saved: " << node_full_out_name;
		}
	}

	return ret;
}

/*
�����⾡�����٣�pcl��boost��osg��glog��opencv
���ܣ����Զ�ȡ���las�ļ���ƥ�����ļ���ͨ��osg��ʾ�����ֵ��ƣ�����ֱ�߱�ʾƥ���ϵ������ͨ��������������ʾ�����Ǹ��ĸ�ƥ���ϵ
��ͨ��osg�ϵĽ�������ʾƥ���������ƥ���ļ�����
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
		return 0;
	}

	// ger configuration parameter file
	std::string config_file = argv [1];
	if (!VF::Config::setParameterFile(config_file))
	{
		LOG(ERROR) << "The configuration file is not existed! plz check it." << std::endl;
		google::FlushLogFiles(google::GLOG_ERROR);
		google::ShutdownGoogleLogging();
		return 0;
	}

	std::string las_patch1_path = VF::Config::get<std::string>("PointCloudPatch1");
	std::vector<std::string> las_patch1_files;
	VF::GetFiles(las_patch1_path, ".las", las_patch1_files);
	int n_patch1 = (int)las_patch1_files.size();

	// load las files in batch
	VisualTool::Point3d offset_src;

	// color setting
	std::vector<float> color_vec = VF::Config::get<std::vector<float>>("ColorPatch1");
	osg::Vec4 color_src = toOSGVec4(color_vec);
	
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

	std::string las_patch2_path = VF::Config::get<std::string>("PointCloudPatch2");
	std::vector<std::string> las_patch2_files;
	VF::GetFiles(las_patch2_path, ".las", las_patch2_files);
	int n_patch2 = (int)las_patch2_files.size();

	// load las files in batch
	VisualTool::Point3d offset_tar;

	// color setting
	color_vec = VF::Config::get<std::vector<float>>("ColorPatch2");
	osg::Vec4 color_tar = toOSGVec4(color_vec);

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

	float pt_size = VF::Config::get<float>("PointSize");
	float line_width = VF::Config::get<float>("PointSize");

	std::string base_path = VF::Config::get<std::string>("MatchResultPath");
	std::vector<std::string> match_files;
	VF::GetFiles(base_path, ".mrf", match_files);
	int n_match = (int)match_files.size();

	std::vector<osg::ref_ptr<osg::Geode>> nodeptr_vec_match(n_match);
	for (int i = 0; i < n_match; i++)
	{
		VN::lineNode line_node(match_files[i], color_tar, color_tar, pt_size, line_width);
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
	if (std::stoi(VF::Config::get<std::string>("SaveNodes")) == 1)
	{
		std::thread saveNode_thread(saveNodes, std::ref(nodeptr_vec_src), std::ref(las_patch1_files));
	}

	//get the resolution of the window screen
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE) << "Error, no WindowSystemInterface available, cannot create windows." << std::endl;
		return 0;
	}

	unsigned int win_width = 0, win_height = 0;
	std::vector<int> winsize_vec = VF::Config::get<std::vector<int>>("WindowSize");
	if (winsize_vec.empty())
	{
		LOG(WARNING) << "No window size is set!, the osg window will be set as default size of 1920x1080";
		win_width = 1920;
		win_height = 1080;
	}
	else
	{
		win_width = winsize_vec[0];
		win_height = winsize_vec[1];
	}

	osg::GraphicsContext::ScreenIdentifier main_screen_id;

	main_screen_id.readDISPLAY();
	main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
	wsi->getScreenResolution(main_screen_id, win_width, win_height);

	osgViewer::Viewer viewer;
	viewer.getCamera()->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 1.f));
	viewer.setSceneData(root.get());
	viewer.home();

	//�����ǵ���Ļ��ʾ
	viewer.setUpViewOnSingleScreen(1);
	//viewer.apply(new osgViewer::SingleScreen(0));

	osgViewer::GraphicsWindow *pWnd = dynamic_cast<osgViewer::GraphicsWindow*>(viewer.getCamera()->getGraphicsContext());
	if (pWnd)
	{
		pWnd->setWindowDecoration(false);
	}

	return viewer.run();

	system("pause");
	return 0;
}
