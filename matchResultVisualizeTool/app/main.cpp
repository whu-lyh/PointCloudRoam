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
#include <thread>
#include <future>

#include <glog/logging.h>

#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osgDB/WriteFile>
#include <osgDB/Registry>
#include <osgText/Text>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgUtil/Optimizer>

#include <yaml-cpp/yaml.h>

#include "../include/visualizeLibs.h"
#include "../include/FileUtility.h"
#include "../include/PointCloudIO.h"
#include "../include/pointCloudNode.h"
#include "../include/lineNode.h"
#include "../include/UserKeyBoardHandler.h"

namespace VF = VisualTool::FileUtility;
namespace VN = VisualTool::Node;

// 1 is the minimum number
#define interval 2

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

osg::ref_ptr<osg::Node> getHudTextNode(const std::string &match_result_name,const float pos)
{
	osg::ref_ptr<osg::Group> pGroup = new osg::Group();
	osg::ref_ptr<osg::Geode> pGeode = new osg::Geode();
	osg::ref_ptr<osgText::Text> pText = new osgText::Text();

	pText->setFont(osgText::readFontFile("fonts/simsun.ttc"));
	pText->setText(match_result_name, osgText::String::ENCODING_UTF8);
	pText->setPosition(osg::Vec3f(0.0, pos, 0.0));
	pText->setCharacterSize(25);
	pText->setColor(osg::Vec4f(0.0, 0.0, 1.0, 1.0));
	pText->setDrawMode(osgText::Text::TEXT);
	// 此条会导致hub不显示（必须设置为SCREEN）
	// pText->setAxisAlignment(osgText::Text::XZ_PLANE);
	pText->setAxisAlignment(osgText::Text::XY_PLANE);
	pText->setCharacterSizeMode(osgText::Text::OBJECT_COORDS);
	pText->setFontResolution(160, 80);
	pText->setAlignment(osgText::Text::LEFT_BOTTOM);
	pText->setLayout(osgText::Text::LEFT_TO_RIGHT);
	pText->setBackdropType(osgText::Text::NONE);
	pText->setColorGradientMode(osgText::Text::SOLID);

	// 步骤一：创建HUD摄像机
	osg::ref_ptr<osg::Camera> pCamera = new osg::Camera;
	// 步骤二：设置投影矩阵
	pCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1920, 0, 1080));
	// 步骤三：设置视图矩阵,同时确保不被场景中其他图形位置变换影响, 使用绝对帧引用
	pCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	pCamera->setViewMatrix(osg::Matrix::identity());
	// 步骤四：清除深度缓存
	pCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	// 步骤五：设置POST渲染顺序(最后渲染)
	pCamera->setRenderOrder(osg::Camera::POST_RENDER);
	// 步骤六：设置为不接收事件,始终得不到焦点
	pCamera->setAllowEventFocus(false);

	pGeode = new osg::Geode();
	osg::ref_ptr<osg::StateSet> pStateSet = pGeode->getOrCreateStateSet();
	// 步骤七：关闭光照
	pStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	// 步骤八：关闭深度测试
	pStateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

	pGeode->addDrawable(pText.get());
	pCamera->addChild(pGeode.get());
	pGroup->addChild(pCamera.get());

	return pGroup.get();
}

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

	std::string las_patch2_path = config["PointCloudPatch2"].as<std::string>();
	std::vector<std::string> las_patch2_files;
	VF::GetFiles(las_patch2_path, ".las", las_patch2_files);
	int n_patch2 = (int)las_patch2_files.size();

	// load las files in batch
	VisualTool::Point3d offset_average;
	
	for (int i = 0; i < n_patch1; i++)
	{
		VisualTool::Point3d offset_tmp;
		VisualTool::PointIO::loadSingleLASHeader<int>(las_patch1_files[i], offset_tmp);
		offset_average += offset_tmp;
	}

	for (int i = 0; i < n_patch2; i++)
	{
		VisualTool::Point3d offset_tmp;
		VisualTool::PointIO::loadSingleLASHeader<int>(las_patch2_files[i], offset_tmp);
		offset_average += offset_tmp;
	}

	// get average offset
	offset_average /= (n_patch1 + n_patch2);

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

	std::vector<osg::ref_ptr<osg::Geode>> nodeptr_vec_src(n_patch1);
	for (int i = 0; i < n_patch1; i++)
	{
		VN::pointCloudNode<pcl::PointXYZ> pc_node(las_patch1_files[i], color_src);
		pc_node.setAveOffset(offset_average);
		pc_node.setDownSampleInterval(interval);
		nodeptr_vec_src[i] = pc_node.getGeoNode();
	}

	std::vector<osg::ref_ptr<osg::Geode>> nodeptr_vec_tar(n_patch2);
	for (int i = 0; i < n_patch2; i++)
	{
		VN::pointCloudNode<pcl::PointXYZ> pc_node(las_patch2_files[i], color_tar);
		pc_node.setAveOffset(offset_average);
		pc_node.setDownSampleInterval(interval);
		nodeptr_vec_tar[i] = pc_node.getGeoNode();
	}

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
	if (n_match == 0)
	{
		LOG(WARNING) << "No match result is loaded!";
	}

	std::vector<osg::ref_ptr<osg::Geode>> nodeptr_vec_match(n_match);
	std::vector<osg::ref_ptr<osg::Node>> nodeptr_vec_match_text(n_match);
	float line_vertical_spacing = 0.f;
	for (int i = 0; i < n_match; i++)
	{
		VN::lineNode line_node(match_files[i], color_pt, color_line, pt_size, line_width);
		line_node.setAveOffset(offset_average);
		nodeptr_vec_match[i] = line_node.getGeoNode();
		std::string label = "No."+std::to_string(i)+" \t"+VisualTool::FileUtility::GetNameWithoutExt(match_files[i]) +
			", matched numbers: " + std::to_string(line_node.getLineNum());
		nodeptr_vec_match_text[i] = getHudTextNode(label, line_vertical_spacing);
		line_vertical_spacing += 25;
	}

	// base root node
	//osg::ref_ptr<osg::Group> root = new osg::Group;
	osg::ref_ptr<osg::Switch> root = new osg::Switch;
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
	// text display
	for (auto node : nodeptr_vec_match_text)
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
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.addEventHandler(new osgViewer::StatsHandler());

	{
		osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
		keyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
		keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
		keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
		keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());
		viewer.setCameraManipulator(keyswitchManipulator.get());
		viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);
	}

	viewer.addEventHandler(new VisualTool::UserKeyBoardHandler());

	viewer.getCamera()->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 1.f)); //background=white
	osgUtil::Optimizer optimizer;
	optimizer.optimize(root.get());
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

	viewer.setThreadingModel(osgViewer::Viewer::CullThreadPerCameraDrawThreadPerContext);
	viewer.realize();
	viewer.run();

	system("pause");
	return 0;
}
