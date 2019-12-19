//#include <Windows.h>
//#include <osg/Geode>
//#include <osg/Geometry>
//#include <osgText/Text>
//#include <osgViewer/Viewer>
//#include <osgViewer/ViewerEventHandlers>
//#include <locale.h>
//#include <osgDB/ReadFile>
//
//void setupProperties (osgText::Text& textObject, osgText::Font* font, float size, const osg::Vec3& pos)
//{
//	textObject.setFont (font);//
//	textObject.setCharacterSize (size);//�����С
//	textObject.setPosition (pos);
//	textObject.setColor (osg::Vec4 (1.0, 1.0, 1.0, 1.0));
//	textObject.setAlignment (osgText::Text::CENTER_BOTTOM);//������ʾ����
//	//textObject.setAxisAlignment(osgText::Text::SCREEN);//��ȡ���ֶԳƳɷ�ʽ������Ļ����
//	//textObject.setCharacterSizeMode(osgText::Text::SCREEN_COORDS);//�����ӽǲ��ϱ仯��������ԽԶ������Խ��
//	textObject.setAutoRotateToScreen (true);//�����ӽǲ��ϱ仯����������ԽԶ������ԽС������ʵ����������
//	textObject.setBackdropType (osgText::Text::OUTLINE);//�����ֽ������
//	textObject.setBackdropColor (osg::Vec4 (1.0, 0.0, 0.0, 1.0));//�����ɫ
//	textObject.setDrawMode (osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);//������ֱ߿�
//	textObject.setAxisAlignment (osgText::Text::XZ_PLANE);//��ȡ���ֶԳƳɷ�ʽ
//}
//
//void createContent (osgText::Text& textObject, const char* string)
//{
//	int requiredSize = mbstowcs (NULL, string, 0);//���mbstowcs��һ����ΪNULL��ô�����ַ�������Ŀ
//	wchar_t* wText = new wchar_t [requiredSize + 1];
//	mbstowcs (wText, string, requiredSize + 1);//��charת����wchar����
//	textObject.setText (wText);
//	delete wText;
//}
//
//int main ()
//{
//	//setlocale (LC_ALL, ".936");//�����õ�����Ϣ
//	const char* titleString = "ľ����\n��ž����ʼ���";
//	const char* textString = {
//		"������ֻ�������������籯���ȣ�\n"
//		"���б�ȴ�����ģ�ȴ���������ױ䣻\n"
//		"��ɽ��������룬ҹ�������ղ�Թ��\n"
//		"���籡�ҽ����ɣ�������֦����Ը��"
//	};
//	osgText::Font* fontHei = osgText::readFontFile ("C:/Windows/Fonts/simhei.ttf");
//	osgText::Font* fontKai = osgText::readFontFile ("C:\\Windows\\Fonts\\simkai.ttf");
//
//	osg::ref_ptr<osgText::Text> title = new osgText::Text;
//	setupProperties (*title, fontHei, 20.0f, osg::Vec3 (0.0, 0.0, 0.0));
//	createContent (*title, titleString);
//
//	osg::ref_ptr<osgText::Text> text = new osgText::Text;
//	setupProperties (*text, fontKai, 15.0, osg::Vec3 (0.0, 0.0, -80.0f));
//	createContent (*text, textString);
//
//	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
//	geode->addDrawable (osg::createTexturedQuadGeometry (osg::Vec3 (-150.0, 1.0, -130.0), osg::Vec3 (300.0, 0.0, 0.0), osg::Vec3 (0.0, 0.0, 200.0), 1.0, 1.0));//����һ��д�ְ�
//	geode->addDrawable (title.get ());
//	geode->addDrawable (text.get ());
//
//	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
//	viewer->addEventHandler (new osgViewer::StatsHandler);
//	viewer->setSceneData (geode.get ());
//
//	return viewer->run ();
//}