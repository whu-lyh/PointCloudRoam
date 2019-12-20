
#include <windows.h>// this line have to be at the first line of the whole header files

#include <QApplication>
#include "mygraphicwindowqt.h"

#include <osgDB/ReadFile>

#include "mainwindow.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    //osg::Node* model = osgDB::readNodeFile("cow.osg");
    osg::Node* model = osgDB::readNodeFile("D:/Software/OpenSceneGraph-Data/cow.osgt");

    osg::ref_ptr<osg::Group> group = new osg::Group;
    group->addChild(model);

    // The qt window
    MainWindow widget;

    // set the scene to render
    widget.setSceneData(group.get());
    widget.setCameraManipulator(new osgGA::TrackballManipulator);

    widget.setGeometry(100, 100, 800, 600);
    widget.show();

    return app.exec();
}
