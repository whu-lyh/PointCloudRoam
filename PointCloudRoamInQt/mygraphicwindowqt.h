#pragma once

#ifndef MYGRAPHICWINDOWQT_H
#define MYGRAPHICWINDOWQT_H


#include <osgViewer/GraphicsWindow>
#include <osgQt/Export>

#include <QMutex>
#include <QEvent>
#include <QQueue>
#include <QSet>
#include <QtOpenGL/QGLWidget>
#include <QObject>
#include "movethread.h"
class QInputEvent;
class MainWindow;
namespace osgViewer {
    class ViewerBase;
}


// forward declarations
class MyGraphicWindowQt;

/// The function sets the WindowingSystem to Qt.
void initQtWindowingSystem();

/** The function sets the viewer that will be used after entering
 *  the Qt main loop (QCoreApplication::exec()).
 *
 *  The function also initializes internal structures required for proper
 *  scene rendering.
 *
 *  The method must be called from main thread. */
void setViewer( osgViewer::ViewerBase *viewer );


class GLWidget : public QGLWidget
{
    typedef QGLWidget inherited;

public:

    GLWidget( QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0, bool forwardKeyEvents = false );
    GLWidget( QGLContext* context, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0, bool forwardKeyEvents = false );
    GLWidget( const QGLFormat& format, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0, bool forwardKeyEvents = false );
    virtual ~GLWidget();

    inline void setGraphicsWindow( MyGraphicWindowQt* gw ) { _gw = gw; }
    inline MyGraphicWindowQt* getGraphicsWindow() { return _gw; }
    inline const MyGraphicWindowQt* getGraphicsWindow() const { return _gw; }

    inline bool getForwardKeyEvents() const { return _forwardKeyEvents; }
    virtual void setForwardKeyEvents( bool f ) { _forwardKeyEvents = f; }

    void setKeyboardModifiers( QInputEvent* event );

    virtual void keyPressEvent( QKeyEvent* event );
    virtual void keyReleaseEvent( QKeyEvent* event );
    virtual void mousePressEvent( QMouseEvent* event );
    virtual void mouseReleaseEvent( QMouseEvent* event );
    virtual void mouseDoubleClickEvent( QMouseEvent* event );
    virtual void mouseMoveEvent( QMouseEvent* event );
    virtual void wheelEvent( QWheelEvent* event );

protected:

    int getNumDeferredEvents()
    {
        QMutexLocker lock(&_deferredEventQueueMutex);
        return _deferredEventQueue.count();
    }
    void enqueueDeferredEvent(QEvent::Type eventType, QEvent::Type removeEventType = QEvent::None)
    {
        QMutexLocker lock(&_deferredEventQueueMutex);

        if (removeEventType != QEvent::None)
        {
            if (_deferredEventQueue.removeOne(removeEventType))
                _eventCompressor.remove(eventType);
        }

        if (_eventCompressor.find(eventType) == _eventCompressor.end())
        {
            _deferredEventQueue.enqueue(eventType);
            _eventCompressor.insert(eventType);
        }
    }
    void processDeferredEvents();

    friend class MyGraphicWindowQt;
    MyGraphicWindowQt* _gw;

    QMutex _deferredEventQueueMutex;
    QQueue<QEvent::Type> _deferredEventQueue;
    QSet<QEvent::Type> _eventCompressor;

    bool _forwardKeyEvents;

    virtual void resizeEvent( QResizeEvent* event );
    virtual void moveEvent( QMoveEvent* event );
    virtual void glDraw();
    virtual bool event( QEvent* event );
};



class MyGraphicWindowQt : public QObject,public osgViewer::GraphicsWindow
{
    Q_OBJECT

public:
    MyGraphicWindowQt( osg::GraphicsContext::Traits* traits, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0 );
    MyGraphicWindowQt( GLWidget* widget );
    virtual ~MyGraphicWindowQt();

    inline GLWidget* getGLWidget() { return _widget; }
    inline const GLWidget* getGLWidget() const { return _widget; }

    /// deprecated
    inline GLWidget* getGraphWidget() { return _widget; }
    /// deprecated
    inline const GLWidget* getGraphWidget() const { return _widget; }

    struct WindowData : public osg::Referenced
    {
        WindowData( GLWidget* widget = NULL, QWidget* parent = NULL ): _widget(widget), _parent(parent) {}
        GLWidget* _widget;
        QWidget* _parent;
    };

    bool init( QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f );

    static QGLFormat traits2qglFormat( const osg::GraphicsContext::Traits* traits );
    static void qglFormat2traits( const QGLFormat& format, osg::GraphicsContext::Traits* traits );
    static osg::GraphicsContext::Traits* createTraits( const QGLWidget* widget );

    virtual bool setWindowRectangleImplementation( int x, int y, int width, int height );
    virtual void getWindowRectangle( int& x, int& y, int& width, int& height );
    virtual bool setWindowDecorationImplementation( bool windowDecoration );
    virtual bool getWindowDecoration() const;
    virtual void grabFocus();
    virtual void grabFocusIfPointerInWindow();
    virtual void raiseWindow();
    virtual void setWindowName( const std::string& name );
    virtual std::string getWindowName();
    virtual void useCursor( bool cursorOn );
    virtual void setCursor( MouseCursor cursor );

    virtual bool valid() const;
    virtual bool realizeImplementation();
    virtual bool isRealizedImplementation() const;
    virtual void closeImplementation();
    virtual bool makeCurrentImplementation();
    virtual bool releaseContextImplementation();
    virtual void swapBuffersImplementation();
    virtual void runOperations();

    virtual void requestWarpPointer( float x, float y );

    void setMainWindow(MainWindow* mainWindow);

public:

    friend class GLWidget;
    GLWidget* _widget;
    QWidget*    _myWidget;
    bool _ownsWidget;
    QCursor _currentCursor;
    bool _realized;
    MoveThread* _moveThread;
    MainWindow* _mainWindow;
Q_SIGNALS:
    void moveOpenglContextToNewThread(QThread *newThread);
public slots:
    void onMoveOpenglContextToNewThread(QThread *newThread);
};

#endif // MYGRAPHICWINDOWQT_H
