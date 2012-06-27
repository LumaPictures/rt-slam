/**
 * \file osgWidget.hpp
 * \date 16/04/2012
 * \author Paul Molodowitch
 * File defining a QT widget for displaying an OpenSceneGraph view
 * \ingroup rtslam
 */

#ifndef OSG_WIDGET__HPP_
#define OSG_WIDGET__HPP_

#include "jafarConfig.h"

#if defined(HAVE_OSG) && defined(HAVE_QT4)

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>

#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>
#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtGui/QMainWindow>
#include <QtGui/QMdiSubWindow>
#include <QtGui/QMdiArea>

using Qt::WindowFlags;

#include <iostream>
#include <vector>

namespace jafar {
namespace rtslam {
namespace display {

class OsgWidget : public QGLWidget
{
    public:

        OsgWidget( QWidget * parent = 0, const char * name = 0, const QGLWidget * shareWidget = 0, WindowFlags f = 0 );

        virtual ~OsgWidget() {}

        osgViewer::GraphicsWindow* getGraphicsWindow() { return _gw.get(); }
        const osgViewer::GraphicsWindow* getGraphicsWindow() const { return _gw.get(); }

    protected:

        void init();

        virtual void resizeGL( int width, int height );
        virtual void keyPressEvent( QKeyEvent* event );
        virtual void keyReleaseEvent( QKeyEvent* event );
        virtual void mousePressEvent( QMouseEvent* event );
        virtual void mouseReleaseEvent( QMouseEvent* event );
        virtual void mouseMoveEvent( QMouseEvent* event );

        osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;
};



class ViewerQT : public osgViewer::Viewer, public OsgWidget
{
    public:

        ViewerQT(QWidget * parent = 0, const char * name = 0, const QGLWidget * shareWidget = 0, WindowFlags f = 0):
            OsgWidget( parent, name, shareWidget, f )
        {
            getCamera()->setViewport(new osg::Viewport(0,0,width(),height()));
            getCamera()->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(width())/static_cast<double>(height()), 1.0f, 10000.0f);
            getCamera()->setGraphicsContext(getGraphicsWindow());

            setThreadingModel(osgViewer::Viewer::SingleThreaded);

            connect(&_timer, SIGNAL(timeout()), this, SLOT(updateGL()));
            _timer.start(10);
        }

        virtual void paintGL()
        {
            frame();
        }

    protected:

        QTimer _timer;
};

class CompositeViewerQT : public osgViewer::CompositeViewer, public OsgWidget
{
    public:

        CompositeViewerQT(QWidget * parent = 0, const char * name = 0, const QGLWidget * shareWidget = 0, WindowFlags f = 0):
            OsgWidget( parent, name, shareWidget, f )
        {
            setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);

            connect(&_timer, SIGNAL(timeout()), this, SLOT(updateGL()));
            _timer.start(10);
        }

        virtual void paintGL()
        {
            frame();
        }

    protected:

        QTimer _timer;
};

} //namespace display
} //namespace rtslam
} //namespace jafar

#endif //HAVE_OSG && HAVE_QT4
#endif //OSG_WIDGET__HPP_

