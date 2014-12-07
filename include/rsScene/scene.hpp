#ifndef RSSCENE_SCENE_HPP_
#define RSSCENE_SCENE_HPP_

#include <osg/Depth>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osg/TextureCubeMap>
#include <osgDB/ReadFile>
#include <osgGA/OrbitManipulator>
#include <osgGA/StateSetManipulator>
#include <osgText/Text>
#include <osgShadow/ShadowedScene>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <rs/macros.hpp>
#include <rs/enum.hpp>
#include <rsRobots/robot.hpp>
#include <rsRobots/linkbot.hpp>
#include <rsScene/keyboardHandler.hpp>

extern osg::Node::NodeMask NOT_VISIBLE_MASK;
extern osg::Node::NodeMask RECEIVES_SHADOW_MASK;
extern osg::Node::NodeMask CASTS_SHADOW_MASK;
extern osg::Node::NodeMask IS_PICKABLE_MASK;
extern osg::Node::NodeMask VISIBLE_MASK;

namespace rsScene {

	class Scene : public keyboardHandler {
		// public functions
		public:
			Scene(void);
			virtual ~Scene(void);

			int drawGround(int, const double*, const double*, const double*, const double*);
			int drawMarker(int, const double*, const double*, const double*, int, std::string);
			int drawRobot(rsRobots::Robot*, int, const double*, const double*, bool);
			osgText::Text* getHUDText(void);
			std::string getTexturePath(void);
			void setDelete(int);
			void setGrid(bool, std::vector<double>);
			void setPauseText(int);
			int setupCamera(osg::GraphicsContext*, double, double);
			int setupScene(double, double);
			int setupViewer(osgViewer::Viewer*);
			void start(int);

		// virtual functions for inherited classes
		protected:
			virtual void keyPressed(int) {};

		// private functions
		private:
			//int draw_cubus(rsRobots::Cubus*, osg::Group*, const double*, const double*, bool, double*);
			int draw_linkbot(rsRobots::LinkbotT*, osg::Group*, const double*, const double*, bool, double*);
			//int draw_mobot(rsRobots::Mobot*, osg::Group*, const double*, const double*, bool, double*);
			//int draw_nxt(rsRobots::NXT*, osg::Group*, const double*, const double*, bool, double*);
			static void* graphics_thread(void*);

		// private data
		private:
			bool _thread;						// flag: thread is running
			bool _units;						// flag: SI (true) or customary (false)
			int _deleting;						// temp variable for deleting robots
			osg::Camera *_camera;				// camera to view scene
			osg::Group *_root;					// root node of scene
			osg::Group *_staging;				// temp variable for adding objects
			osgViewer::Viewer *_viewer;			// viewer
			osgShadow::ShadowedScene *_scene;	// shadow root of scene
			std::string _tex_path;				// texture path
			std::vector<double> _grid;			// grid
			MUTEX_T _thread_mutex;				// mutex: thread running state
			THREAD_T _osgThread;				// thread: graphics window
	};

} // namespace rsScene

#endif // RSSCENE_SCENE_HPP_

