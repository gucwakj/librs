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

			int addChild(void);
			int drawGround(int, const double*, const double*, const double*, const double*);
			int drawMarker(int, const double*, const double*, const double*, int, std::string);
			int drawRobot(rsRobots::Robot*, int, const double*, const double*, int);
			osgText::Text* getHUDText(void);
			std::string getTexPath(void);
			virtual void keyPressed(int);
			void setPauseText(int);
			int setupCamera(osg::GraphicsContext*, double, double);
			int setupScene(double, double);
			int setupViewer(osgViewer::Viewer*);
			int stageForDelete(int);
			void start(int);

		// private functions
		private:
			//int draw_cubus(rsRobots::Cubus*, const double*, const double*, int, double*);
			int draw_linkbot(rsRobots::LinkbotT*, const double*, const double*, int, double*);
			//int draw_mobot(rsRobots::Mobot*, const double*, const double*, int, double*);
			//int draw_nxt(rsRobots::NXT*, const double*, const double*, int, double*);
			static void* graphics_thread(void*);

		// private data
		private:
			struct Robot {
				osg::Group *robot;
				osg::ShapeDrawable *led;
			};

			std::vector<Robot*> _robot;			// robots

			bool _thread;						// flag: thread is running
			bool _units;						// flag: SI (true) or customary (false)
			double _grid[7];					// grid spacing (tics, major, total)
			int _ending;						// temp variable for deleting robots
			osg::Camera *_camera;				// camera to view scene
			osg::Group *_root;					// root node of scene
			osg::Group *_staging;				// temp variable for adding objects
			osgViewer::Viewer *_viewer;			// viewer
			osgShadow::ShadowedScene *_scene;	// shadow root of scene
			std::string _tex_path;				// texture path
			MUTEX_T _thread_mutex;				// mutex: thread running state
			THREAD_T _osgThread;				// thread: graphics window
	};

} // namespace rsScene

#endif // RSSCENE_SCENE_HPP_

