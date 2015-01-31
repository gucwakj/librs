#ifndef RSSCENE_SCENE_HPP_
#define RSSCENE_SCENE_HPP_

#include <osg/Depth>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osg/TextureCubeMap>
#include <osgDB/ReadFile>
#include <osgGA/OrbitManipulator>
#include <osgGA/StateSetManipulator>
#include <osgText/Text>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgUtil/SmoothingVisitor>
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
	typedef osg::Group Ground;
	typedef osg::Group Robot;
}

namespace rsScene {

	// enum - Levels
	enum Levels {
		OUTDOORS,
		BOARD,
		ROBOPLAY2013,
		ROBOPLAY2014,
		NUM_LEVELS
	};

	// class - Scene
	class Scene : public keyboardHandler {
		// public functions
		public:
			Scene(void);
			virtual ~Scene(void);

			int addChild(void);
			void addHighlight(int, bool = 1);
			int deleteChild(int);
			void drawConnector(rsRobots::ModularRobot*, Robot*, int, int, double, int, int);
			Ground* drawGround(int, const double*, const double*, const double*, const double*);
			int drawMarker(int, const double*, const double*, const double*, int, std::string);
			Robot* drawRobot(rsRobots::Robot*, int, const double*, const double*, bool);
			osgText::Text* getHUDText(void);
			std::string getTexturePath(void);
			void setDelete(int);
			void setGrid(bool, std::vector<double>);
			void setHighlight(bool);
			void setLabel(bool);
			void setPauseText(int);
			int setupCamera(osg::GraphicsContext*, double, double);
			int setupScene(double, double, bool);
			int setupViewer(osgViewer::Viewer*);
			void start(int);
			void toggleHighlight(osg::Group*, osg::Node*);
			void toggleLabel(osg::Group*, osg::Node*);

		// virtual functions for inherited classes
		protected:
			virtual void keyPressed(int) {};

		// private functions
		private:
			osg::Material* create_material(osg::Vec4);
			void draw_grid(void);
			void draw_global_hud(double, double, bool);
			//int draw_cubus(rsRobots::Cubus*, osg::Group*, const double*, const double*, bool, double*);
			void draw_robot_linkbot(rsRobots::LinkbotT*, Robot*, const double*, const double*, bool, double*);
			void draw_robot_linkbot_conn(rsRobots::LinkbotT*, Robot*, int, int, double, int, int);
			//int draw_mobot(rsRobots::Mobot*, osg::Group*, const double*, const double*, bool, double*);
			//int draw_nxt(rsRobots::NXT*, osg::Group*, const double*, const double*, bool, double*);
			void draw_scene_outdoors(double, double, bool);
			void draw_scene_board(double, double, bool);
			static void* graphics_thread(void*);

		// private data
		private:
			bool _highlight;					// flag: enable object highlight on click
			bool _label;						// flag: enable object hud on click
			bool _thread;						// flag: thread is running
			bool _units;						// flag: SI (true) or customary (false)
			int _deleting;						// temp variable for deleting robots
			int _level;							// level to load
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

