#ifndef GRAPHICS_HPP_
#define GRAPHICS_HPP_

#include "robot.hpp"

#include <ode/ode.h>
#include <osg/Billboard>
#include <osg/ClearNode>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Notify>
#include <osg/Object>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/TexGen>
#include <osg/TexMat>
#include <osg/Texture2D>
#include <osg/Transform>
#include <osg/TextureCubeMap>
#include <osg/VertexProgram>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgGA/OrbitManipulator>
#include <osgGA/StateSetManipulator>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgUtil/CullVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <OpenThreads/Thread>
//#include <tinyxml2.h>

extern osg::Node::NodeMask NOT_VISIBLE_MASK;
extern osg::Node::NodeMask RECEIVES_SHADOW_MASK;
extern osg::Node::NodeMask CASTS_SHADOW_MASK;
extern osg::Node::NodeMask IS_PICKABLE_MASK;
extern osg::Node::NodeMask VISIBLE_MASK;

struct Ground;
class RoboSim;
class CMobot;
class CLinkbotT;
class CNXT;
class Cubus;
//extern RoboSim *g_sim;

struct Drawing {
	double p1[3], p2[3], c[4];
	int i, type;
	std::string str;
};

/**********************************************************
	Graphics drawing class
 **********************************************************/
class Graphics {
	// public api
	public:
		Graphics(RoboSim*);
		~Graphics(void);

		int addChild(void);
		int drawGround(Ground*);
		int drawMarker(Drawing*);
		int drawRobot(Robot*, XMLRobot*, int, int);
		osgText::Text* getHUDText(void);
		std::string getTexPath(void);
		int getUnits(void);
		//void readXML(tinyxml2::XMLDocument*);
		int setupCamera(osg::GraphicsContext*, osgViewer::Viewer*, double, double);
		int setupScene(osgViewer::Viewer*, double, double);
		int setupViewer(osgViewer::Viewer*);
		int stageForDelete(int);
		void start(int);

	// private functions
	private:
		int draw(Cubus*, int, double*);			// draw cubus
		int draw(CLinkbotT*, XMLRobot*, int, double*);		// draw linkbot
		int draw(CMobot*, int, double*);		// draw mobot
		int draw(CNXT*, int, double*);			// draw nxt
		static void* graphics_thread(void*);	// thread for graphics objects

	// private data
	private:
		// enumeration of drawing objects
		typedef enum drawing_objects_e {
			DOT,
			LINE,
			TEXT,
			NUM_TYPES
		} drawingObjects_t;

		struct GRobot {
			osg::Group *robot;
			osg::ShapeDrawable *led;
		};

		RoboSim *_sim;						// RoboSim instance
		std::vector<Drawing*> _drawings;	// all graphics objects
		std::vector<GRobot*> _robots;		// all robot objects
		double _grid[7];					// grid spacing (tics, major, total)
		int _ending;						// temp variable for deleting robots
		int _graphics;						// flag for graphics
		int _viewer;						// flag for viewer
		int _us;							// us units or not
		std::string _tex_path;				// texture path
		osg::Camera *_camera;				// camera to view scene
		osg::Group *_root;					// root node of scene
		osg::Group *_staging;				// temp variable for adding robots
		osgViewer::Viewer *_view;			// viewer
		osgShadow::ShadowedScene *_scene;	// shadow root of scene
		COND_T _graphics_cond;				// condition for graphics
		MUTEX_T _graphics_mutex;			// mutex for graphics existence
		MUTEX_T _viewer_mutex;				// mutex for viewer running state
		THREAD_T _osgThread;				// thread to hold graphics
};

/**********************************************************
	MoveEarthySkyWithEyePointTransform
 **********************************************************/
class MoveEarthySkyWithEyePointTransform : public osg::Transform {
	public:
		virtual bool computeLocalToWorldMatrix(osg::Matrix &matrix, osg::NodeVisitor *nv) const;
		virtual bool computeWorldToLocalMatrix(osg::Matrix &matrix, osg::NodeVisitor *nv) const;
};

/**********************************************************
	TexMatCallback
 **********************************************************/
class TexMatCallback : public osg::NodeCallback {
	public:
		TexMatCallback(osg::TexMat &tm) : _texMat(tm) {}
		virtual void operator()(osg::Node *node, osg::NodeVisitor *nv);
	private:
		osg::TexMat& _texMat;
};

/**********************************************************
	Keyboard Event Handler
 **********************************************************/
class keyboardEventHandler : public osgGA::GUIEventHandler {
	public:
		keyboardEventHandler(osgText::Text *text);
		virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
		virtual void accept(osgGA::GUIEventHandlerVisitor &v);
	private:
		osgText::Text *_text;
};

/**********************************************************
	Mouse Pick Event Handler
 **********************************************************/
class pickHandler : public osgGA::GUIEventHandler {
	public:
		pickHandler(void);
		bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
		void pick(const osgGA::GUIEventAdapter &ea, osgViewer::Viewer *viewer);
	private:
		float _mx, _my;
};

/**********************************************************
	Ground Node Callback
 **********************************************************/
class groundNodeCallback : public osg::NodeCallback {
	public:
		groundNodeCallback(Ground *ground);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		Ground *_ground;
};

/**********************************************************
	Linkbot Node Callback
 **********************************************************/
class CLinkbotT;
class linkbotNodeCallback : public osg::NodeCallback {
	public:
		linkbotNodeCallback(CLinkbotT *robot, osg::ShapeDrawable*);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		CLinkbotT *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

/**********************************************************
	Mobot Node Callback
 **********************************************************/
class CMobot;
class mobotNodeCallback : public osg::NodeCallback {
	public:
		mobotNodeCallback(CMobot *robot, osg::ShapeDrawable*);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		CMobot *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

/**********************************************************
	NXT Node Callback
 **********************************************************/
class CNXT;
class nxtNodeCallback : public osg::NodeCallback {
	public:
		nxtNodeCallback(CNXT *robot, osg::ShapeDrawable*);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		CNXT *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

/**********************************************************
	Cubus Node Callback
 **********************************************************/
class Cubus;
class cubusNodeCallback : public osg::NodeCallback {
	public:
		cubusNodeCallback(Cubus *robot, osg::ShapeDrawable*);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		Cubus *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

#endif // GRAPHICS_HPP_

