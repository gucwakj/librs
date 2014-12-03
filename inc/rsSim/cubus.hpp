#ifndef CUBUS_HPP_
#define CUBUS_HPP_

#include "robosim.hpp"

class DLLIMPORT Cubus : public ModularRobot {
#ifdef ENABLE_GRAPHICS
		friend class cubusNodeCallback;
		friend class Graphics;
#endif // ENABLE_GRAPHICS

	// public api
	public:
		Cubus(void);
		virtual ~Cubus(void);

		int driveForeverNB(void);
		int driveForwardNB(double);
		int drivexyTo(double, double, double, double);
		int getJointAngles(double&, double&, double&, double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&, double&, double&, double&);
		int getJointSpeeds(double&, double&, double&, double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&, double&, double&, double&);
		int move(double, double, double, double, double, double);
		int moveNB(double, double, double, double, double, double);
		int moveTo(double, double, double, double, double, double);
		int moveToNB(double, double, double, double, double, double);
		int setJointSpeeds(double, double, double, double, double, double);
		int setJointSpeedRatios(double, double, double, double, double, double);
		int turnLeftNB(double, double, double);
		int turnRightNB(double, double, double);

	// inherited functions from ModularRobot class
	private:
		virtual int addConnector(int, int, double);
		virtual int build(XMLRobot*, dMatrix3, double*, dBodyID, XMLConn*);
		virtual int fixBodyToConnector(dBodyID, int);
		virtual int fixConnectorToBody(int, dBodyID, int = -1);
		virtual int getConnectorParams(int, int, dMatrix3, double*);
		virtual int getFaceParams(int, dMatrix3, double*);

	// inherited functions from Robot class
	private:
		virtual int build(XMLRobot*, int = 1);
		virtual int buildIndividual(double, double, double, dMatrix3, double*);
		virtual double getAngle(int);
		virtual int initParams(int, int);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

	// private functions
	private:
		int build_body(double, double, double, dMatrix3, double);
		int build_face(int, double, double, double, dMatrix3, double);
		int build_simple(Connector*, int face, int = -1, int = -1);

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			BODY,
			FACE1,
			FACE2,
			FACE3,
			FACE4,
			FACE5,
			FACE6,
			NUM_PARTS
		};

		// dimensions
		double	_face_depth,
				_face_radius;
};

class DLLIMPORT CubusGroup : public Group<Cubus> {
	public:
		CubusGroup(void) : Group<Cubus>() {};
		virtual ~CubusGroup(void) {};

		inline int move(double, double, double, double, double, double);
		inline int moveNB(double, double, double, double, double, double);
		inline int moveTo(double, double, double, double, double, double);
		inline int moveToNB(double, double, double, double, double, double);
		inline int setJointSpeeds(double, double, double, double, double, double);
		inline int setJointSpeedRatios(double, double, double, double, double, double);
};
#include "cubusgroup.tpp"

// simulation
//extern RoboSim *g_sim;

#endif // CUBUS_HPP_

