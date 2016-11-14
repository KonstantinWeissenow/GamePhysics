#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class MassPoint {
public:
	Vec3 m_position, m_velocity, m_force;
	bool m_isFixed;

	MassPoint(Vec3 position, Vec3 velocity, bool isFixed) : m_position(position), m_velocity(velocity), m_isFixed(isFixed) {}
};

class Spring {
public:
	float m_initialLength;
	std::weak_ptr<MassPoint> m_point0;
	std::weak_ptr<MassPoint> m_point1;

	Spring(std::shared_ptr<MassPoint> point0, std::shared_ptr<MassPoint> point1, float initialLength) : m_point0(point0), m_point1(point1), m_initialLength(initialLength) {}
};

class MassSpringSystemSimulator:public Simulator{
public:
	//Construtors
	MassSpringSystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void setIntegrator(int integrator);
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Mass points and springs
	std::vector<std::shared_ptr<MassPoint>> m_massPoints;
	std::vector<std::shared_ptr<Spring>> m_springs;
};
#endif