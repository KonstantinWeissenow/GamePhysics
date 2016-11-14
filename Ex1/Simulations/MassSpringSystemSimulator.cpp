#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {

}

const char * MassSpringSystemSimulator::getTestCasesStr(){
	return "Demo1,Demo2,Demo3,Demo4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset() {
	
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for (std::shared_ptr<MassPoint> massPoint : m_massPoints) {
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(massPoint->m_position, 0.1f);
	}
	
	DUC->beginLine();
	for (std::shared_ptr<Spring> spring : m_springs) {
		std::shared_ptr<MassPoint> point0 = spring->m_point0.lock();
		std::shared_ptr<MassPoint> point1 = spring->m_point1.lock();

		if (point0 && point1)
			DUC->drawLine(point0->m_position, Vec3(1.f), point1->m_position, Vec3(1.f));
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;

	m_springs.clear();
	m_massPoints.clear();

	switch (m_iTestCase) {
	default:
	case 0:
		addMassPoint(Vec3(0, 0, 0), Vec3(-1.f, 0, 0), false);
		addMassPoint(Vec3(0, 2.f, 0), Vec3(1.f, 0, 0), false);
		addSpring(0, 1, 1.f);
		setStiffness(40.f);
		setMass(10.f);
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	for (std::shared_ptr<MassPoint> massPoint : m_massPoints)
		massPoint->m_force = m_externalForce;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	// Force time steps for some test cases
	if (m_iTestCase == 0)
		timeStep = 0.1f;

	// Compute spring forces
	for (std::shared_ptr<Spring> spring : m_springs) {
		std::shared_ptr<MassPoint> point0 = spring->m_point0.lock();
		std::shared_ptr<MassPoint> point1 = spring->m_point1.lock();
		if (!point0 || !point1)
			continue;

		float currentLength = sqrt(point0->m_position.squaredDistanceTo(point1->m_position));
		point0->m_force += -m_fStiffness * (currentLength - spring->m_initialLength) * (point0->m_position - point1->m_position) / currentLength;
		point1->m_force += -m_fStiffness * (currentLength - spring->m_initialLength) * (point1->m_position - point0->m_position) / currentLength;
	}

	// Integrate mass points
	for (std::shared_ptr<MassPoint> massPoint : m_massPoints) {
		if (massPoint->m_isFixed)
			continue;

		massPoint->m_force /= m_fMass;

		massPoint->m_position += timeStep * massPoint->m_velocity;
		massPoint->m_velocity += timeStep * massPoint->m_force;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y) {

}

void MassSpringSystemSimulator::onMouse(int x, int y) {

}

void MassSpringSystemSimulator::setIntegrator(int integrator) {
	this->m_iIntegrator = integrator;
}

void MassSpringSystemSimulator::setMass(float mass) {
	this->m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	this->m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	m_massPoints.push_back(std::shared_ptr<MassPoint>(new MassPoint(position, velocity, isFixed)));
	return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	if (masspoint1 < 0 || masspoint1 >= m_massPoints.size() || masspoint2 < 0 || masspoint2 >= m_massPoints.size())
		return;

	std::shared_ptr<MassPoint> point0 = m_massPoints.at(masspoint1);
	std::shared_ptr<MassPoint> point1 = m_massPoints.at(masspoint2);

	m_springs.push_back(std::shared_ptr<Spring>(new Spring(point0, point1, initialLength)));
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	if (index < 0 || index >= m_massPoints.size())
		return Vec3();
	return m_massPoints.at(index)->m_position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	if (index < 0 || index >= m_massPoints.size())
		return Vec3();
	return m_massPoints.at(index)->m_velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	this->m_externalForce += force;
}