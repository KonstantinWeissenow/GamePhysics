#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
	: m_iIntegrator(EULER), m_bPrintedTestCase1(false) {
}

const char * MassSpringSystemSimulator::getTestCasesStr(){
	return "Demo1,Demo2,Demo3,Demo4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;

	TwEnumVal integratorsEV[] = { { EULER, "Euler" }, { LEAPFROG, "Leapfrog" }, { MIDPOINT, "Midpoint" } };
	TwType integratorType = TwDefineEnum("Integrator", integratorsEV, 3);
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", integratorType, &m_iIntegrator, NULL);
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

	if (m_iTestCase == 3) {
		addMassPoint(Vec3(), Vec3(), false);				// 0
		addMassPoint(Vec3(1.f, 0, 0), Vec3(), false);		// 1
		addMassPoint(Vec3(0, 0, 1.f), Vec3(), false);		// 2
		addMassPoint(Vec3(1.f, 0, 1.f), Vec3(), false);		// 3
		addMassPoint(Vec3(0, 1.f, 0), Vec3(), false);		// 4
		addMassPoint(Vec3(1.f, 1.f, 0), Vec3(), false);		// 5
		addMassPoint(Vec3(0, 1.f, 1.f), Vec3(), false);		// 6
		addMassPoint(Vec3(1.f, 1.f, 1.f), Vec3(), false);	// 7
		addSpring(0, 1, 1.f);
		addSpring(0, 2, 1.f);
		addSpring(1, 3, 1.f);
		addSpring(2, 3, 1.f);
		addSpring(4, 5, 1.f);
		addSpring(4, 6, 1.f);
		addSpring(5, 7, 1.f);
		addSpring(6, 7, 1.f);
		addSpring(0, 4, 1.f);
		addSpring(1, 5, 1.f);
		addSpring(2, 6, 1.f);
		addSpring(3, 7, 1.f);
		addMassPoint(Vec3(0.5f, -0.5f, 0.5f), Vec3(), false); // 8
		addMassPoint(Vec3(0.5f, 1.5f, 0.5f), Vec3(), false); // 9
		addMassPoint(Vec3(-0.5f, 0.5f, 0.5f), Vec3(), false); // 10
		addMassPoint(Vec3(1.5f, 0.5f, 0.5f), Vec3(), false); // 11
		addMassPoint(Vec3(0.5f, 0.5f, -0.5f), Vec3(), false); // 12
		addMassPoint(Vec3(0.5f, 0.5f, 1.5f), Vec3(), false); // 13
		float edgeLength = sqrt(0.5f);
		addSpring(0, 8, edgeLength);
		addSpring(1, 8, edgeLength);
		addSpring(2, 8, edgeLength);
		addSpring(3, 8, edgeLength);
		addSpring(4, 9, edgeLength);
		addSpring(5, 9, edgeLength);
		addSpring(6, 9, edgeLength);
		addSpring(7, 9, edgeLength);
		addSpring(0, 10, edgeLength);
		addSpring(2, 10, edgeLength);
		addSpring(4, 10, edgeLength);
		addSpring(6, 10, edgeLength);
		addSpring(1, 11, edgeLength);
		addSpring(3, 11, edgeLength);
		addSpring(5, 11, edgeLength);
		addSpring(7, 11, edgeLength);
		addSpring(0, 12, edgeLength);
		addSpring(1, 12, edgeLength);
		addSpring(4, 12, edgeLength);
		addSpring(5, 12, edgeLength);
		addSpring(2, 13, edgeLength);
		addSpring(3, 13, edgeLength);
		addSpring(6, 13, edgeLength);
		addSpring(7, 13, edgeLength);

		float diagLength = sqrt(2.f);
		addSpring(0, 7, diagLength);
		addSpring(1, 6, diagLength);
		addSpring(2, 5, diagLength);
		addSpring(3, 4, diagLength);

		addSpring(8, 9, 2.f);
		addSpring(10, 11, 2.f);
		addSpring(12, 13, 2.f);

		for (std::shared_ptr<MassPoint> massPoint : m_massPoints)
			massPoint->m_position += Vec3(0.f, 1.f, 0.f);

		applyExternalForce(Vec3(0, -9.8f, 0));
	}
	else {
		addMassPoint(Vec3(0, 0, 0), Vec3(-1.f, 0, 0), false);
		addMassPoint(Vec3(0, 2.f, 0), Vec3(1.f, 0, 0), false);
		addSpring(0, 1, 1.f);
		setStiffness(40.f);
		setMass(10.f);
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	for (std::shared_ptr<MassPoint> massPoint : m_massPoints) {
		massPoint->m_force = massPoint->m_forceTemp = m_externalForce;

		if (m_iTestCase == 3 && massPoint->m_position.y < -0.8f) {
			Vec3 groundForce = Vec3(0, 1.f, 0) * Vec3(0, massPoint->m_position.y, 0).squaredDistanceTo(Vec3(0, -0.8f, 0)) * 20000.f;
			massPoint->m_force += groundForce;
			massPoint->m_forceTemp += groundForce;
		}
	}

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	// Force time steps for some test cases
	if (m_iTestCase == 0)
		timeStep = 0.1f;

	// Console output
	if (m_iTestCase == 0 && !m_bPrintedTestCase1) {
		for (std::shared_ptr<MassPoint> massPoint : m_massPoints)
			printf("Old position: %f %f %f\nOld velocity: %f %f %f\n", massPoint->m_position.x, massPoint->m_position.y, massPoint->m_position.z, massPoint->m_velocity.x, massPoint->m_velocity.y, massPoint->m_velocity.z);
	}

	if (m_iIntegrator == EULER) {
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

		// Euler integration
		for (std::shared_ptr<MassPoint> massPoint : m_massPoints) {
			if (massPoint->m_isFixed)
				continue;

			massPoint->m_force /= m_fMass;

			massPoint->m_position += timeStep * massPoint->m_velocity;
			massPoint->m_velocity += timeStep * massPoint->m_force;
		}
	}
	else if (m_iIntegrator == MIDPOINT) {
		// Compute spring forces #1
		for (std::shared_ptr<Spring> spring : m_springs) {
			std::shared_ptr<MassPoint> point0 = spring->m_point0.lock();
			std::shared_ptr<MassPoint> point1 = spring->m_point1.lock();
			if (!point0 || !point1)
				continue;

			float currentLength = sqrt(point0->m_position.squaredDistanceTo(point1->m_position));
			point0->m_force += -m_fStiffness * (currentLength - spring->m_initialLength) * (point0->m_position - point1->m_position) / currentLength;
			point1->m_force += -m_fStiffness * (currentLength - spring->m_initialLength) * (point1->m_position - point0->m_position) / currentLength;
		}

		// Compute temporary position and velocity
		for (std::shared_ptr<MassPoint> massPoint : m_massPoints) {
			if (massPoint->m_isFixed)
				continue;

			massPoint->m_force /= m_fMass;

			massPoint->m_positionTemp = massPoint->m_position + (timeStep / 2.f) * massPoint->m_velocity;
			massPoint->m_velocityTemp = massPoint->m_velocity + (timeStep / 2.f) * massPoint->m_force;
		}

		// Compute spring forces #2
		for (std::shared_ptr<Spring> spring : m_springs) {
			std::shared_ptr<MassPoint> point0 = spring->m_point0.lock();
			std::shared_ptr<MassPoint> point1 = spring->m_point1.lock();
			if (!point0 || !point1)
				continue;

			float currentLength = sqrt(point0->m_positionTemp.squaredDistanceTo(point1->m_positionTemp));
			point0->m_forceTemp += -m_fStiffness * (currentLength - spring->m_initialLength) * (point0->m_positionTemp - point1->m_positionTemp) / currentLength;
			point1->m_forceTemp += -m_fStiffness * (currentLength - spring->m_initialLength) * (point1->m_positionTemp - point0->m_positionTemp) / currentLength;
		}

		// Final midpoint integration
		for (std::shared_ptr<MassPoint> massPoint : m_massPoints) {
			if (massPoint->m_isFixed)
				continue;

			massPoint->m_forceTemp /= m_fMass;

			massPoint->m_position = massPoint->m_position + timeStep * massPoint->m_velocityTemp;
			massPoint->m_velocity = massPoint->m_velocity + timeStep * massPoint->m_forceTemp;
		}
	}
	if (m_iIntegrator == LEAPFROG) {
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

		// Leapfrog integration
		for (std::shared_ptr<MassPoint> massPoint : m_massPoints) {
			if (massPoint->m_isFixed)
				continue;

			massPoint->m_force /= m_fMass;

			massPoint->m_velocity += timeStep * massPoint->m_force;
			massPoint->m_position += timeStep * massPoint->m_velocity;
		}
	}

	// Console output
	if (m_iTestCase == 0 && !m_bPrintedTestCase1) {
		for (std::shared_ptr<MassPoint> massPoint : m_massPoints)
			printf("New position: %f %f %f\nNew velocity: %f %f %f\n", massPoint->m_position.x, massPoint->m_position.y, massPoint->m_position.z, massPoint->m_velocity.x, massPoint->m_velocity.y, massPoint->m_velocity.z);
		m_bPrintedTestCase1 = true;
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
	this->m_externalForce = force;
}