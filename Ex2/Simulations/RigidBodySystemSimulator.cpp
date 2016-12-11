#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

#define RESTCOEFF 0.8f

RigidBodySystemSimulator::RigidBodySystemSimulator() {
}

const char * RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset() {
	m_rigidBodies.clear();
	m_externalForce = Vec3();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for (std::shared_ptr<RigidBodySystem> rigidBody : m_rigidBodies) {
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawRigidBody(rigidBody->getTransformMatrix());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;

	reset();

	switch (testCase) {
	default:
	case 0:
	case 1:
		addRigidBody(Vec3(), Vec3(1.0f, 0.6f, 0.5f), 2);
		m_rigidBodies[0]->orientation = Quat(Vec3(0, 0, 1), (float)(M_PI)*0.5f);
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0));
		break;
	case 2:
		addRigidBody(Vec3(), Vec3(1.0f, 0.6f, 0.5f), 2);
		setOrientationOf(0, Quat(Vec3(1, 0, 1), (float)(M_PI)*0.9f));
		setVelocityOf(0, Vec3(0, 1.0f, 0));
		addRigidBody(Vec3(-0.3f, 2.0f, 0), Vec3(1.0f, 0.6f, 0.5f), 2);
		setOrientationOf(1, Quat(Vec3(0, 1, 1), (float)(M_PI)*0.4f));
		setVelocityOf(1, Vec3(0, -1.0f, 0));
		break;
	case 3:

		srand(104657893);
		float heightCounter = 2.0f;

		for (int i = 0; i < 20; ++i) {
			addRigidBody(Vec3((((float)rand())/RAND_MAX-0.5f)*10.0f, heightCounter, (((float)rand())/RAND_MAX-0.5f)*1.0f), Vec3(1.0f, 1.0f, 1.0f), 2);
			setOrientationOf(i, Quat(Vec3(1, 1, 1), ((float)rand() / RAND_MAX)));
			heightCounter += 1.0f;
		}

		addRigidBody(Vec3(), Vec3(10.0f, 1.0f, 1.0f), 10);
		m_rigidBodies[m_rigidBodies.size()-1]->massInverse = 0;
		m_rigidBodies[m_rigidBodies.size()-1]->inertiaTensorInverse = Mat4(0);

		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 3) {
		// Gravity for last test case:
		for (std::shared_ptr<RigidBodySystem> rigidBody : m_rigidBodies) {
			if (rigidBody->massInverse != 0)
				rigidBody->forceAccumulator += Vec3(0, -9.8f, 0);
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	if (m_iTestCase == 0)
		timeStep = 2.0f;
	else if (m_iTestCase == 1)
		timeStep = 0.01f;

	// Collision test and response
	for (int i = 0; i < m_rigidBodies.size(); ++i) {
		for (int j = i + 1; j < m_rigidBodies.size(); ++j) {
			CollisionInfo colInfo = checkCollisionSAT(m_rigidBodies[i]->getTransformMatrix(), m_rigidBodies[j]->getTransformMatrix());
			if (!colInfo.isValid)
				continue;

			Vec3 n = colInfo.normalWorld;
			Vec3 velA = m_rigidBodies[i]->velocity + cross(m_rigidBodies[i]->angularVelocity, colInfo.collisionPointWorld - m_rigidBodies[i]->position);
			Vec3 velB = m_rigidBodies[j]->velocity + cross(m_rigidBodies[j]->angularVelocity, colInfo.collisionPointWorld - m_rigidBodies[j]->position);
			Vec3 vRel = velA - velB;
			float contactFactor = dot(n, vRel);
			if (contactFactor < 0) {
				Vec3 x_a = colInfo.collisionPointWorld - m_rigidBodies[i]->position;
				Vec3 x_b = colInfo.collisionPointWorld - m_rigidBodies[j]->position;
				Mat4 I_a = m_rigidBodies[i]->inertiaTensorInverse;
				Mat4 I_b = m_rigidBodies[j]->inertiaTensorInverse;

				float J = -(1 + RESTCOEFF) * contactFactor /
					(m_rigidBodies[i]->massInverse + m_rigidBodies[j]->massInverse +
					dot(cross(I_a*cross(x_a, n), x_a) + cross(I_b*cross(x_b, n), x_b), n));

				m_rigidBodies[i]->velocity += J*n*m_rigidBodies[i]->massInverse;
				m_rigidBodies[j]->velocity -= J*n*m_rigidBodies[j]->massInverse;

				m_rigidBodies[i]->angularMomentum += cross(x_a, J*n);
				m_rigidBodies[j]->angularMomentum -= cross(x_b, J*n);
			}
		}
	}

	// Time integration
	for (std::shared_ptr<RigidBodySystem> rigidBody : m_rigidBodies) {
		rigidBody->integrate(timeStep);
	}

	if (!printedTestCase1) {
		printedTestCase1 = true;

		Vec3 linVel = m_rigidBodies[0]->velocity;
		Vec3 angVel = m_rigidBodies[0]->angularVelocity;
		Vec3 x = Vec3(-0.3f, -0.5f, -0.25f) - m_rigidBodies[0]->position;
		Vec3 worldVel = linVel + cross(angVel, x);
		printf("Linear velocity: (%f, %f, %f)\nAngular velocity: (%f, %f, %f)\nWorld space velocity of point: (%f, %f, %f)\n", linVel.x, linVel.y, linVel.z, angVel.x, angVel.y, angVel.z, worldVel.x, worldVel.y, worldVel.z);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y) {
	// Questionable mouse interaction for demo 2 (What are we supposed to do with only one object in terms of interaction?)
	if (m_iTestCase == 1) {
		m_rigidBodies[0]->angularMomentum = 0.01f * normalize(Vec3(((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX)));
	}
}

void RigidBodySystemSimulator::onMouse(int x, int y) {

}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return m_rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	if (i >= m_rigidBodies.size() || i < 0)
		return Vec3();
	return m_rigidBodies[i]->position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	if (i >= m_rigidBodies.size() || i < 0)
		return Vec3();
	return m_rigidBodies[i]->velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	if (i >= m_rigidBodies.size() || i < 0)
		return Vec3();
	return m_rigidBodies[i]->angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	if (i >= m_rigidBodies.size() || i < 0)
		return;
	m_rigidBodies[i]->forceAccumulator += force;
	m_rigidBodies[i]->torqueAccumulator += cross(loc - m_rigidBodies[i]->position, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	m_rigidBodies.push_back(std::shared_ptr<RigidBodySystem>(new RigidBodySystem(position, size, mass)));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	if (i >= m_rigidBodies.size() || i < 0)
		return;
	m_rigidBodies[i]->orientation = orientation.unit();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	if (i >= m_rigidBodies.size() || i < 0)
		return;
	m_rigidBodies[i]->velocity = velocity;
}

Vec3 RigidBodySystemSimulator::cross(Vec3 a, Vec3 b) {
	return Vec3(a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x);
}

float RigidBodySystemSimulator::dot(Vec3 a, Vec3 b) {
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 RigidBodySystemSimulator::normalize(Vec3 v) {
	float l = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
	return (1.0f / l) * v;
}