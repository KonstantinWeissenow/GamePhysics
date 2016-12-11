#ifndef RIGIDBODYSYSTEM_H
#define RIGIDBODYSYSTEM_H

class RigidBodySystem {
public:
	RigidBodySystem(Vec3 center, Vec3 size, int mass) {
		this->position = center;
		this->size = size;
		this->massInverse = 1.0f / mass;
		this->orientation = Quat(1, 0, 0, 0);

		double inertiaValues[16] = { 0 };
		inertiaValues[0] = (mass * 1.0f / 12.0f) * (size.y * size.y + size.z * size.z);
		inertiaValues[5] = (mass * 1.0f / 12.0f) * (size.x * size.x + size.z * size.z);
		inertiaValues[10] = (mass * 1.0f / 12.0f) * (size.x * size.x + size.y * size.y);
		inertiaValues[15] = 1.0;
		Mat4 inertiaTensor;
		inertiaTensor.initFromArray(inertiaValues);
		inertiaTensorInverseInitial = inertiaTensorInverse = inertiaTensor.inverse();
	}

	float massInverse;
	Vec3 position;
	Vec3 size;
	Vec3 velocity;
	Mat4 inertiaTensorInverseInitial; // I_0^-1
	Mat4 inertiaTensorInverse; // I^-1
	Quat orientation; // r
	Vec3 angularVelocity; // w
	Vec3 angularMomentum; // L
	Vec3 forceAccumulator; // F
	Vec3 torqueAccumulator; // q

	Mat4 getTransformMatrix() {
		return Mat4(XMMatrixScaling(size.x, size.y, size.z)) * (orientation.getRotMat() * Mat4(XMMatrixTranslation(position.x, position.y, position.z)));
	}

	void integrate(float deltaT) {
		if (massInverse == 0)
			return;

		// Euler step
		position = position + deltaT * velocity;
		velocity = velocity + deltaT * forceAccumulator * massInverse;

		// Orientation, angular properties
		Quat temp = Quat(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0) * orientation;
		orientation = orientation + (deltaT / 2.0f) * temp;
		orientation = orientation.unit();
		angularMomentum = angularMomentum + deltaT * torqueAccumulator;

		Mat4 rotMatrix = orientation.getRotMat();
		Mat4 rotMatrixTranspose = rotMatrix;
		rotMatrixTranspose.transpose();
		inertiaTensorInverse = rotMatrix * inertiaTensorInverseInitial * rotMatrixTranspose;
		angularVelocity = inertiaTensorInverse * angularMomentum;

		// Reset accumulators
		torqueAccumulator = forceAccumulator = Vec3();
	}
};

#endif