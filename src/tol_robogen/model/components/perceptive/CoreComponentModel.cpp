/*
 * TODO License
 */
#include <tol_robogen/model/components/perceptive/CoreComponentModel.h>

#include <sdf_builder/sensor/Sensor.h>

#include <tol_robogen/model/io/Sensor.h>

namespace sb = sdf_builder;

namespace tol_robogen {

// mass of just the brick
const float CoreComponentModel::BRICK_MASS = 14.9;
// mass of brick with electronics (including battery)
const float CoreComponentModel::CORE_MASS = 55.4;
const float CoreComponentModel::WIDTH = 46.5;


CoreComponentModel::CoreComponentModel(std::string id, ConfigurationPtr conf, bool hasSensors) :
		Component(id, conf), hasSensors_(hasSensors)
{}

CoreComponentModel::~CoreComponentModel()
{}

bool CoreComponentModel::initModel() {
	coreComponent_ = this->createLink(B_CORE_COMPONENT_ID);
	double mass = inGrams(hasSensors_ ? CORE_MASS : BRICK_MASS);

	// Give box geometry
	double width = inMm(WIDTH);
	coreComponent_->makeBox(mass, width, width, width);

	if (hasSensors_) {
		// Add IMU;
		sb::SensorPtr imu(new sb::Sensor("core_imu", "imu"));
		imu->updateRate = 1.0 / conf_->actuationTime;

		// The robot will be driven by the core component's update,
		// so no need to keep the sensor always on!
		imu->alwaysOn = false;

		// TODO Specify noise parameters

		// Add the SDF sensor object to the core link
		coreComponent_->addPosable(imu);

		// Register all six outputs of the IMU sensor with
		// the neural network.
		for (unsigned int i = 0; i < 6; ++i) {
			IOPtr io(new Sensor(id_, i, "imu", imu, coreComponent_, i == 0));
			this->addIO(io);
		}
	}

	// Already positioned at origin
	return true;
}

sb::LinkPtr CoreComponentModel::getRoot() {
	return coreComponent_;
}

sb::LinkPtr CoreComponentModel::getSlot(unsigned int /*i*/) {
	return coreComponent_;
}

sb::Vector3 CoreComponentModel::getSlotPosition(unsigned int i) {

	if (i > 5) {
		std::cerr << "[CoreComponentModel] Invalid slot: " << i << std::endl;
		assert(i <= 5);
	}

	sb::Vector3 curPos = this->getRootPosition();
	sb::Vector3 slotAxis = this->getSlotAxis(i) * inMm(WIDTH) / 2;
	curPos = curPos + slotAxis;

	return curPos;

}

sb::Vector3 CoreComponentModel::getSlotAxis(unsigned int i) {

	if (i > 5) {
		std::cerr << "[CoreComponentModel] Invalid slot: " << i << std::endl;
		assert(i <= 5);
	}

	sb::Quaternion quat = this->getRootAttitude();
	sb::Vector3 axis;

	if (i == LEFT_FACE_SLOT) {

		// Left face
		axis = sb::Vector3(-1, 0, 0);

	} else if (i == RIGHT_FACE_SLOT) {

		// Right face
		axis = sb::Vector3(1, 0, 0);

	} else if (i == TOP_FACE_SLOT) {

		// Top face
		axis = sb::Vector3(0, 0, 1);

	} else if (i == BOTTOM_FACE_SLOT) {

		// Bottom face
		axis = sb::Vector3(0, 0, -1);

	} else if (i == FRONT_FACE_SLOT) {

		// Front face
		axis = sb::Vector3(0, -1, 0);

	} else if (i == BACK_FACE_SLOT) {

		// Back face
		axis = sb::Vector3(0, 1, 0);

	}

//	std::cerr << quat.w() << ' ' << quat.x() << ' ' << quat.y() << ' ' << quat.z() << std::endl;
//	std::cerr << axis << std::endl;
//	sb::Vector3 tst = quat * axis;
//	std::cerr << tst << std::endl;
	return quat * axis;

}

sb::Vector3 CoreComponentModel::getSlotOrientation(unsigned int i) {

	if (i > 5) {
		std::cout << "[CoreComponentModel] Invalid slot: " << i << std::endl;
		assert(i <= 5);
	}

	sb::Quaternion quat = this->getRootAttitude();
	sb::Vector3 tangent;

	if (i == LEFT_FACE_SLOT) {

		// Left face
		tangent = sb::Vector3(0, 1, 0);

	} else if (i == RIGHT_FACE_SLOT) {

		// Right face
		tangent = sb::Vector3(0, 1, 0);

	} else if (i == TOP_FACE_SLOT) {

		// Top face
		tangent = sb::Vector3(1, 0, 0);

	} else if (i == BOTTOM_FACE_SLOT) {

		// Bottom face
		tangent = sb::Vector3(1, 0, 0);

	} else if (i == FRONT_FACE_SLOT) {

		// Front face
		tangent = sb::Vector3(0, 0, 1);

	} else if (i == BACK_FACE_SLOT) {

		// Back face
		tangent = sb::Vector3(0, 0, 1);

	}

	return quat * tangent;

}

}
