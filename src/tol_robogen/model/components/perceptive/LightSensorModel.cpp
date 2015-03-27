/*
 * LightSensorModel.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: elte
 */

#include <tol_robogen/model/components/perceptive/LightSensorModel.h>

#include <sdf_builder/sensor/Sensor.h>

#include <tol_robogen/model/io/Sensor.h>

#include <sstream>

namespace sb = sdf_builder;

namespace tol_robogen {

const float LightSensorModel::MASS = 2;
const float LightSensorModel::SENSOR_BASE_WIDTH = 34;
const float LightSensorModel::SENSOR_BASE_THICKNESS = 1.5;
const float LightSensorModel::SENSOR_DISPLACEMENT = 8;

LightSensorModel::LightSensorModel(std::string id, ConfigurationPtr conf,
		bool internalSensor):
	Component(id, conf),
	internalSensor_(internalSensor)
{}

LightSensorModel::~LightSensorModel() {}

bool LightSensorModel::initModel() {
	this->sensorRoot_ = this->createLink(B_SENSOR_BASE_ID);
	this->sensorRoot_->makeBox(inGrams(MASS), inMm(SENSOR_BASE_THICKNESS),
			inMm(SENSOR_BASE_WIDTH), inMm(SENSOR_BASE_WIDTH));

	// We implement the light sensor as a 1px camera
	auto cam = this->createSensor("light_sensor", "camera");

	std::stringstream camDetails;
	camDetails << "<camera>"
			"<image>"
			"<width>1</width>"
			"<height>1</height>"
			"<format>R8G8B8</format>"
			"</image>"
			// TODO Decide cutoff; between 0.01mm and 50m is quite
			// arbitrary.
			"<clip><near>" << inMm(0.01) << "</near>"
			"<far>" << inMm(50000) << "</far></clip>"
			"</camera>";

	// TODO Noise values

	cam->addString(camDetails.str());
	this->sensorRoot_->addPosable(cam);

	IOPtr io(new Sensor(id_, 0, "light", cam, sensorRoot_));
	this->addIO(io);

	return true;
}

sdf_builder::LinkPtr LightSensorModel::getRoot() {
	return this->sensorRoot_;
}

sdf_builder::LinkPtr LightSensorModel::getSlot(unsigned int i) {
	this->checkSlot(i);
	return this->sensorRoot_;
}

sdf_builder::Vector3 LightSensorModel::getSlotPosition(unsigned int i) {
	this->checkSlot(i);

	auto curPos = this->sensorRoot_->position();
	auto slotAxis = this->getSlotAxis(i);
	return curPos + slotAxis * (inMm(SENSOR_BASE_THICKNESS) / 2.0);
}

sdf_builder::Vector3 LightSensorModel::getSlotOrientation(unsigned int i) {
	this->checkSlot(i);

	auto quat = this->sensorRoot_->rotation();
	auto axis = sb::Vector3(0, 1, 0);
	return quat * axis;
}

sdf_builder::Vector3 LightSensorModel::getSlotAxis(unsigned int i) {
	this->checkSlot(i);

	auto quat = this->sensorRoot_->rotation();
	auto axis = sb::Vector3(-1, 0, 0);
	return quat * axis;
}

void LightSensorModel::checkSlot(unsigned int i) {
	if (i != SLOT_A) {
		std::cerr << "[LightSensorModel] Invalid slot: " << i << std::endl;
		throw std::runtime_error("Sensor error");
	}
}

} /* namespace tol_robogen */
