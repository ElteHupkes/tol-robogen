/*
 * SensorFactory.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/sensors/SensorFactory.h>
#include <tol_robogen/gazebo/sensors/Sensors.h>

#include <stdexcept>
#include <iostream>

namespace gz = gazebo;

namespace tol_robogen {
namespace gazebo {

SensorFactory::SensorFactory()
{}

SensorFactory::~SensorFactory()
{}

SensorPtr SensorFactory::create(sdf::ElementPtr sensor, ::gazebo::physics::ModelPtr model) {
	auto typeParam = sensor->GetAttribute("type");
	auto nameParam = sensor->GetAttribute("ref");
	auto partIdParam = sensor->GetAttribute("part_id");
	auto ioIdParam = sensor->GetAttribute("io_id");
	auto linkParam = sensor->GetAttribute("link");

	if (!typeParam || !nameParam || !partIdParam || !ioIdParam || !linkParam) {
		std::cerr << "Sensor is missing required attributes." << std::endl;
		throw std::runtime_error("Sensor error");
	}

	auto partId = partIdParam->GetAsString();
	unsigned int ioId;
	ioIdParam->Get(ioId);

	auto sensorName = nameParam->GetAsString();
	auto linkName = linkParam->GetAsString();
	auto link = model->GetLink(linkName);
	if (!link) {
		std::cerr << "Link '" << linkName << "' for sensor '"
				<< sensorName << "' is not present in model." << std::endl;
		throw std::runtime_error("Sensor error");
	}

	std::string scopedName = link->GetScopedName(true) + "::" + sensorName;
	gz::sensors::SensorPtr gzSensor = gz::sensors::get_sensor(scopedName);

	if (!gzSensor) {
		std::cerr << "Sensor with scoped name '" << scopedName
				<< "' could not be found." << std::endl;
		throw std::runtime_error("Sensor error");
	}

	auto type = typeParam->GetAsString();
	if ("imu" == type) {
		SensorPtr out(new ImuSensor(model, gzSensor, partId, ioId));
		return out;
	} else {
		std::cerr << "Sensor type '" << type << "' is not supported." << std::endl;
	}

	throw std::runtime_error("Not yet implemented");
}

} /* namespace gazebo */
} /* namespace tol_robogen */
