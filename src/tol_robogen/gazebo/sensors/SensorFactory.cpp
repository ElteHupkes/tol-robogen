/*
 * SensorFactory.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#include <tol_robogen/gazebo/sensors/SensorFactory.h>
#include <tol_robogen/gazebo/sensors/Sensors.h>

#include <stdexcept>

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

	if (!typeParam || !nameParam || !partIdParam || !ioIdParam) {
		std::cerr << "Sensor is missing required attributes." << std::endl;
		throw std::runtime_error("Sensor error");
	}

	auto sensorName = nameParam->GetAsString();

	//gz::sensors::SensorPtr tst = gz::sensors::get_sensor(sensorName);

	throw std::runtime_error("Not yet implemented");
}

} /* namespace gazebo */
} /* namespace tol_robogen */
