/*
 * Sensor.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#include <tol_robogen/model/io/Sensor.h>
#include <sdf_builder/Types.h>
#include <sdf_builder/sensor/Sensor.h>

namespace tol_robogen {

Sensor::Sensor(std::string partId, unsigned int ioId,
		std::string type, sdf_builder::SensorPtr sensor,
		sdf_builder::LinkPtr link):
	IO("sensor", partId, ioId, type, sensor->name()),
	link_(link)
{}

Sensor::~Sensor() {}

std::string Sensor::attributes() {
	return "link=\""+link_->name()+"\"";
}

} /* namespace tol_robogen */
