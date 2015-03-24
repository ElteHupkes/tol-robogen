/*
 * Sensor.h
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_IO_SENSOR_H_
#define TOL_ROBOGEN_MODEL_IO_SENSOR_H_

#include <tol_robogen/model/io/IO.h>

#include <sdf_builder/Link.h>
#include <sdf_builder/sensor/Sensor.h>

namespace tol_robogen {

class Sensor: public IO {
public:
	/**
	 * Creates an I/O element for representing a sensor.
	 *
	 * @param The ID of the part that contains the sensor
	 * @param ioID The I/O ID on the part
	 * @param The type of the sensor
	 * @param The sdf builder sensor element
	 * @param The link element containing the sensor
	 */
	Sensor(std::string partId, unsigned int ioId,
			std::string type, sdf_builder::SensorPtr sensor,
			sdf_builder::LinkPtr link);
	virtual ~Sensor();

protected:
	virtual std::string attributes();

	sdf_builder::LinkPtr link_;
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_IO_SENSOR_H_ */
