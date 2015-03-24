/*
 * Sensor.h
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_IO_SENSOR_H_
#define TOL_ROBOGEN_MODEL_IO_SENSOR_H_

#include <tol_robogen/model/io/IO.h>

namespace tol_robogen {

class Sensor: public IO {
public:
	Sensor(std::string partId, unsigned int ioId,
				std::string type,
				sdf_builder::SensorPtr sensor);
	virtual ~Sensor();
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_IO_SENSOR_H_ */
