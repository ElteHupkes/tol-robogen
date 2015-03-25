/*
 * Sensor.h
 *
 *  Created on: Mar 24, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_SENSORS_SENSOR_H_
#define TOL_ROBOGEN_GAZEBO_SENSORS_SENSOR_H_

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <tol_robogen/gazebo/Types.h>

namespace tol_robogen {
namespace gazebo {

class Sensor {
public:
	Sensor(::gazebo::physics::ModelPtr model, ::gazebo::sensors::SensorPtr sensor,
			std::string partId, unsigned int ioId);
	virtual ~Sensor();

	/**
	 * Reads the current value of this sensor
	 */
	virtual float read() = 0;

	/**
	 * @return The part ID
	 */
	std::string partId();

	/**
	 * @return I/O ID
	 */
	unsigned int ioId();

	/**
	 * @return The attached Gazebo sensor
	 */
	::gazebo::sensors::SensorPtr gzSensor();

protected:
	/**
	 * The model this motor is part of
	 */
	::gazebo::physics::ModelPtr model_;

	/**
	 * The joint this motor is controlling
	 */
	::gazebo::sensors::SensorPtr sensor_;

	/**
	 * ID of the body part the motor belongs to
	 */
	std::string partId_;

	/**
	 * I/O ID on the body part
	 */
	unsigned int ioId_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_SENSORS_SENSOR_H_ */
