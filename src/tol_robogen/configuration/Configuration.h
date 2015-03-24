/*
 * Configuration.h
 *
 *  Created on: Mar 22, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_CONFIGURATION_H_
#define TOL_ROBOGEN_CONFIGURATION_H_

namespace tol_robogen {

/**
 * A configuration class tied to neither the representation
 * nor the Gazebo part, which is convenient for passing
 * config around.
 */
class Configuration {
public:
	Configuration();
	virtual ~Configuration();

public:
	/**
	 * Number of seconds between sensor updates /
	 * neural network feeds.
	 */
	double updateRate = 0.025;

	/**
	 * World scaling factor; scaling currently isn't
	 * really reliable.
	 */
	double scaling = 1.0;

	/**
	 * The level of motor noise
	 */
	double motorNoiseLevel = 0;

	/**
	 * The level of sensor noise
	 */
	double sensorNoiseLevel = 0;
};

} /* namespace sdf_builder */

#endif /* TOL_ROBOGEN_CONFIGURATION_H_ */
