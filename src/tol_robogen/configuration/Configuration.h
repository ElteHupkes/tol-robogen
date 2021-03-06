/*
 * Configuration.h
 *
 *  Created on: Mar 22, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_CONFIGURATION_H_
#define TOL_ROBOGEN_CONFIGURATION_H_

#include <string>

namespace tol_robogen {

/**
 * A configuration class used when Robots are generated.
 */
class Configuration {
public:
	Configuration();
	virtual ~Configuration();

	/**
	 * Number of seconds between sensor updates /
	 * neural network feeds.
	 */
	double actuationTime = 0.025;

	/**
	 * The level of motor noise
	 */
	double motorNoiseLevel = 0;

	/**
	 * The level of sensor noise
	 */
	double sensorNoiseLevel = 0;

	/**
	 * World scaling factor; scaling currently isn't
	 * reliable, I advice against using this for other
	 * purposes than visual model checks.
	 */
	double scaling = 1.0;

	/**
	 * Turn sensor visualization on model generation
	 * on or off.
	 */
	bool visualizeSensors = true;

	// Phycics parameters; got these from
	// RobogenCollision.cpp originally
	double surfaceFriction1 = 1.0;
	double surfaceFriction2 = 1.0;
	double surfaceSlip1 = 0.1;
	double surfaceSlip2 = 0.1;
	double surfaceSoftCfm = 0.01;
	double surfaceSoftErp = 0.96;

};

} /* namespace sdf_builder */

#endif /* TOL_ROBOGEN_CONFIGURATION_H_ */
