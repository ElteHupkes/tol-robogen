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
	double scaling = 10.0;
};

} /* namespace sdf_builder */

#endif /* TOL_ROBOGEN_CONFIGURATION_H_ */
