/*
 * PartFactory.h
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_PARTFACTORY_H_
#define TOL_ROBOGEN_MODEL_PARTFACTORY_H_

#include <tol_robogen/tol.h>
#include <tol_robogen/configuration/Configuration.h>
#include <vector>
#include <string>

namespace tol_robogen {

class PartFactory {
public:
	virtual ~PartFactory();

	/**
	 * @return New component based on the given type
	 */
	static ModelPtr getComponent(std::string type, std::string id,
			const Configuration & conf,
			const std::vector<double>& params);
private:
	// Singleton
	PartFactory();
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_PARTFACTORY_H_ */
