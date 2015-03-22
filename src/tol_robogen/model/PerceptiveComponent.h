/*
 * @(#) PerceptiveComponent.h   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef TOL_ROBOGEN_PERCEPTIVE_COMPONENT_H_
#define TOL_ROBOGEN_PERCEPTIVE_COMPONENT_H_

#include <tol_robogen/model/Model.h>
#include <vector>

//#include "model/sensors/Sensor.h"
//#include "scenario/Environment.h"


namespace tol_robogen {

class PerceptiveComponent : public Model {

public:

	/**
	 * Constructor
	 * @see Model
	 */
	PerceptiveComponent(std::string id, const Configuration & conf) :
		Model(id, conf) {}

	/**
	 * Destructor
	 */
	virtual ~PerceptiveComponent() {}

	/**
	 * @return the available sensors
	 */
	//virtual void getSensors(std::vector<boost::shared_ptr<Sensor> >& sensors) = 0;

	/**
	 * Updates the internal values of the sensors
	 */
	//virtual void updateSensors(boost::shared_ptr<Environment>& env) = 0;
};

}

#endif /* TOL_ROBOGEN_PERCEPTIVE_COMPONENT_H_ */
