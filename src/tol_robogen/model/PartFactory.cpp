/*
 * PartFactory.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#include <iostream>

#include <tol_robogen/model/PartFactory.h>

namespace tol_robogen {

PartFactory::PartFactory() {}

PartFactory::~PartFactory() {}

ModelPtr PartFactory::getComponent(std::string type, std::string id, const std::vector<double> & params) {
	ModelPtr model;

	if (type == PART_TYPE_CORE_COMPONENT) {
		model.reset(new CoreComponentModel(id, true));
	} else if (type == PART_TYPE_FIXED_BRICK) {
		model.reset(new CoreComponentModel(id, false));
	} else if (type == PART_TYPE_PASSIVE_HINGE) {
		model.reset(new HingeModel(id));
	}

	// TODO Add other models

	if (!model) {
		std::cerr << "Part type '" << type << "' could not be resolved!"
				<< std::endl;
		throw std::runtime_error("");
	}

	return model;
}

} /* namespace tol_robogen */
