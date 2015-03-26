/*
 * PartFactory.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#include <iostream>

#include <tol_robogen/model/PartFactory.h>
#include <tol_robogen/evolution/representation/PartList.h>
#include <tol_robogen/model/Models.h>

namespace tol_robogen {

PartFactory::PartFactory() {}

PartFactory::~PartFactory() {}

ComponentPtr PartFactory::getComponent(std::string type, std::string id,
		ConfigurationPtr conf, const std::vector<double> & params) {
	ComponentPtr model;

	if (PART_TYPE_CORE_COMPONENT == type) {
		model.reset(new CoreComponentModel(id, conf, true));
	} else if (PART_TYPE_FIXED_BRICK == type) {
		model.reset(new CoreComponentModel(id, conf, false));
	} else if (PART_TYPE_PASSIVE_HINGE == type) {
		model.reset(new HingeModel(id, conf));
	} else if (PART_TYPE_ACTIVE_HINGE == type) {
		model.reset(new ActiveHingeModel(id, conf));
	} else if (PART_TYPE_LIGHT_SENSOR == type) {
		model.reset(new LightSensorModel(id, conf, true));
	}

	// TODO Add other models

	if (!model) {
		std::cerr << "Part type '" << type << "' could not be resolved!"
				<< std::endl;
		throw std::runtime_error("Part error");
	}

	// Initialize the model
	model->initModel();
	return model;
}

} /* namespace tol_robogen */
