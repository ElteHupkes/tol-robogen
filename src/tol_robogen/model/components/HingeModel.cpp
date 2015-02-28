/*
 * @(#) HingeModel.cpp   1.0   Feb 8, 2013
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
#include <tol_robogen/model/components/HingeModel.h>

namespace tol_robogen {

namespace sb = sdf_builder;

const float HingeModel::MASS_SLOT = inGrams(2);
const float HingeModel::MASS_FRAME = inGrams(1);
const float HingeModel::SLOT_WIDTH = inMm(34);
const float HingeModel::SLOT_THICKNESS = inMm(1.5);
const float HingeModel::CONNECTION_PART_LENGTH = inMm(20.5);
const float HingeModel::CONNECTION_PART_HEIGHT = inMm(20);
const float HingeModel::CONNECTION_PART_THICKNESS = inMm(2);

// Computed from the left corner of the connection part
const float HingeModel::CONNECTION_ROTATION_OFFSET = inMm(18.5);

// Center of rotation 18.5 from the slot

HingeModel::HingeModel(std::string id) :
		Model(id)
{}

HingeModel::~HingeModel() {

}

bool HingeModel::initModel() {

	// Create the 4 components of the hinge
	// TODO Should these bodies self-collide? They do by default now.
	hingeRoot_ = this->createLink(B_SLOT_A_ID);
	sb::LinkPtr connectionPartA = this->createLink(B_CONNECTION_A_ID);
	sb::LinkPtr connectionPartB = this->createLink(B_CONNECTION_B_ID);
	hingeTail_ = this->createLink(B_SLOT_B_ID);

	// Create the geometries
	float separation = inMm(0.1);

	// Root will be at (0, 0, 0), no need to reposition
	hingeRoot_->makeBox(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	// Connection part A needs to be positioned
	double xPartA = SLOT_THICKNESS / 2 + separation
			+ CONNECTION_PART_LENGTH / 2;
	connectionPartA->position(sb::Vector3(xPartA, 0, 0));
	connectionPartA->makeBox(MASS_FRAME, CONNECTION_PART_LENGTH,
			CONNECTION_PART_THICKNESS, CONNECTION_PART_HEIGHT);

	// Part b also must be positioned
	double xPartB = xPartA
			+ (CONNECTION_PART_LENGTH / 2
					- (CONNECTION_PART_LENGTH - CONNECTION_ROTATION_OFFSET))
					* 2;
	connectionPartB->position(sb::Vector3(xPartB, 0, 0));
	connectionPartB->makeBox(MASS_FRAME, CONNECTION_PART_LENGTH, CONNECTION_PART_THICKNESS,
			CONNECTION_PART_HEIGHT);

	// Finally the tail, also with a position
	double xTail = xPartB + CONNECTION_PART_LENGTH / 2 + separation
			+ SLOT_THICKNESS / 2;
	hingeTail_->position(sb::Vector3(xTail, 0, 0));
	hingeTail_->makeBox(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	// Create joints to hold pieces in position
	// root <-> connectionPartA
	this->fixLinks(hingeRoot_, connectionPartA,
			sb::Vector3(1, 0, 0), sb::Vector3(-CONNECTION_PART_LENGTH / 2, 0, 0));

	// connectionPartA <(hinge)> connectionPArtB
	// Hinge joint axis should point straight up, and anchor the
	// points in the center. Note that the position of a joint is
	// expressed in the child link frame, so we need to take the
	// position from the original code and subtract connectionPartB's
	// position.
	sb::JointPtr revolve(new sb::RevoluteJoint(connectionPartA, connectionPartB));
	revolve->axis->xyz(sb::Vector3(0, 0, 1));
	revolve->position(sb::Vector3(
			CONNECTION_PART_LENGTH / 2 - CONNECTION_ROTATION_OFFSET, 0, 0));
	this->addJoint(revolve);

	// connectionPartB <-> tail
	this->fixLinks(connectionPartB, hingeTail_,
			sb::Vector3(1, 0, 0), sb::Vector3(-SLOT_THICKNESS / 2, 0, 0));

	return true;

}

sb::LinkPtr HingeModel::getRoot() {
	return hingeRoot_;
}

sb::LinkPtr HingeModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return hingeRoot_;
	} else {
		return hingeTail_;
	}
}

sb::Vector3 HingeModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Vector3 slotPos;
	if (i == SLOT_A) {

		sb::Vector3 curPos = hingeRoot_->position();
		sb::Vector3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		sb::Vector3 curPos = hingeTail_->position();
		sb::Vector3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	}

	return slotPos;

}

sb::Vector3 HingeModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Quaternion quat;
	sb::Vector3 axis;

	if (i == SLOT_A) {

		quat = hingeRoot_->rotation();
		axis[0] = -1;
		axis[1] = 0;
		axis[2] = 0;

	} else if (i == SLOT_B) {

		quat = hingeTail_->rotation();
		axis[0] = 1;
		axis[1] = 0;
		axis[2] = 0;

	}

	return quat * axis;

}

sb::Vector3 HingeModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Quaternion quat;
	sb::Vector3 axis;

	if (i == SLOT_A) {

		quat = hingeRoot_->rotation();
		axis[0] = 0; axis[1] = 1; axis[2] = 0;

	} else if (i == SLOT_B) {

		quat = hingeTail_->rotation();
		axis[0] = 0; axis[1] = 1; axis[2] = 0;

	}

	return quat * axis;

}

}
