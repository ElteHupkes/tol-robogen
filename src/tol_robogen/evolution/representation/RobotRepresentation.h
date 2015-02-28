/*
 * @(#) RobotRepresentation.h   1.0   Aug 28, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
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

#ifndef TOL_ROBOTREPRESENTATION_H
#define TOL_ROBOTREPRESENTATION_H

#include <string>
#include <set>
#include <stdexcept>
#include <memory>
#include <boost/random/mersenne_twister.hpp>

#include <tol_robogen/tol.h>
#include <tol_robogen/evolution/representation/PartRepresentation.h>
#include <tol_robogen/evolution/representation/NeuralNetworkRepresentation.h>

namespace tol_robogen {

/**
 * Robot representation to be used for evolution. More lightweight than the
 * robot representation of the simulator, and implements evolution-specific
 * methods.
 */
class RobotRepresentation {

public:

	/**
	 * Map from an id string to a weak pointer of a part representation
	 */
	typedef std::map<std::string, WeakPartRepresentationPtr > IdPartMap;

	/**
	 * Copy constructor: Deep copy body parts and Neural network
	 */
	RobotRepresentation(const RobotRepresentation &r);

	/**
	 * Error-less constructor for memory assignment
	 */
	RobotRepresentation();

	/**
	 * assignment operator: Deep copy body parts and Neural network,
	 * using copy-swap.
	 */
	RobotRepresentation &operator=(RobotRepresentation r);

	/**
	 * Swap function for copy-swap
	 */
	friend void swap(RobotRepresentation & a, RobotRepresentation & b);

	/**
	 * Constructs a robot representation from nothing.
	 * Will have just the core component.
	 */
	bool init();

	/**
	 * Constructs a robot representation from a robot text file
	 * @param robotTextFile text file of the robot description
	 * @todo make a better handling of formatting errors
	 */
	bool init(std::string robotTextFile);

	/**
	 * @return robot message of this robot to be transmitted to simulator
	 * or stored as population checkpoint
	 */
	//robogenMessage::Robot serialize() const;

	/**
	 * Provides weight and bias handles for a mutator.
	 * @param weights reference to a vector to be filled with weight pointers
	 * @param types reference to a vector to be filled with types of neurons
	 * @param params reference to a vector to be filled with params pointers
	 */
	void getBrainGenome(std::vector<double*> &weights,
			std::vector<unsigned int> &types,
			std::vector<double*> &params);

	/**
	 * @return a shared pointer to the robots brain
	 */
	NeuralNetworkRepresentationPtr getBrain() const;

	/**
	 * @return a shared pointer to the robots body
	 */
	const IdPartMap &getBody() const;

	/**
	 * Removes body part and all children at indicated position.
	 * @return false upon failure
	 */
	bool trimBodyAt(const std::string& id);

	/**
	 * @return a string unique id
	 */
	std::string generateUniqueIdFromSomeId();

	/**
	 * Duplicate a subtree in the body tree
	 *
	 * @param subtreeRootPartId the root of the subtree to duplicate
	 * @param subtreeDestPartId the destination part where the subtree will be attached to
	 * @param slotId the slot identifier of the destination part where the subtree will be attached to
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool duplicateSubTree(const std::string& subtreeRootPartId,
			const std::string& subtreeDestPartId, unsigned int slotId);

	/**
	 * Swap subtrees in the body tree
	 *
	 * @param subtreeRoot1 the root of the first subtree
	 * @param subtreeRoot2 the root of the second subtree
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool swapSubTrees(const std::string& subtreeRoot1,
			const std::string& subtreeRoot2);

	/**
	 * Insert a part into the body tree
	 *
	 * @param parentPartId id of the part that will become the parent of newPart
	 * @param parentPartSlot slot id where the newPart will be inserted
	 * @param newPart the new part to insert
	 * @param newPartSlot the slot of the new part where the subtree of parentPartId
	 *        connected to parentPartSlot will be connected to
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool insertPart(const std::string& parentPartId,
			unsigned int parentPartSlot,
			PartRepresentationPtr newPart,
			unsigned int newPartSlot);

	/**
	 * Remove a part from the body tree
	 *
	 * @param partId id of the part to remove
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool removePart(const std::string& partId);

	/**
	 * Creates a robot that can be converted to SDF from
	 * this representation.
	 */
	RobotPtr toRobot() const;

	/**
	 * @return the id of the root body part
	 */
	const std::string& getBodyRootId();

	/**
	 * Check the consistency of this robot
	 * @return true if the body is consistent with the neural representation, and there are no
	 * dangling body parts/neurons
	 */
	bool check();

	/**
	 * @return a string representation of the robot
	 */
	std::string toString();

private:
	/**
	 *
	 */
	void recurseNeuronRemoval(PartRepresentationPtr part);

	/**
	 * Insert parts to the body id-parts map
	 *
	 * @param part the root of the subtree to be inserted into the body id to parts map
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool addClonesToMap(PartRepresentationPtr part,
			std::map<std::string, std::string> &neuronReMapping);

	/**
	 * Points to the root of the robot body tree
	 */
	PartRepresentationPtr bodyTree_;

	/**
	 * Neural network representation of the robot
	 */
	NeuralNetworkRepresentationPtr neuralNetwork_;

	/**
	 * Map from part id to part representation
	 * @todo use to avoid multiple samenames
	 */
	IdPartMap idToPart_;

	/**
	 * Counter for unique ID.
	 */
	int maxid_;

};

#endif /* TOL_ROBOTREPRESENTATION_H */
