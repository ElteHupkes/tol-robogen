/*
 * NeuralNetwork.h
 *
 *  Created on: Mar 16, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_BRAIN_BRAIN_H_
#define TOL_ROBOGEN_GAZEBO_BRAIN_BRAIN_H_

#include <tol_robogen/gazebo/Types.h>

#include <gazebo/common/common.hh>

extern "C" {
	#include <tol_robogen/gazebo/brain/NeuralNetwork.h>
}

namespace tol_robogen {
namespace gazebo {

class Brain {
public:
	/**
	 * @param The brain node
	 * @param Reference to motor list, will be reordered
	 * @param The brain update interval in nanoseconds
	 */
	Brain(sdf::ElementPtr node, std::vector< MotorPtr > & motors);
	virtual ~Brain();

	/**
	* @param Reference to motor list
	* TODO sensors
	*/
	void update(const std::vector< MotorPtr > & motors, double t, unsigned int step);

	// Input / output arrays used for the neural network,
	// these are stored with the object so they do not need
	// to be reallocated every time.
	float networkInputs_[MAX_INPUT_NEURONS];
	float networkOutputs_[MAX_OUTPUT_NEURONS];

protected:
	/**
	 * The neural network that controls the brain
	 */
	std::shared_ptr< NeuralNetwork > neuralNetwork_;
};

} /* namespace gazebo */
} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_GAZEBO_BRAIN_BRAIN_H_ */
