/*
 * Motor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_IO_IO_H_
#define TOL_ROBOGEN_MODEL_IO_IO_H_

#include <tol_robogen/tol.h>

#include <sdf_builder/Element.h>

namespace tol_robogen {

/**
 * Simple class to represent a motor/sensor in the model, we'll pass
 * it in the SDF to the controller classes.
 */
class IO : public sdf_builder::Element {
public:
	IO(std::string ioType, std::string partId, unsigned int ioId,
			std::string type, std::string ref);
	IO(const IO & other);
	virtual ~IO();

	virtual IO * clone() const;

	/**
	 * Returns the XML representation of this motor
	 */
	virtual std::string toXML();

protected:
	/**
	 * Optional attributes
	 */
	virtual std::string attributes();

	/**
	 * Type of IO; "sensor" or "motor"
	 */
	std::string ioType_;

	/**
	 * ID of the parent part
	 */
	std::string partId_;

	/**
	 * Type of the motor
	 */
	std::string type_;

	/**
	 * I/O ID of the motor on the main part
	 */
	unsigned int ioId_;

	/**
	 * Reference to the relevant part in SDF,
	 * either a joint or a sensor name.
	 */
	std::string ref_;
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_IO_IO_H_ */
