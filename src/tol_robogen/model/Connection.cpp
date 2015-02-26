/*
 * @(#) Connection.cpp   1.0   Aug 26, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013
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
#include "model/Connection.h"
#include <map>
#include <sstream>
#include <stdexcept>
#include <tol_robogen/model/Model.h>

namespace tol_robogen{

Connection::Connection(ModelPtr from, int fromSlot, ModelPtr to, int toSlot):
		from_(from),
		fromSlot_(fromSlot),
		to_(to),
		toSlot_(toSlot)
{}

ModelPtr Connection::getFrom(){
	return from_;
}

int Connection::getFromSlot(){
	return fromSlot_;
}

ModelPtr Connection::getTo(){
	return to_;
}

int Connection::getToSlot(){
	return toSlot_;
}

} /* namespace robogen */
