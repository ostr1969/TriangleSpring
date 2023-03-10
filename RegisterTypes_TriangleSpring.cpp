/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoJumpGoal.cpp                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#define REGCPP
#include "TriangleSpring.h"
#include "RegisterTypes_TriangleSpring.h"

using namespace OpenSim;

static TriangleSpringInstantiator instantiator;

TRIANGLESPRING_API void RegisterTypes_TriangleSpring() {
    try {
        Object::registerType(TriangleSpring());
//	Object::registerType(NonlinearSpring());
//	Object::registerType(ActivationMuscleLikeCoordinateActuator());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during TriangleSpring "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

TriangleSpringInstantiator::TriangleSpringInstantiator() {
    registerDllClasses();
}

void TriangleSpringInstantiator::registerDllClasses() {
    RegisterTypes_TriangleSpring();
}
