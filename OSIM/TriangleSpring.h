#ifndef OPENSIM_TRIANGLESPRING_H
#define OPENSIM_TRIANGLESPRING_H
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  TriangleSpring.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/osimActuatorsDLL.h>

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>
using namespace std;
//using namespace OpenSim;

namespace OpenSim {

/**
Similar to CoordinateActuator (simply produces a generalized force) but
with first-order linear activation dynamics. This actuator has one state
variable, `activation`, with \f$ \dot{a} = (x - a) / \tau \f$, where
\f$ a \f$ is activation, \f$ x \f$ is excitation, and \f$ \tau \f$ is the
activation time constant (there is no separate deactivation time constant).
The statebounds_activation output is used in Moco to set default values for
the activation state variable.
<b>Default %Property Values</b>
@verbatim
activation_time_constant: 0.01
default_activation: 0.5
@endverbatim
 */
class OSIMACTUATORS_API TriangleSpring
        : public CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(TriangleSpring,
        CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(ang0, double,
        "angle in deg where the spring start and stop");
//    OpenSim_DECLARE_PROPERTY(coordinate, string,
//        "joint which the spring is assembled on      ");
    OpenSim_DECLARE_PROPERTY(stiffness, double,
        "spring stiffness Nm/deg     ");

    OpenSim_DECLARE_OUTPUT(torque, double, computeTorque,
                           SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(angle, double, computeAngle,
                           SimTK::Stage::Position);

    TriangleSpring() {
        constructProperties();
    }

    /// Provide the coordinate name.
    explicit TriangleSpring(const double initAng,
			const double stif,
			const std::string& coordinateName)
            : TriangleSpring() {
        set_ang0(initAng);
	set_stiffness(stif);
        if (!coordinateName.empty()) {
            set_coordinate(coordinateName);
        }
    }


    double computeTorque   (  const SimTK::State& s) const{
		    return computeActuation(s);
    } ;
    double computeAngle    ( const SimTK::State& s) const{
	string coordRef=get_coordinate();
	const Coordinate& aCoord = getModel().getCoordinateSet().get(coordRef);
		    return aCoord.getValue(s);
    } ;

protected:
/*    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activation", SimTK::Stage::Dynamics);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "activation", get_default_activation());
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_activation(getStateVariableValue(s, "activation"));
    }

    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        // No need to do clamping, etc; CoordinateActuator is bidirectional.
        const auto& tau = get_activation_time_constant();
        const auto& detau = get_deactivation_time_constant();
        const auto& x = getControl(s);
        double c01=std::tanh(10*(x-.5))/2+0.5;//fix control to push to 0 or 1
        const auto& a = getStateVariableValue(s, "activation");
	SimTK::Real adot;
	if (c01>=a)
          adot = (c01 - a) / tau;
	else
          adot = (c01 - a) / detau;
	//if (x<0.5)
	//	adot=0;
        setStateVariableDerivativeValue(s, "activation", adot);
    }*/

    double computeActuation(const SimTK::State& s) const override {
    const OpenSim::SmoothSphereHalfSpaceForce& tip = getModel().
           getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactFront_r");
    const double tfy=abs(tip.getFy(s));
	
	string coordRef=get_coordinate();
	const Coordinate& aCoord = getModel().getCoordinateSet().get(coordRef);
        if (aCoord.getValue(s)*180/SimTK::Pi<get_ang0() and tfy>50)
        	return -get_stiffness()* (aCoord.getValue(s)*180/SimTK::Pi-get_ang0());
	else 
		return 0;
    }
private:
    void constructProperties() {
        constructProperty_stiffness(10);
        constructProperty_ang0(0);
    }
};

} // namespace OpenSim

#endif // OPENSIM_TRIANGLESPRING_H
