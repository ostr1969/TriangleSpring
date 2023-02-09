/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoHighFoot.cpp                                     *
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

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/OpenSim.h>
//#include "LeftKneeAngleGoal.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;
class LeftKneeAngleGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(LeftKneeAngleGoal, MocoGoal);

public:
    LeftKneeAngleGoal() {}
    LeftKneeAngleGoal(std::string name) : MocoGoal(std::move(name)) {}
    LeftKneeAngleGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(1, 1);
    }
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override {
      //  getModel().realizeVelocity(input.state);
	 double tim=input.time;
    double kneeLangle=getModel().
	    getStateVariableValue(input.state,"/jointset/knee_l/knee_angle_l/value");
    auto& tipl=getModel().getMarkerSet().get("tipl");
    double tipl_y=tipl.getLocationInGround(input.state)[1];
    //double dist=pow(0.25-tipl_y,2);
    double ref=27.48*tim*tim-10.46*tim-0.9378;
    double dist=abs(ref-kneeLangle);
    integrand = tim<0.13?dist:0;
    }

    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
    cost[0]=input.integral;
    }
};



