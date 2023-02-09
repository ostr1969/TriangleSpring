#ifndef MOCO_DELPACTUATOR_H
#define MOCO_DELPACTUATOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DelpActuator.h                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

//#include "/home/barako/Opensim/opensim-moco/Moco/Moco/osimMocoDLL.h"
//#include <Moco/osimMocoDLL.h>
#define OSIMMOCO_API
#include <OpenSim/Common/readxy.h>
#include <OpenSim/Common/mypchip.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.cpp>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/SimmSpline.h>

namespace OpenSim {

/// Similar to CoordinateActuator (simply produces a generalized force) but
/// with first-order linear activation dynamics. This actuator has one state
/// variable, `activation`, with \f$ \dot{a} = (u - a) / \tau \f$, where
/// \f$ a \f$ is activation, \f$ u \f$ is excitation, and \f$ \tau \f$ is the
/// activation time constant (there is no separate deactivation time constant).
/// <b>Default %Property Values</b>
/// @verbatim
/// activation_time_constant: 0.01
/// default_activation: 0.5
/// @endverbatim
class OSIMMOCO_API
    DelpActuator : public CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(DelpActuator,
        CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Smaller value means activation can change more rapidly "
        "(units: seconds).");

    OpenSim_DECLARE_PROPERTY(dactivation_time_constant, double,
        "Smaller value means activation can change more rapidly "
        "(units: seconds).");

    OpenSim_DECLARE_PROPERTY(default_activation, double,
        "Value of activation in the default state returned by initSystem().");

    OpenSim_DECLARE_PROPERTY(has_delp_curve, int,
        "Notify 1 if has delp curve, else use optimal force");

    OpenSim_DECLARE_PROPERTY(delp_curve_filename, std::string,
        "Delp curve filename, file of ang/max biologicl in degrees");

    OpenSim_DECLARE_PROPERTY(qfac_dir, int,
        "1 or -1 for velocity factor(ashby)");

    OpenSim_DECLARE_PROPERTY(max_velocity, double,
        "Maximum velocity in rad/sec if using delp curve");

    OpenSim_DECLARE_PROPERTY(force_length_curve, Function,
        "Function representing the force-length behavior of the ligament");

   // OpenSim_DECLARE_SOCKET(delpactuator, DelpActuator,
//		            "delpactuator by barak.");


    OpenSim_DECLARE_OUTPUT(statebounds_activation, SimTK::Vec2,
        getBoundsActivation, SimTK::Stage::Model);
    OpenSim_DECLARE_OUTPUT(tou, double,
			      getCurrentTorque, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(act, double,
			      getCurrentActivation, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(qfac, double,
			      getCurrentQFactor, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(ex, double,
			      getCurrentControl, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(opt, double,
			      getOptimalByCurrentAngle, SimTK::Stage::Dynamics);

	std::vector<vector<double>> delpcurve;

    DelpActuator() {
        constructProperties();
	
    }
    DelpActuator(string name,double mxvel,int qfacdir,double act,
		    double dact,double defact,string filname,Coordinate* coord)
    {
	constructProperties();
        set_activation_time_constant(act);
        set_dactivation_time_constant(dact);
        set_default_activation(defact);
        set_has_delp_curve(1);
        set_qfac_dir(qfacdir);
        set_max_velocity(mxvel);
	set_delp_curve_filename(filname);
	setMinControl(-1);
	setMaxControl(1);
	setOptimalForce(1);	
	setCoordinate(coord);
	setName(name);
	readxy(filname,delpcurve);

    }

    int SetCurve(string filename){
	set_has_delp_curve(1);
	set_delp_curve_filename(filename);
        //cout<<"Curvefilename "<<filename<<","<<get_coordinate<<endl;
	readxy(filename,delpcurve);
	return 1;
	} 
    virtual const Function& getTorqueAngleCurve() const
		    { return get_force_length_curve(); }
    bool setTorqueAngleCurve(const Function& aTorqueAngleCurve)
    { 
	    set_force_length_curve(aTorqueAngleCurve);
		    return true; }


    double getDelpOptimal( SimTK::State s) const {
	//get optimal force by temporary zero speed and activation=1
	setStateVariableValue(s, "activation", 1);
	//s.setU(SimTK::Vector(4,0.0));
	return computeActuation(s);
	} 
    double getCurrentTorque(const SimTK::State& s) const {
	    return computeActuation(s);}
    double getCurrentActivation (const SimTK::State& s) const {
	    return getStateVariableValue(s, "activation");}
    double getCurrentQFactor(const SimTK::State& s) const {
	    return qToQfac(getSpeed(s)*get_qfac_dir(),  get_max_velocity());}
    double getCurrentControl(const SimTK::State& s) const {
	    const auto& u=getControl(s);
	    return u;}
    double getOptimalByCurrentAngle(const SimTK::State& s) const {
	    Coordinate* coord1=getCoordinate();//get the coordinate number
	    double optimal=getTorqueAngleCurve().calcValue(
			     SimTK::Vector(1, coord1->getValue(s)));
	    return optimal;}

    double getDelpLowAngle(){
	cout<<"delpcurve low angle:"<<delpcurve[0][0]<<"/"<<delpcurve[0][0]*180/SimTK::Pi<<endl;
	return delpcurve[0][0];
	}
    double getDelpHighAngle(){
        int dL=delpcurve.size();
	cout<<"delpcurve high angle:"<<delpcurve[dL-1][0]<<"/"<<delpcurve[dL-1][0]*180/SimTK::Pi<<endl;
	return delpcurve[dL-1][0];}

    double getDelpVars(SimTK::State s,int i) const {
	double var;//tou,q,act,fac,optimal
	if (i==0) var=computeActuation(s);
        if (i==1) var=getStateVariableValue(s, "activation");
	if (i==2) var=qToQfac(getSpeed(s)*get_qfac_dir(),  get_max_velocity());
	//get optimal force by temporary zero speed and activation=1
	if (i==3) {setStateVariableValue(s, "activation", 1);
        //s.setU(SimTK::Vector(4,0.0));
	var= computeActuation(s)/qToQfac(getSpeed(s)*get_qfac_dir(),  get_max_velocity());}
	return var;
	} 

    SimTK::Vec2 getBoundsActivation(const SimTK::State&) const {
        return SimTK::Vec2(getMinControl(), getMaxControl());
    }

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
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

    // TODO no need to do clamping, etc; CoordinateActuator is bidirectional.
    void computeStateVariableDerivatives(const SimTK::State& s) const override
    {
        SimTK::Real adot=0.;
        const auto& tau = get_activation_time_constant();
        const auto& taud = get_dactivation_time_constant();
        const auto& u = getControl(s);
        const auto& a = getStateVariableValue(s, "activation");
        if (u*u>=a*a)  adot = (u - a) / tau;
        if (u*u<a*a)   adot = (u - a) / taud;
        setStateVariableDerivativeValue(s, "activation", adot);
    }


    double computeActuation(const SimTK::State& s) const override {
        if (get_has_delp_curve()==1){//set optimal force by text file ang/optimal
//			string filename=get_delp_curve_filename();
	                //SetCurve(filename);
//			cout<<"EmptyCurve, filling now\n";}
        SimTK::Vector Q=s.getQ();//Q*=(180./SimTK::Pi);
        Coordinate* coord1=getCoordinate();//get the coordinate number
//cout<<Q<<endl;
        //double optimal = getForceLengthCurve().calcValue(
	//	  SimTK::Vector(1, coord1->getValue(s)));
	//cout<<coord1->getValue(s)<<","<<optimal<<endl;
	double optimal;
	if (delpcurve.empty()) optimal=getTorqueAngleCurve().calcValue(
				SimTK::Vector(1, coord1->getValue(s)));
	else
        	optimal=limitedPchip(delpcurve,coord1->getValue(s));
        double qfac=qToQfac(getSpeed(s)*get_qfac_dir(),  get_max_velocity());
        return getStateVariableValue(s, "activation") * optimal*qfac;
        }
        else
        return getStateVariableValue(s, "activation") * getOptimalForce();
    }

private:
    void constructProperties() {
        constructProperty_activation_time_constant(0.011);
        constructProperty_dactivation_time_constant(0.068);
        constructProperty_default_activation(0.0);
        constructProperty_has_delp_curve(0);
        constructProperty_delp_curve_filename("src/delp1.txt");
        constructProperty_qfac_dir(0);
        constructProperty_max_velocity(0);
        int forceLengthCurvePoints = 3;
 	double forceLengthCurveX[] = {-5.0,0.998,0.999};
	double forceLengthCurveY[] = {0.,0,0};
        SimmSpline forceLengthCurve(forceLengthCurvePoints, forceLengthCurveX, forceLengthCurveY);
        constructProperty_force_length_curve(forceLengthCurve);

    }
	double qToQfac(const double qdot, const double qmax) const{
		double Tvel;
		if (qdot>qmax) Tvel=0.;
    		else if (qdot<=qmax && qdot>=-1./1.5*qmax*tan(0.5*atan(1.5)))
        		Tvel=1.-1./atan(1.5)*atan(1.5*qdot/qmax);
    		else
        	Tvel=1.5;

    		return Tvel;
		}		

};

} // namespace OpenSim

#endif // MOCO_DELPACTUATOR_H
