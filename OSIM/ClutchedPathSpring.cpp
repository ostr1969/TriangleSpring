/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ClutchedPathSpring.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "ClutchedPathSpring.h"
#include <OpenSim/Simulation/Model/Model.h>



//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const Vec3 DefaultClutchedPathSpringColor(.9,.9,.9); // mostly white 

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
ClutchedPathSpring::ClutchedPathSpring()
{
    constructProperties();
}

ClutchedPathSpring::ClutchedPathSpring(const string& name, double stiffness,
                    double dissipation, double relaxationTau, double stretch0)
{
    constructProperties();
    setName(name);
    set_stiffness(stiffness);
    set_dissipation(dissipation);
    set_relaxation_time_constant(relaxationTau);
    set_initial_stretch(stretch0);
}

//_____________________________________________________________________________
/*
 * Construct and initialize the properties for the ClutchedPathSpring.
 */
void ClutchedPathSpring::constructProperties()
{
    setAuthors("Barak Ostraich");
    constructProperty_stiffness(SimTK::NaN);
    constructProperty_dissipation(SimTK::NaN);
    constructProperty_relaxation_time_constant(0.001); //1ms
    constructProperty_initial_stretch(0.0);
    constructProperty_initial_length (0.0);
    constructProperty_coordRef ("");
    setMinControl(0.0);
    setMaxControl(1.0);

    setOptimalForce(1.0);
}

//_____________________________________________________________________________
/*
 * Set the stiffness.
 */
void ClutchedPathSpring::setStiffness(double stiffness)
{
    set_stiffness(stiffness);
}
//_____________________________________________________________________________
/*
 * Set the dissipation.
 */
void ClutchedPathSpring::setDissipation(double dissipation)
{
    set_dissipation(dissipation);
}

//_____________________________________________________________________________
/*
 * Set the initial stretch.
 */
void ClutchedPathSpring::setInitialStretch(double stretch0)
{
    set_initial_stretch(stretch0);
}
void ClutchedPathSpring::setInitialLength (double length0)
{
    set_initial_length (length0);
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this ClutchedPathSpring.
 */
 void ClutchedPathSpring::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    // The spring force is dependent of stretch so only invalidate dynamics
    // if the stretch state changes
    addStateVariable("stretch");
}

 void ClutchedPathSpring::extendInitStateFromProperties(SimTK::State& state) const
 {
     setStateVariableValue(state, "stretch", get_initial_stretch());
 }

 void ClutchedPathSpring::extendSetPropertiesFromState(const SimTK::State& state)
 {
     set_initial_stretch(getStretch(state));
 }
 
 void ClutchedPathSpring::extendFinalizeFromProperties()
 {
     Super::extendFinalizeFromProperties();

     OPENSIM_THROW_IF_FRMOBJ(
         (SimTK::isNaN(get_stiffness()) || get_stiffness() < 0),
         InvalidPropertyValue, getProperty_stiffness().getName(),
         "Stiffness cannot be less than zero");
     OPENSIM_THROW_IF_FRMOBJ(
         (SimTK::isNaN(get_dissipation()) || get_dissipation() < 0),
         InvalidPropertyValue, getProperty_dissipation().getName(),
         "Dissipation cannot be less than zero");
     OPENSIM_THROW_IF_FRMOBJ(
         (SimTK::isNaN(get_relaxation_time_constant()) || get_relaxation_time_constant() < 0),
         InvalidPropertyValue, 
         getProperty_relaxation_time_constant().getName(),
         "Relaxation time constant cannot be less than zero");
     OPENSIM_THROW_IF_FRMOBJ(
         (SimTK::isNaN(get_initial_stretch()) || get_initial_stretch() < 0),
         InvalidPropertyValue, getProperty_initial_stretch().getName(),
         "Initial stretch cannot be less than zero");
 }



//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// STRETCH and TENSION
//-----------------------------------------------------------------------------

double ClutchedPathSpring::getStretch(const SimTK::State& s) const
{
    return getStateVariableValue(s, "stretch");
}


double ClutchedPathSpring::getTension(const SimTK::State& s) const
{
    // evaluate tension in the spring
    // note tension is positive and produces shortening
    // damping opposes lengthening, which is positive lengthening speed
    // there for stretch and lengthening speed increase tension
    return getActuation(s);
}


//-----------------------------------------------------------------------------
// COMPUTATIONS
//-----------------------------------------------------------------------------
double ClutchedPathSpring::computeLength( const SimTK::State& s ) const
{
		    return getGeometryPath().getLength(s);
}
double ClutchedPathSpring::computeStretch( const SimTK::State& s ) const
{
		    return getStretch(s);
}
double ClutchedPathSpring::computeTorque ( const SimTK::State& s ) const
{
		    return computeActuation(s)*computeArm(s);
}
double ClutchedPathSpring::showControl ( const SimTK::State& s ) const
{
		    return getControl(s);
}
double ClutchedPathSpring::showNControl ( const SimTK::State& s ) const
{
	    double control = SimTK::clamp(0.0, getControl(s), 1.0);
	    double Ncontrol=std::tanh(10*(control-.5))/2+.5;

		    return Ncontrol;
}
double ClutchedPathSpring::computeArm( const SimTK::State& s ) const
{
	string coordRef=get_coordRef();
	const Coordinate& aCoord = getModel().getCoordinateSet().get(coordRef);
	return getGeometryPath().computeMomentArm(s, aCoord);
}
double ClutchedPathSpring::computeActuation(const SimTK::State& s) const
{
    // clamp or cap the control input to [0, 1]
    double control = SimTK::clamp(0.0, getControl(s), 1.0); 
    control=std::tanh(5*(control-.5))/2+0.5;
    double tension =control*
    //            getStiffness()*(getLength(s)-get_initial_length()) ;   
                getStiffness()*getStretch(s) ;                 //elastic force
    setActuation(s, tension);
    return tension;
}

void ClutchedPathSpring::
    computeStateVariableDerivatives(const SimTK::State& s) const
{
	double zdot;
/*	if (get_initial_length()>0 )//restore mode
	       if(getControl(s)>SimTK::SignificantReal)//engaged	
		 if(getStretch(s)+get_initial_length()>=getLength(s))
            		zdot=getLengtheningSpeed(s);//going up on length curve
		 else
			zdot=getLength(s)/get_relaxation_time_constant();//going up fast
	       else//disangeged control<=0.5
			zdot=-getStretch(s)/get_relaxation_time_constant();//going down fast to 0
	else //residual mode
     		zdot = getControl(s) > .5 ? // non-zero control
                    getLengtheningSpeed(s) : // clutch is engaged up from current strech parallel
		    //to lengthing curve
                    -getStretch(s)/get_relaxation_time_constant();//down to zero*/
    double control = SimTK::clamp(0.0, getControl(s), 1.0); 
    if (control>0.5){
     double dist=getStretch(s)-(getLength(s)-get_initial_length());
     bool isStreching=(getLengtheningSpeed(s)>0);
     bool isLowStretch=(dist<=0);//dist negative
     bool isHighStretch=(dist>0);
     dist=abs(dist);
	   if      (isLowStretch and isStreching)
		  zdot=getLengtheningSpeed(s)*(1+dist/0.15*20);
	   else if (isLowStretch and not isStreching)
		//zdot=getLengtheningSpeed(s)*(1-pow(dist/0.15,0.4));
		zdot=getLengtheningSpeed(s)*(1-dist/0.13);
	   else if (isHighStretch and  isStreching)
		zdot=getLengtheningSpeed(s)*(1-dist/0.13);
	   else if (isHighStretch and  not isStreching)
		  zdot=getLengtheningSpeed(s)*(1+dist/0.15*20);
	   else 
	    	  zdot=getLengtheningSpeed(s);
    //cout<<"stretch:"+to_string(getStretch(s))+
//	"  need to be:"+to_string(getLength(s)-get_initial_length())+
//	" isStreching:"+string(isStreching?"T":"F")+
//	" realzdot:"+to_string(getLengtheningSpeed(s))+
  //      " zdot:"+to_string(zdot)+"\n";	
    }
    else
	    zdot=getLengtheningSpeed(s)*0.5;
    //           zdot = control > .5 ? // non-zero control
    //                getLengtheningSpeed(s) :0; // clutch is engaged up from current strech parallel
                    //to lengthing curve
                    //-getStretch(s)/get_relaxation_time_constant();//down to zero*/


    setStateVariableDerivativeValue(s, "stretch", zdot);
}

SimTK::Vec3 ClutchedPathSpring::computePathColor(const SimTK::State& state) const 
{
    double shade = SimTK::clamp(0.1, getControl(state), 1.0);
    const SimTK::Vec3 color(shade, 0.9, 0.1); // green to yellow
    return color;
}
