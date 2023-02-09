/* -------------------------------------------------------------------------- *
 * OpenSim Moco: example2DWalking.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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


#define MAIN   
#include <OpenSim/Moco/MocoGoal/MocoOutputGoal.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "additions.h"
//#include "MocoHighFoot.h"
#include "LowAcelGoal.cpp"
#include "GroundForceGoal.cpp"
#include "ClutchOnGroundGoal.cpp"
//#include "TrackVelGoal.cpp"
#include "LeftKneeAngleGoal.cpp"
//#include "HighFootGoal.cpp"
#include <OpenSim/Actuators/ClutchedPathSpring.h>
#include <unistd.h>


using namespace OpenSim;
using  std::cout;
using  std::endl;

/// Set a coordinate tracking problem where the goal is to minimize the
/// difference between provided and simulated coordinate values and speeds
/// (and ground reaction forces) as well as to minimize an effort cost (squared
/// controls). The provided data represents half a gait cycle. Endpoint
/// constraints enforce periodicity of the coordinate values (except for pelvis
/// tx) and speeds, coordinate actuator controls, and muscle activations.
///
/// If GRFTrackingWeight is set to 0 then GRFs will not be tracked. Setting
/// GRFTrackingWeight to 1 will cause the total tracking error (states + GRF) to
/// have about the same magnitude as control effort in the final objective
/// value.


// Set a gait prediction problem where the goal is to minimize effort (squared
// controls) over distance traveled while enforcing symmetry of the walking
// cycle and a prescribed average gait speed through endpoint constraints. The
// solution of the coordinate tracking problem is passed as an input argument
// and used as an initial guess for the PREDiction.
void writeMusclesWork(Model model,StatesTrajectory statesTraj);

void gaitPrediction() {

    using SimTK::Pi;
    dato=readvars();
    dato.print();
    char dir[100];
    string tr=dato.gets("predDirectory");
    strcpy(dir,"mkdir ");
    strcat(dir, tr.c_str());
    system(dir);

    char cmd[100];
    char hostname[10];
    strcpy(cmd,"printf '\033]2;%s\033\\' ");
    gethostname(hostname, 10);
    strcat(cmd, tr.c_str());
    strcat(cmd, ":");
    strcat(cmd, hostname);
    system(cmd);
    double newmass=dato.getd("newMass");

    string modelfile=dato.gets("predOsimfile");
    string modelfileName=modelfile;
    modelfileName=modelfileName.replace(0,4,"");
    string guessFile=dato.gets("predGuessFile");
    copyFile(guessFile,tr+"/guess.sto");
    copyFile("additions.h",tr+"/additions.h");
    copyFile("mocorun.cpp",tr+"/mocorun.cpp");
    copyFile("GroundForceGoal.cpp",tr+"/GroundForceGoal.cpp");
    copyFile("ClutchOnGroundGoal.cpp",tr+"/ClutchOnGroundGoal.cpp");
    copyFile("HighFootGoal.cpp",tr+"/HighFootGoal.cpp");
    copyFile("TipGoal.cpp",tr+"/TipGoal.cpp");
    copyFile("HeadStabGoal.cpp",tr+"/HeadStabGoal.cpp");
    copyFile("src/inpdata.txt",tr+"/inpdata.txt");
    copyFile(dato.gets("predOsimfile"),tr+"/"+modelfileName);
    MocoStudy study;
    study.setName("gaitPrediction");

    // Define the optimal control problem.
    // ===================================

    MocoProblem& problem = study.updProblem();
    //
    cout<<"loading "<<modelfile<<endl;
    Model baseModel(modelfile);
    cout<<"**setting mass**"<<endl;
    baseModel.updBodySet().get("exomass_r").setMass(0); 
    baseModel.updBodySet().get("exomass_l").setMass(0); 
    for(int i=0; i<baseModel.updBodySet().getSize(); i++){
            double cmass=baseModel.updBodySet()[i].getMass();
            baseModel.updBodySet()[i].setMass(cmass/62*newmass);}
    baseModel.updBodySet().get("exomass_r").setMass(1.5); 
    baseModel.updBodySet().get("exomass_l").setMass(1.5); 

    OpenSim::SmoothSphereHalfSpaceForce& c2=baseModel.
	    updComponent<OpenSim::SmoothSphereHalfSpaceForce>("/contactFront_r");
    c2.set_stiffness(dato.getd("ContactStiff"));

    // Add metabolics

    Bhargava2004SmoothedMuscleMetabolics* metabolics =
            new Bhargava2004SmoothedMuscleMetabolics();
    metabolics->setName("metabolics");
    metabolics->set_use_smoothing(true);
    metabolics->addMuscle("hamstrings_r",
            baseModel.getComponent<Muscle>("hamstrings_r"));
    metabolics->addMuscle("hamstrings_l",
            baseModel.getComponent<Muscle>("hamstrings_l"));
    metabolics->addMuscle("bifemsh_r",
            baseModel.getComponent<Muscle>("bifemsh_r"));
    metabolics->addMuscle("bifemsh_l",
            baseModel.getComponent<Muscle>("bifemsh_l"));
    metabolics->addMuscle("glut_max_r",
            baseModel.getComponent<Muscle>("glut_max_r"));
    metabolics->addMuscle("glut_max_l",
            baseModel.getComponent<Muscle>("glut_max_l"));
    metabolics->addMuscle("iliopsoas_r",
            baseModel.getComponent<Muscle>("iliopsoas_r"));
    metabolics->addMuscle("iliopsoas_l",
            baseModel.getComponent<Muscle>("iliopsoas_l"));
    metabolics->addMuscle("rect_fem_r",
            baseModel.getComponent<Muscle>("rect_fem_r"));
    metabolics->addMuscle("rect_fem_l",
            baseModel.getComponent<Muscle>("rect_fem_l"));
    metabolics->addMuscle("vasti_r",
            baseModel.getComponent<Muscle>("vasti_r"));
    metabolics->addMuscle("vasti_l",
            baseModel.getComponent<Muscle>("vasti_l"));
    metabolics->addMuscle("gastroc_r",
            baseModel.getComponent<Muscle>("gastroc_r"));
    metabolics->addMuscle("gastroc_l",
            baseModel.getComponent<Muscle>("gastroc_l"));
    metabolics->addMuscle("soleus_r",
            baseModel.getComponent<Muscle>("soleus_r"));
    metabolics->addMuscle("soleus_l",
            baseModel.getComponent<Muscle>("soleus_l"));
    metabolics->addMuscle("tib_ant_r",
            baseModel.getComponent<Muscle>("tib_ant_r"));
    metabolics->addMuscle("tib_ant_l",
            baseModel.getComponent<Muscle>("tib_ant_l"));
    baseModel.addComponent(metabolics);
    //    for (int i=0; i<metabolics->getNumMetabolicMuscles(); ++i)
    //            cout<<metabolics->getName()+"_"+
   // metabolics->get_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()[i].getName())>>endl;
    //            
    //const auto& calcn = dynamic_cast<Body&>(model.updComponent("calcn_r"));
    const auto& calcn_r=baseModel.updBodySet().get("calcn_r");
    const auto& calcn_l=baseModel.updBodySet().get("calcn_l");
    const auto& torso=baseModel.updBodySet().get("torso");
    auto* tipr = new Marker("tipr", calcn_r, Vec3(0.17740932, -0.01565376, 0.00521792126));
    baseModel.addMarker(tipr);
    auto* tipl = new Marker("tipl", calcn_l, Vec3(0.17740932, -0.01565376, -0.00521792126));
    baseModel.addMarker(tipl);
    auto* headMarker = new Marker("head", torso, Vec3(0,.6,0));
    baseModel.addMarker(headMarker);

    baseModel.finalizeConnections();
    double stif =dato.getd("springStif");
    double ang =dato.getd("springAng");
    auto* aca = new TriangleSpring(ang,stif,"knee_angle_r");
    aca->setName("aca");
    baseModel.addForce(aca);
    baseModel.finalizeFromProperties();
    baseModel.finalizeConnections();

//CLUTCH***************
 //maximum strech about 0.15 and raduis about 0.1
    double Nm=50;
    double stiffness =dato.getd("spring_Nm")*Nm ;//66.66*0.15*0.1=1 each 66.66 is 1Nm
    double dissipation = 0.003;


    ClutchedPathSpring* spring_r =
        new ClutchedPathSpring("clutch_spring_r", stiffness, dissipation, 0.1);
    PhysicalFrame& fem=baseModel.updComponent<PhysicalFrame>("/bodyset/femur_r");
    spring_r->updGeometryPath().appendNewPathPoint("origin", fem, Vec3(0.1, -0.44 ,0.0));
    PhysicalFrame& tib=baseModel.updComponent<PhysicalFrame>("/bodyset/tibia_r");
    spring_r->updGeometryPath().appendNewPathPoint("insertion", tib , Vec3(0.1, -0.2 ,0.0));
    spring_r->set_relaxation_time_constant(dato.getd("springRelax"));
    spring_r->set_coordRef("knee_angle_r");
   // baseModel.addForce(spring_r);
    ClutchedPathSpring* spring_l =
        new ClutchedPathSpring("clutch_spring_l", stiffness, dissipation, 0.1);
    PhysicalFrame& fem2=baseModel.updComponent<PhysicalFrame>("/bodyset/femur_l");
    spring_l->updGeometryPath().appendNewPathPoint("origin", fem2, Vec3(0.1, -0.44 ,0.0));
    PhysicalFrame& tib2=baseModel.updComponent<PhysicalFrame>("/bodyset/tibia_l");
    spring_l->updGeometryPath().appendNewPathPoint("insertion", tib2 , Vec3(0.1, -0.2 ,0.0));
    spring_l->set_relaxation_time_constant(dato.getd("springRelax"));
    spring_l->set_coordRef("knee_angle_l");
    //baseModel.addForce(spring_l);
    baseModel.finalizeConnections();
//END CLUTCH
  double speedWight=dato.getd("trackSpeedWeight");
  double angleWight=dato.getd("trackAngleWeight");
	MocoWeightSet myWeights;
    myWeights.cloneAndAppend({"/jointset/knee_r/knee_angle_r/speed",  speedWight});
    myWeights.cloneAndAppend({"/jointset/knee_l/knee_angle_l/speed",  speedWight});
    myWeights.cloneAndAppend({"/jointset/ankle_r/ankle_angle_r/speed",  speedWight});
    myWeights.cloneAndAppend({"/jointset/ankle_l/ankle_angle_l/speed",  speedWight});
    myWeights.cloneAndAppend({"/jointset/hip_r/hip_flexion_r/speed",  speedWight});
    myWeights.cloneAndAppend({"/jointset/hip_l/hip_flexion_l/speed",  speedWight});
    myWeights.cloneAndAppend({"/jointset/knee_r/knee_angle_r/value",  angleWight});
    myWeights.cloneAndAppend({"/jointset/knee_l/knee_angle_l/value",  angleWight});
    myWeights.cloneAndAppend({"/jointset/ankle_r/ankle_angle_r/value",  angleWight});
    myWeights.cloneAndAppend({"/jointset/ankle_l/ankle_angle_l/value",  angleWight});
    myWeights.cloneAndAppend({"/jointset/hip_r/hip_flexion_r/value",  angleWight});
    myWeights.cloneAndAppend({"/jointset/hip_l/hip_flexion_l/value",  angleWight});
    myWeights.cloneAndAppend({"/jointset/ground_pelvis/pelvis_tx/value", 100});

    double mf=dato.getd("MuscleFactor");
    DeGrooteFregly2016Muscle& ham_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/hamstrings_r");
    ham_r.set_coordRef1("knee_angle_r");
    ham_r.set_coordRef2("hip_flexion_r");
    ham_r.set_max_isometric_force(ham_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& bif_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/bifemsh_r");
    bif_r.set_coordRef1("knee_angle_r");
    bif_r.set_max_isometric_force(bif_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& gas_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/gastroc_r");
    gas_r.set_coordRef1("knee_angle_r");
    gas_r.set_coordRef2("ankle_angle_r");
    gas_r.set_max_isometric_force(gas_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& rec_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/rect_fem_r");
    rec_r.set_coordRef1("knee_angle_r");
    rec_r.set_coordRef2("hip_flexion_r");
    rec_r.set_max_isometric_force(rec_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& vas_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/vasti_r");
    vas_r.set_coordRef1("knee_angle_r");
    vas_r.set_max_isometric_force(vas_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& glu_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/glut_max_r");
    glu_r.set_coordRef1("hip_flexion_r");
    glu_r.set_max_isometric_force(glu_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& ili_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/iliopsoas_r");
    ili_r.set_coordRef1("hip_flexion_r");
    ili_r.set_max_isometric_force(ili_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& sol_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/soleus_r");
    sol_r.set_coordRef1("ankle_angle_r");
    sol_r.set_max_isometric_force(sol_r.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& tib_r=baseModel.updComponent<DeGrooteFregly2016Muscle>("/tib_ant_r");
    tib_r.set_coordRef1("ankle_angle_r");
    tib_r.set_max_isometric_force(tib_r.get_max_isometric_force()*mf);


    DeGrooteFregly2016Muscle& ham_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/hamstrings_l");
    ham_l.set_coordRef1("knee_angle_l");
    ham_l.set_coordRef2("hip_flexion_l");
    ham_l.set_max_isometric_force(ham_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& bif_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/bifemsh_l");
    bif_l.set_coordRef1("knee_angle_l");
    bif_l.set_max_isometric_force(bif_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& gas_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/gastroc_l");
    gas_l.set_coordRef1("knee_angle_l");
    gas_l.set_coordRef2("ankle_angle_l");
    gas_l.set_max_isometric_force(gas_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& rec_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/rect_fem_l");
    rec_l.set_coordRef1("knee_angle_l");
    rec_l.set_coordRef2("hip_flexion_l");
    rec_l.set_max_isometric_force(rec_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& vas_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/vasti_l");
    vas_l.set_coordRef1("knee_angle_l");
    vas_l.set_max_isometric_force(vas_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& glu_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/glut_max_l");
    glu_l.set_coordRef1("hip_flexion_l");
    glu_l.set_max_isometric_force(glu_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& ili_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/iliopsoas_l");
    ili_l.set_coordRef1("hip_flexion_l");
    ili_l.set_max_isometric_force(ili_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& sol_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/soleus_l");
    sol_l.set_coordRef1("ankle_angle_l");
    sol_l.set_max_isometric_force(sol_l.get_max_isometric_force()*mf);
    DeGrooteFregly2016Muscle& tib_l=baseModel.updComponent<DeGrooteFregly2016Muscle>("/tib_ant_l");
    tib_l.set_coordRef1("ankle_angle_l");
    tib_l.set_max_isometric_force(tib_l.get_max_isometric_force()*mf);

    for (auto& muscle : baseModel.updComponentList<DeGrooteFregly2016Muscle>()) {
//    for (int i = 0; i < muscles.getSize(); i++) {
	muscle.set_activation_time_constant(dato.getd("MuscleActConstant"));
	muscle.set_deactivation_time_constant(dato.getd("MuscleDeactConstant"));
	muscle.set_max_contraction_velocity(dato.getd("MuscleContractionVel")); 
    }
    ModelProcessor modelprocessor = ModelProcessor(baseModel);
    //
    //ModelProcessor modelprocessor =
    //        ModelProcessor("2D_gait.osim");
    problem.setModelProcessor(modelprocessor);

    // Goal_s.
    // =====
    // Symmetry.
    // 		@@@ PERIODIC GOAL
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    Model model = modelprocessor.process();
    model.initSystem();
    // Symmetric coordinate values (except for pelvis_tx) and speeds.
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        if (endsWith(coord.getName(), "_r")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_r"), "_l")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_r"), "_l")});
        }
        if (endsWith(coord.getName(), "_l")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_l"), "_r")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_l"), "_r")});
        }
        if (!endsWith(coord.getName(), "_l") &&
                !endsWith(coord.getName(), "_r") &&
                !endsWith(coord.getName(), "_tx")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    coord.getStateVariableNames()[0]});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    coord.getStateVariableNames()[1]});
        }
    }
    symmetryGoal->addStatePair({"/jointset/ground_pelvis/pelvis_tx/speed"});
    // Symmetric coordinate actuator controls.
    symmetryGoal->addControlPair({"/lumbarAct"});
    // Symmetric muscle activations.
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (endsWith(muscle.getName(), "_r")) {
            symmetryGoal->addStatePair({muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_r"), "_l")});
        }
        if (endsWith(muscle.getName(), "_l")) {
            symmetryGoal->addStatePair({muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_l"), "_r")});
        }
    }
    //END PRIODIC GOAL
    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speed");
    speedGoal->set_desired_average_speed(dato.getd("predVelocity"));
    //problem.addGoal<MocoHighFoot>("highfoot",0);

    auto* metGoal = problem.addGoal<MocoOutputGoal>("met",dato.getd("predMetWeight"));
    metGoal->setOutputPath("/metabolics|total_metabolic_rate");
    metGoal->setDivideByDisplacement(true);
    metGoal->setDivideByMass(true);
    //		@@@ CLUTCH ACTIVATED ON GROUND
    if (dato.getd("predTrackWeight")!=0){
    	auto* stateTrackingGoal = problem.addGoal<MocoStateTrackingGoal>("statetrack",
		    dato.getd("predTrackWeight"));
	TableProcessor ref=TableProcessor("src/itay_average.sto");    
   	stateTrackingGoal ->setReference(ref);
    	stateTrackingGoal->setWeightSet(myWeights);}
    if (stiffness>0)		
    	auto* clutcOnGroundGoal=problem.addGoal<ClutchOnGroundGoal>("clutch_on_ground_goal",2); 
    if (dato.getd("EffortWeight")!=0)
    	auto* effort = problem.addGoal<MocoControlGoal>("effort",
		    dato.getd("EffortWeight"));
    if (dato.getd("LkneeAngleWeight")!=0)
    problem.addGoal<LeftKneeAngleGoal>("leftkneeangle",
		    dato.getd("LkneeAngleWeight"));
    //		@@@ LOW GROUND FORCE
    if (dato.getd("GroundForceWeight")!=0)
    problem.addGoal<GroundForceGoal>("groundforce",dato.getd("GroundForceWeight"));//20
    //END GOALS
    // Bounds.
    // =======
    double t1=dato.getd("LowestTime");
    problem.setTimeBounds(0, {t1 , 0.35});//0.3386
    problem.setStateInfo("/jointset/ground_pelvis/pelvis_tilt/value",
            {-30 * Pi / 180, -10 * Pi / 180});
    problem.setStateInfo("/jointset/ground_pelvis/pelvis_tx/value", {0, 1.25},0);
    problem.setStateInfo(
            "/jointset/ground_pelvis/pelvis_ty/value", {0.75, 1});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-15 * Pi / 180, 60 * Pi / 180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-15 * Pi / 180, 60 * Pi / 180});
    problem.setStateInfo(
            "/jointset/knee_l/knee_angle_l/value", {-120 * Pi / 180, 10*Pi/180},-54*Pi/180);
    problem.setStateInfo(
            "/jointset/knee_r/knee_angle_r/value", {-120 * Pi / 180, 10*Pi/180});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-30 * Pi / 180, 30 * Pi / 180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-30 * Pi / 180, 20 * Pi / 180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value", {8*Pi/180, 20 * Pi / 180});
   MocoParameter p0("aca_stif", "/forceset/aca", "stiffness", MocoBounds(0,15) );
  // problem.addParameter(p0);
  //  problem.setStateInfo("/forceset/clutch_spring_r/stretch", {0,0.2});
  //  problem.setStateInfo("/forceset/clutch_spring_l/stretch", {0,0.2});
    //problem.setControlInfo("/forceset/clutch_spring_r",{0,1}, {0,1});
    //problem.setControlInfo("/forceset/clutch_spring_l",{0,0}, {0,0});
  //  problem.setControlInfo("/forceset/clutch_spring_r",{0,1}, {0});
  //  problem.setControlInfo("/forceset/clutch_spring_l",{0,0}, {0,0});

    // Configure the solver.
    // =====================
    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(dato.geti("predDivisions"));
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(dato.getd("predConvergence"));
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(dato.geti("predNumIterations"));
    // Use the solution from the tracking simulation as initial guess.
    MocoTrajectory initGuess;
    TimeSeriesTable states0;
    TimeSeriesTable controls0;

   //ADD MISSING CONTROL AND STATE TO THE GUESS FILE
   initGuess=MocoTrajectory(dato.strings[2].val); 
   auto time = initGuess.getTime();
   auto endtim=time(time.size()-1);
   vector<double> tim1={0,endtim};

   TimeSeriesTable addedStates(tim1);
   TimeSeriesTable addedControls(tim1);
   SimTK::Vector zeros(2, 0.0);
   SimTK::Vector ones(2, 1.0);
   //addedStates.appendColumn("/forceset/clutch_spring_l/stretch", zeros);
   //addedStates.appendColumn("/forceset/aca", zeros);
   addedControls.appendColumn("/forceset/aca", zeros);
   //addedControls.appendColumn("/forceset/clutch_spring_r", zeros);
   //initGuess.insertStatesTrajectory(addedStates);
  // initGuess.insertControlsTrajectory(addedControls);
   // initGuess.removeState("/forceset/clutch_spring_l/stretch");
   // initGuess.removeControl("/forceset/clutch_spring_l");
   // initGuess.removeState("/forceset/clutch_spring_r/stretch");
   // initGuess.removeControl("/forceset/clutch_spring_r");
  // initGuess.appendParameter("aca_stif",1);
   if (dato.geti("predIsGuess"))
   solver.setGuess(initGuess);



    baseModel.print(dato.strings[0].val+"/2DrunWithMetabolic.osim");


    SimTK::State state = baseModel.initSystem();
    cout<<"TOTAL MASS:"<<baseModel.getTotalMass(state)<<endl;

    // Solve problem.
    // ==============
    MocoSolution solution = study.solve();

    solution.unseal();
    auto full = createPeriodicTrajectory(solution);
    full.write(tr+"/pred3_solution_fullcycle.sto");
    solution.write(tr+"/bred3_solution.sto");
    auto statesTraj = solution.exportToStatesTrajectory(problem);

    TimeSeriesTable statesTable=solution.exportToStatesTable();
    STOFileAdapter::write(statesTable, tr+"/pred3_states.sto");

    TimeSeriesTable controlTable=solution.exportToControlsTable();
    STOFileAdapter::write(controlTable, tr+"/pred3_controls.sto");
    //GET COM DISTANCE
    //StatesTrajectory stTraj=StatesTrajectory::createFromStatesStorage(baseModel,
    //		    tr+"/pred3_solution.sto",true,true);
    auto& initState=statesTraj.get(0);
    auto& finalState=statesTraj.get(statesTraj.getSize()-1);
    auto comInitial= baseModel.calcMassCenterPosition(initState);
    auto mass=baseModel.getTotalMass(initState);
    auto comFinal= baseModel.calcMassCenterPosition(finalState);
    double comDist=(comFinal - comInitial).norm();
    //cout<<"after comdist"<<endl;

    TimeSeriesTable outputTable =
	    analyzeMocoTrajectory<double>(baseModel, solution,
            {"/jointset/.*/.*\\|value",
             "/jointset/.*/.*\\|speed",
	     "/markerset/tipr\\|loc_y",
	     "/markerset/tipl\\|loc_y",
	     "/markerset/head\\|acc_y",
	     "/markerset/head\\|acc_x",
	     "/contactFront_r\\|fy",
	     "/contactHeel_r\\|fy",
	     "/.*\\|torque1",
	     "/.*\\|torque2",
	     "/forceset/clutch_.*\\|torque",
	     "/forceset/clutch_.*\\|control",
	     "/forceset/aca\\|torque",
	     "/forceset/aca\\|angle",
	     "/metabolics\\|total_metabolic_rate"

	     });
    //cout<<"before metadata"<<endl;
    double metWeight= dato.getd("predMetWeight");
    auto met=solution.getObjectiveTerm("met")/metWeight;
    auto metWkg=met/solution.getFinalTime()*comDist;
    outputTable.updTableMetaData().setValueForKey("met",std::to_string(metWkg));
    outputTable.updTableMetaData().setValueForKey("mass",std::to_string(mass));
    outputTable.updTableMetaData().setValueForKey("comdist",std::to_string(comDist));
    //cout<<"after metadata"<<endl;
    TimeSeriesTable_<SimTK::Vec<3>> outputTable2 =
	    analyzeMocoTrajectory<SimTK::Vec<3>>(baseModel, solution,
            {
	     ".*com_.*",
	     "/markerset/head\\|location",
	     "/markerset/head\\|velocity",
	     "/markerset/head\\|acceleration",
	     "/bodyset/.*\\|linear_acceleration",
	    });
     std::ofstream workFile(tr+"/musclesWork.sto");
     workFile<<"endheader"<<endl<<"time";
    auto& metabolic_Probe =
	     model.getComponent<Bhargava2004SmoothedMuscleMetabolics>("metabolics");
   int n=metabolic_Probe.getNumMetabolicMuscles(); 
   for(int i=0;i<n;i++)
   	workFile<< "\t"<<metabolic_Probe.get_muscle_parameters(i).getName();
   workFile<<endl;
   for(auto s:statesTraj){ 
	workFile<<s.getTime();   
   	model.realizeDynamics(s);
   	auto& finalVal = 
		metabolic_Probe.getOutputValue<Vector>(s, "muscles_work");
   	for(int i=0;i<n;i++)
   		workFile<<"\t"<<finalVal(i);
   	workFile<<endl;
   }
   workFile.close();
    STOFileAdapter_<SimTK::Vec<3>>::write(outputTable2, tr+"/toutputs3.sto");
    //cout<<"after write output3"<<endl;



    // Extract ground reaction forces.
    // ===============================
   // std::vector<std::string> contact_front;
   // std::vector<std::string> contact_heel;
    //contact_heel.push_back("contactHeel_r");
   // contact_r.push_back("contactFront_r");
   // contact_l.push_back("contactHeel_r");
    //contact_front.push_back("contactFront_r");
    //TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
//			            model, full, contact_r, contact_l);
//   STOFileAdapter::write(externalForcesTableFlat,
//			                "gaitPrediction_solutionGRF_fullcycle.sto");



    TimeSeriesTableVec3 externalForcesTable;
    int count = 0;
    baseModel.initSystem();
    for (const auto& state : statesTraj) {
        model.realizeVelocity(state);
        SimTK::Vec3 heelForce(0);
        SimTK::Vec3 heelTorque(0);
        SimTK::Vec3 frontForce(0);
        SimTK::Vec3 frontTorque(0);
            Array<double> forceValues =
            baseModel.getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactFront_r").
		    getRecordValues(state);
            heelForce += SimTK::Vec3(forceValues[0], forceValues[1],
                    forceValues[2]);
            heelTorque += SimTK::Vec3(forceValues[3], forceValues[4],
                    forceValues[5]);
            Array<double> forceValues2 =
            baseModel.getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactFront_r").
		    getRecordValues(state);
            frontForce += SimTK::Vec3(forceValues2[0], forceValues2[1],
                    forceValues2[2]);
            frontTorque += SimTK::Vec3(forceValues2[3], forceValues2[4],
                    forceValues2[5]);

        SimTK::RowVector_<SimTK::Vec3> row(6);
        row(0) = heelForce;
        row(1) = SimTK::Vec3(0);
	row(2)=heelTorque;
        row(3) = SimTK::Vec3(0);
        row(4) = frontForce;
	row(5)=frontTorque;
        externalForcesTable.appendRow(state.getTime(), row);
        ++count;

    }
    std::vector<std::string> labels{"heel_force_r_v","heel_force_r_p",
            "heel_torque_r_", "front_force_r_v","front_force_r_p",
            "front_torque_r_"};
    externalForcesTable.setColumnLabels(labels);
    TimeSeriesTable externalForcesTableFlat =
            externalForcesTable.flatten({"x", "y", "z"});

 //   TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
 //           model, solution, contact_front, contact_heel);
    STOFileAdapter::write(externalForcesTableFlat,dato.strings[0].val+
            "/pred3_solutionGRF.sto");
   // auto& metabolic_Probe =
	//	            model.getComponent<Bhargava2004Metabolics>("metabolics");
//	cout<<"before clc vrms"<<endl;
	vector<double> vrms=calcVrms(solution);
	cout<<"vrms: ang="<<vrms[0]*vrms[1]*vrms[2]<<
		" vel="<<vrms[3]*vrms[4]*vrms[5]<<" all="<<
		vrms[0]*vrms[1]*vrms[2]*vrms[3]*vrms[4]*vrms[5]<<endl;
    outputTable.updTableMetaData().setValueForKey("vrms_a",std::to_string(vrms[0]*vrms[1]*vrms[2]));
    outputTable.updTableMetaData().setValueForKey("vrms_v",std::to_string(vrms[3]*vrms[4]*vrms[5]));
    outputTable.updTableMetaData().setValueForKey("vrms_all",std::to_string(vrms[3]*vrms[4]*vrms[5]*
			    vrms[0]*vrms[1]*vrms[2]));
    cout<<tr+"/outputs.sto"<<endl;
    STOFileAdapter_<double>::write(outputTable, tr+"/outputs.sto");


    auto tx=solution.getState("/jointset/ground_pelvis/pelvis_tx/value");   
    int s=tx.size();
    //dato.print();
    markStos(tr,tr);
   // copyFile(dato.strings[1].val,dato.strings[0].val+"/"+modelfile);

    //addLineToEndOfFile(dato.strings[0].val+"/inpdata.txt","s met_result "+to_string(met)); 
    cout<<"last tx:"<<tx[s-1]<<" Velocity" << (tx[s-1]-tx[0])/(solution.getFinalTime()-solution.getInitialTime())<<endl;

    cout<<"\nFINALTIME: " <<solution.getFinalTime()<<", starttime: "<<solution.getInitialTime()<<endl;
    
    std::cout << "The metabolic cost of transport is: "
        << met << " [J kg-1 m-1]." << std::endl;
    cout<<"met:"<<met/solution.getFinalTime()*comDist<<"W/Kg"<<endl;
}

int main() {
    try {
        gaitPrediction();
    } catch (const std::exception& e) { std::cout << e.what() << std::endl; }
    return EXIT_SUCCESS;
}
