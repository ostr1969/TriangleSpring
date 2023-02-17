#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/TriangleSpring.h>
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>
using namespace OpenSim;
using std::cout;
using std::endl;

class TriWorkGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(TriWorkGoal, MocoGoal);

public:
    TriWorkGoal() {}
    TriWorkGoal(std::string name) : MocoGoal(std::move(name)) {}
    TriWorkGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(1, 1);
    }
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override {
        getModel().realizeVelocity(input.state);
    double kneeRspeed=getModel().
	    getStateVariableValue(input.state,"/jointset/knee_r/knee_angle_r/speed");
    const OpenSim::TriangleSpring& tri=getModel().
	    getComponent<OpenSim::TriangleSpring>("/forceset/aca");
    const OpenSim::SmoothSphereHalfSpaceForce& tip = getModel().
           getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactFront_r");
    const double tfy=tip.getFy(input.state);
    const double hfy=0;//heel.getFy(input.state);

	integrand=tri.computeTorque(input.state)*kneeRspeed;    
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
    //    getModel().realizeVelocity(input.initial_state);
   // const OpenSim::SmoothSphereHalfSpaceForce& tip = getModel().
   //         getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactFront_r");
   // const OpenSim::SmoothSphereHalfSpaceForce& heel = getModel().
   //         getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactHeel_r");
    //const double tfy=tip.getFy(input.initial_state);
    //const double hfy=heel.getFy(input.initial_state);
    //cost[0]=tfy;
    //cost[1]=hfy;
   // auto comInitial= getModel().calcMassCenterPosition(input.initial_state);
   // auto comFinal  = getModel().calcMassCenterPosition(input.final_state);
    cost[0] = abs(input.integral);// /(comFinal-comInitial).norm();
    }
};

