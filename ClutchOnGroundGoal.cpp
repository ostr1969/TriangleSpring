#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/ClutchedPathSpring.h>
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>
using namespace OpenSim;
using std::cout;
using std::endl;

class ClutchOnGroundGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(ClutchOnGroundGoal, MocoGoal);

public:
    ClutchOnGroundGoal() {}
    ClutchOnGroundGoal(std::string name) : MocoGoal(std::move(name)) {}
    ClutchOnGroundGoal(std::string name, double weight)
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
        const auto& controls = getModel().getControls(input.state);
    const OpenSim::SmoothSphereHalfSpaceForce& tip = getModel().
            getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactFront_r");
  //  const OpenSim::SmoothSphereHalfSpaceForce& heel = getModel().
  //          getComponent<OpenSim::SmoothSphereHalfSpaceForce>("contactHeel_r");
    const Actuator& clutch=getModel().getComponent<Actuator>("/forceset/clutch_spring_r");
    double clutchControl=clutch.getControls(input.state)[0];
    const double fy=tip.getFy(input.state);//+heel.getFy(input.state);
    double ans;
    if (clutchControl>0.5 and fy<100) ans=1;
    else ans=0; 
        integrand = (fy<100)*clutchControl+(fy>=100)*(1-clutchControl);
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
    cost[0]=input.integral;
    }
};

