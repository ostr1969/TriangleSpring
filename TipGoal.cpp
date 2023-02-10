#include <OpenSim/Moco/osimMoco.h>
using namespace OpenSim;

class TipGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(TipGoal, MocoGoal);

public:
    TipGoal() {}
    TipGoal(std::string name) : MocoGoal(std::move(name)) {}
    TipGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(0, 1);
    }
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override {
        getModel().realizeVelocity(input.state);
        const auto& controls = getModel().getControls(input.state);
        integrand = controls.normSqr();
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
    auto& tip=getModel().getMarkerSet().get("tipr");
    //double tipinit_y=tip.getLocationInGround(input.initial_state)[1];
    double tipinit_y=tip.getLocationInGround(input.initial_state)[1]-0.02;

        cost[0] = tipinit_y*tipinit_y;
    }
};

