#include <OpenSim/Moco/osimMoco.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;


class LowAcelGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(LowAcelGoal, MocoGoal);

public:
    LowAcelGoal() {}
    LowAcelGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
	}
    LowAcelGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
	}
    LowAcelGoal(std::string name, double weight,double limitAcel,std::string targetname)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
	setTargetColName(targetname);
	setLimitAcel(limitAcel);
//cout<<weight<<","<<limitAcel<<","<<targetname<<endl;
	}
    void setTargetColName(std::string colname) {
        set_targetcolname(colname);
    }
    void setLimitAcel(double limitAcel){
	    set_limitacel(limitAcel);
    }
private:
    OpenSim_DECLARE_PROPERTY(targetcolname, std::string,
            "joint where the acel is limited ");
    OpenSim_DECLARE_PROPERTY(limitacel,double,
            "limit aceleration ");
    void constructProperties() {
        constructProperty_targetcolname("");
	constructProperty_limitacel(0);
    }

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }

    void initializeOnModelImpl(const Model& model) const override {
        setRequirements(1, 1);
    }
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override {
        getModel().realizeAcceleration(input.state);
    double tim=input.time;
    const Coordinate &coord=getModel().getCoordinateSet().
	                get(get_targetcolname());
    double jointAcel=coord.getAccelerationValue(input.state);

    integrand=abs(jointAcel)>2?abs(jointAcel):0; 

    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
    cost[0]=input.integral;
    }
};
