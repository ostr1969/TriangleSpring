#include <OpenSim/Moco/osimMoco.h>
using namespace OpenSim;
using namespace std;

class HeadStabGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(HeadStabGoal, MocoGoal);

public:
    HeadStabGoal() {constructProperties(); }
    HeadStabGoal(std::string name) : MocoGoal(std::move(name)) {constructProperties(); }
    HeadStabGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {constructProperties(); }
    void setHeadStabType(int typ) { set_stabtype(typ);
    switch(get_stabtype()){
	    case 0:
      		cout<<"***No head stabilizing goal***"<<endl;
		break;
	    case 1:
		cout<<"***veercamp 2021 head stabilizing goal***"<<endl;
		break;
	    case 2:
		cout<<"***Dorn 2015 head stabilizing goal***"<<endl;
		break;
	    case 3:
		cout<<"***Ong 2019 head stabilizing goal***"<<endl;
		break;
	    case 4:
		cout<<"***veercamp 2021 modified stabilizing goal***"<<endl;
		break;
	    default:
		cout<<"************wrong stabilizing type*****"<<endl;
		exit(300);
    }
    }
    int getHeadStabType() const { return get_stabtype(); }

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(1, 1);
    }
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override {
	if (get_stabtype()==0){
		integrand=0;
		return;}
        getModel().realizeAcceleration(input.state);
        const auto& controls = getModel().getControls(input.state);
    auto& head=getModel().getMarkerSet().get("head");
   // const auto&  headBody=baseModel.updBodySet().get("exomass_r")
   // cout<<head.getAbsolutePathString()<<endl;
    double aHead_y=head.getAccelerationInGround(input.state)[1];
    double aHead_x=head.getAccelerationInGround(input.state)[0];
    auto vHead=head.getVelocityInGround(input.state);
    auto vHead_x=head.getVelocityInGround(input.state)[0];
    auto aHead=head.getAccelerationInGround(input.state);
    auto comVel= getModel().calcMassCenterVelocity(input.state);
    auto comVel_x= getModel().calcMassCenterVelocity(input.state)[0];

       double vCom_Head=(comVel_x-vHead_x);//Dorn2015

       double intgx,intgy;//ong 2019
       double maxAx=2.5,minAx=-2.5,maxAy=5,minAy=-5;
       if (aHead_x>maxAx) 	intgx=aHead_x-maxAx;
       else if (aHead_x<minAx) 	intgx=minAx-aHead_x;
       else 			intgx=0;
       if (aHead_y>maxAy)	intgy=aHead_y-maxAy;
       else if (aHead_y<minAy)	intgy=minAy-aHead_y;
       else 			intgy=0;
       if (get_stabtype()==1)
           integrand = abs(aHead_y)+abs(aHead_x);//VeerKamp2021
       else if (get_stabtype()==2)
           integrand=abs(vCom_Head)>0.2?vCom_Head*vCom_Head:0;//Dorn2015
       else if (get_stabtype()==3)
       	   integrand=intgx*intgx+intgy*intgy;//Ong2019
       else if (get_stabtype()==4)
           integrand = pow(aHead_y,2)+pow(aHead_x,2);//VeerKamp2021_modified
       else if (get_stabtype()==0)
	   integrand=0;    
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        //getModel().realizeVelocity(input.initial_state);
        //getModel().realizeVelocity(input.final_state);
    auto comInitial= getModel().calcMassCenterPosition(input.initial_state);
    auto comFinal  = getModel().calcMassCenterPosition(input.final_state);
        cost[0] = input.integral/(comFinal-comInitial).norm()                 ;
    }
private:
    OpenSim_DECLARE_PROPERTY(stabtype, int,
            "type of head stabilizing,0=none, 1=veerkamp2021, 2=Dorn2015, 3=ong2019"
            "(default: 0).");
    void constructProperties(){
    constructProperty_stabtype(0);}
};

