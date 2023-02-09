#include <fstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <filesystem>
#include <dirent.h>
#include <sys/types.h>
#include "Utils.cpp"

using namespace OpenSim;
using namespace SimTK;
using namespace std;
using std::cout;
using std::endl;

template<typename T>
struct Var{
	T val;
	string label;};

struct InpVars{
	vector<Var<string>> strings;
	vector<Var<int>> ints;
	vector<Var<double>> doubles;
	void print(){
    cout<<"VARS:"<<endl;
    int sss=strings.size();
    for (int i=0;i<sss;i++)
    cout<<i<<"(string) "<<strings[i].label<<"="<<strings[i].val<<endl;
    sss=ints.size();
    for (int i=0;i<sss;i++)
    cout<<i<<"(int) "<<ints[i].label<<"="<<ints[i].val<<endl;
    sss=doubles.size();
    for (int i=0;i<sss;i++)
    cout<<i<<"(double) "<<doubles[i].label<<"="<<doubles[i].val<<endl;

	}
    double  getd(std::string label)
	{
	for (auto d: doubles){if (label==d.label)return d.val;}
	cout<<"Error: No such label:"<<label<<endl;
	exit(0);
	}
   std::string  gets(std::string label)
	{
	for (auto s: strings){if (label==s.label)return s.val;}
	cout<<"Error: No such label:"<<label<<endl;
	exit(0);
	}
    int  geti(std::string label)
	{
	for (auto i: ints){if (label==i.label)return i.val;}
	cout<<"Error: No such label:"<<label<<endl;
	exit(0);
	}
};
vector<double> getcol(MocoSolution solution,string var){ 
	//get state or time  column from soolution as vectors
	vector<double> col;
	Vector tx;
	if (var=="time")
 		tx=solution.getTime();
	else
    		tx=solution.getState(var);   
    int s=tx.size();
    for (auto d:tx)
	    col.push_back(d);
    return col;
}
vector<double> getcolRL(MocoSolution solution,string rvar){
	//concatanate left and right state columns or create double time column
	if (rvar=="time"){
 		vector<double> tx=getcol(solution,"time");
		double maxtime=tx.back();
		for (double d:tx)
			tx.push_back(d+maxtime);
		return tx;
	}

    vector<double> r=getcol(solution,rvar); 
          string  lvar=std::regex_replace(rvar, std::regex("_r"), "_l");
    vector<double> l=getcol(solution,lvar); 
    r.insert( r.end(), l.begin(), l.end() );
    return r;
}
vector<double> calcVrms(MocoSolution solution){

	std::ifstream       file ("src/itayAverage.csv");
	vector<double> col1=getcolRL(solution,"/jointset/ankle_r/ankle_angle_r/value");
	vector<double> col2=getcolRL(solution,"/jointset/knee_r/knee_angle_r/value");
	vector<double> col3=getcolRL(solution,"/jointset/hip_r/hip_flexion_r/value");
	vector<double> col4=getcolRL(solution,"/jointset/ankle_r/ankle_angle_r/speed");
	vector<double> col5=getcolRL(solution,"/jointset/knee_r/knee_angle_r/speed");
	vector<double> col6=getcolRL(solution,"/jointset/hip_r/hip_flexion_r/speed");
	vector<double> tim=getcolRL(solution,"time");
	//get itay average as ref to calculate rms
        CSVRange mycsv=CSVRange(file);
	vector<double> timref=mycsv.col(0);
	vector<double> col1ref=mycsv.multCol(1,Pi/180);
	vector<double> col2ref=mycsv.multCol(2,Pi/180);
	vector<double> col3ref=mycsv.multCol(3,Pi/180);
	vector<double> col4ref=mycsv.col(10);
	vector<double> col5ref=mycsv.col(11);
	vector<double> col6ref=mycsv.col(12);
	double res1=Vrms(tim,col1,timref,col1ref);
	double res2=Vrms(tim,col2,timref,col2ref);
	double res3=Vrms(tim,col3,timref,col3ref);
	double res4=Vrms(tim,col4,timref,col4ref);
	double res5=Vrms(tim,col5,timref,col5ref);
	double res6=Vrms(tim,col6,timref,col6ref);
	cout<<"vrms :"<<
		res1<<","<<res2<<","<<res3<<","<<res4<<","<<res5<<","<<res6<<endl;
	vector<double> vrms={res1,res2,res3,res4,res5,res6};

	return vrms;
}


void addLineToStartOfFile(string inputfile,string line){
//	const char* inputfile=inputfile_;
    std::ofstream outputFile("tmpOutput");
    std::ifstream inputFile(inputfile);
    outputFile << line<<endl;
    outputFile << inputFile.rdbuf();

    inputFile.close();
    outputFile.close();

    std::remove(inputfile.c_str());
    std::rename("tmpOutput",inputfile.c_str());
}
void addLineToEndOfFile(string inputfile,string line){
//	const char* inputfile=inputfile_;
    std::ofstream outputFile("tmpOutput");
    std::ifstream inputFile(inputfile);
    outputFile << inputFile.rdbuf();
    outputFile << line<<endl;

    inputFile.close();
    outputFile.close();

    std::remove(inputfile.c_str());
    std::rename("tmpOutput",inputfile.c_str());
    cout<<"added line to end of "+inputfile<<endl;
}
InpVars readvars(){
  //FILE * ifile;
  //ifile = fopen ("src/input.txt","r");
  InpVars data;string typ,label,val;
  std::ifstream ifile("src/inpdata.txt");
  while (ifile>>typ>>label>>val) {
        //ifile>>typ>>label>>val;
	ifile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        if (!typ.compare("d"))
        {Var<double> tmp;tmp.label=label;tmp.val=atof(val.c_str());
           data.doubles.push_back(tmp);}
        if (!typ.compare("i"))
        {Var<int> tmp;tmp.label=label;tmp.val=atoi(val.c_str());
           data.ints.push_back(tmp);}
        if (!typ.compare("s"))
        {Var<string> tmp;tmp.label=label;tmp.val=val;
           data.strings.push_back(tmp);}
}
  ifile.close();
return data;
}
InpVars dato;

void printAllOutputs(Model model){
	model.finalizeFromProperties();
    for (const auto& comp : model.getComponentList()) {
        for (const auto& outputName : comp.getOutputNames()) {
            const auto& output = comp.getOutput(outputName);
		cout<<output.getPathName()<<endl;
        }
    }

}

// copy in binary mode
bool copyFile(string SRC, string DEST)
{

    std::ifstream src(SRC, std::ios::binary);
    std::ofstream dest(DEST, std::ios::binary);
    dest << src.rdbuf();
    cout<<"copy file "<<SRC<<" to "<<DEST<<endl;
    return src && dest;
}
bool endsWith( std::string s,  std::string suffix)
{
	  if (s.length()<suffix.length())
		  return 0;
	  else
	    return s.rfind(suffix) == labs(s.length()-suffix.length());
}
bool markStos(string path,string line){
    struct dirent *entry;
    DIR *dir = opendir(path.c_str());
    if (dir == NULL) {
        return 0;
	cout<<path<<" not dir"<<endl;
    }

    while ((entry = readdir(dir)) != NULL) {
	if (endsWith(entry->d_name,".sto")){
        //printf("add line to start of %s\n",entry->d_name);
	addLineToStartOfFile(path+"/"+entry->d_name,line);
	}
    }
    closedir(dir);
    return 1;
}
