#include <unistd.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <string_view>

using namespace std;
vector<int> indices(vector<double> v){
//	get vector of indices from 0 to size of v
	vector<int> inds;
	for(int i=0; i < v.size(); i++) inds.push_back(i);
	return inds;
}
void Vout(string txt,vector<double> v){
	cout<<txt<<"=";
	for (double d:v)
		cout<<d<<",";
	cout<<"\n";
}

void clip(vector<double> t,vector<double> v,double tmin,double tmax,
		vector<double>& tout,vector<double>& vout ){
	//clip t and related v between tmin and tmux
	//tout cliped out vector and vout related v
	for(int i : indices(t)) {
		if (t[i]<=tmax and t[i]>=tmin){
		    tout.push_back(t[i]);
		    vout.push_back(v[i]); }
	}

}
double interp(vector<double> t, vector<double> v, double p){
	cout.precision(5);
	for (int i:indices(t)){
		if (p==t[i])
			return v[i];
		else if (p>t[i] and p<=t[i+1]){
			//cout<<p<<" "<<t[i]<<":"<<t[i+1]<<endl;
			//cout<<"  "<<v[i]<<":"<<v[i+1]<<endl;
			return (v[i+1]-v[i])/(t[i+1]-t[i])*(p-t[i])+v[i];
		}}
		

return 0;}
double Vrms(vector<double> t,vector<double> v,vector<double> tref,vector<double> vref){
	double trefmin=tref[0];
	double trefmax=tref.back();
	vector<double> allsq;
	vector<double> cliped_t,cliped_v;
	clip(t,v,trefmin,trefmax,cliped_t,cliped_v);
	//Vout("cliped_t",cliped_t);
	//Vout("cliped_v",cliped_v);
	//Vout("tref",tref);
	//Vout("vref",vref);
	int nt=cliped_t.size();
	double sqDist=0;
	cout.precision(5);
	for(int i:indices(cliped_t)){
		double vrefAtt=interp(tref,vref,cliped_t[i]);
	//	cout<<cliped_t[i]<<" v="<<vrefAtt<<endl;
		sqDist+=pow(cliped_v[i]-vrefAtt,2)/nt;}
	//cout<<sqrt(sqDist)<<std::endl;
	return sqrt(sqDist);

}
class CSVRow
{
    public:
	    std::string_view operator[](std::size_t index) const
        {
            return string_view(&m_line[m_data[index] + 1], m_data[index + 1] -  (m_data[index] + 1));
        }
        std::size_t size() const
        {
            return m_data.size() - 1;
        }
        void readNextRow(std::istream& str)
        {
            std::getline(str, m_line);

            m_data.clear();
            m_data.emplace_back(-1);
            std::string::size_type pos = 0;
            while((pos = m_line.find(',', pos)) != std::string::npos)
            {
                m_data.emplace_back(pos);
                ++pos;
            }
            // This checks for a trailing comma with no data after it.
            pos   = m_line.size();
            m_data.emplace_back(pos);
        }
    private:
        std::string         m_line;
        std::vector<int>    m_data;
};
std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}
class CSVIterator
{   
    public:
        typedef std::input_iterator_tag     iterator_category;
        typedef CSVRow                      value_type;
        typedef std::size_t                 difference_type;
        typedef CSVRow*                     pointer;
        typedef CSVRow&                     reference;

        CSVIterator(std::istream& str)  :m_str(str.good()?&str:nullptr) { ++(*this); }
        CSVIterator()                   :m_str(nullptr) {}

        // Pre Increment
        CSVIterator& operator++()               {if (m_str) { if (!((*m_str) >> m_row)){m_str = nullptr;}}return *this;}
        // Post increment
        CSVIterator operator++(int)             {CSVIterator    tmp(*this);++(*this);return tmp;}
        CSVRow const& operator*()   const       {return m_row;}
        CSVRow const* operator->()  const       {return &m_row;}

        bool operator==(CSVIterator const& rhs) {return ((this == &rhs) || ((this->m_str == nullptr) && (rhs.m_str == nullptr)));}
        bool operator!=(CSVIterator const& rhs) {return !((*this) == rhs);}
    private:
        std::istream*       m_str;
        CSVRow              m_row;
};
class CSVRange
{
    std::istream&   stream;
    public:
        CSVRange(std::istream& file) : stream(file)
        {
	}
        CSVIterator begin() const {return CSVIterator{stream};}
        CSVIterator end()   const {return CSVIterator{};}
	vector<double> col(int n){
		reset();
		vector<double> vcol;
    			for(auto& row: *this){
			char * end;
			string_view s=row[n];
			float x = std::strtof(s.data(), &end);
			vcol.push_back(x);
    			} 
       return vcol; 
	}
	vector<double> multCol(int n,double d){
		reset();
		vector<double> vcol;
    			for(auto& row: *this){
			char * end;
			string_view s=row[n];
			float x = std::strtof(s.data(), &end)*d;
			vcol.push_back(x);
    			} 
       return vcol; 
	}
	void reset(){
		                stream.clear();
				stream.seekg(0);
	}
        vector<double> addDoubleToCol(int n,double d){
		reset();
		vector<double> vcol;
    			for(auto& row: *this){
			char * end;
			string_view s=row[n];
			float x = std::strtof(s.data(), &end)+d;
			vcol.push_back(x);
    			} 
       return vcol; 
}
};
/*int main() {
	std::ifstream       file ("src/itayAverage.csv");
        CSVRange mycsv=CSVRange(file);
	vector<double> tim=mycsv.col(0);
	vector<double> col2=mycsv.col(2);
	vector<double> col2add=mycsv.addDoubleToCol(2,-0.1);
	double res=Vrms(tim,col2,tim,col2add);

return 0;
}*/
