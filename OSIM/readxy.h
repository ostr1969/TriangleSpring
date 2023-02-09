#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
//#include "mypchip.h"
using namespace std;
// g++ readfile.cpp -lstdc++
// vector<vector<double>> delp1;
// vector<vector<double>> delp2;
// vector<vector<double>> delp3;
// vector<vector<double>> delp4;
// vector<vector<double>> delp5;
// vector<vector<double>> delp6;

int readxy(string filename, vector<vector<double>> &scores)
{
   std::ifstream ifile(filename, std::ios::in);

    //check to see that the file was opened correctly:
        if (!ifile.is_open()) {
         std::cerr << "There was a problem opening the input file!\n";
         exit(1);//exit or do additional error checking
                   }
       double num1,num2;int i=0;
       vector<double> v(2);
    //keep storing values from the text file so long as data exists:
       while (ifile >> v[0]>>v[1]) {
        scores.push_back(v);
            }
     ifile.close();
    //verify that the scores were stored correctly:
  //   for (vector<double> v:scores) { 
  //          cout << v[0] << " "<<v[1]<<endl; 
 //   } 
return 0;
} 

/*int main ()
{  
 vector<vector<double>> scores;
readxy( "src/delp1.txt",scores);
 cout<<pchiptx(scores,100)<<endl;
for (int i=0;i<scores.size();i++)
cout<< scores[i][0] << ":"<<scores[i][1]<< endl;
return 0;
}*/
