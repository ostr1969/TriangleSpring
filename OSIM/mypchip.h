#include<cmath>
using namespace std;
//#include <cstdlib>
#include <algorithm>    // std::fill

int sign1(double x)
{return (x < 0) ? -1 : (x > 0);}
double myabs(double x)
{return sign1(x)*x;
}


double pchipend(double h1,double h2,double del1,double del2)
// Noncentered, shape-preserving, three-point formula.
{
double d;
d = ((2*h1+h2)*del1 - h1*del2)/(h1+h2);
if (sign1(d) != sign1(del1))
    d = 0;
else
 if ((sign1(del1)!=sign1(del2)) && (myabs(d)>myabs(3*del1)))
    d = 3*del1;
return d;
}


vector<double> pchipslopes(vector<double> h,vector<double> delta)
{
/*% PCHIPSLOPES Slopes for shape-preserving Hermite cubic
% pchipslopes(h,delta) computes d(k) = P’(x(k)).
% Slopes at interior points
% delta = diff(y)./diff(x).
% d(k) = 0 if delta(k-1) and delta(k) have opposites
% signs or either is zero.
% d(k) = weighted harmonic mean of delta(k-1) and
% delta(k) if they have the same sign.*/

int n = h.size()+1;
vector<double> d (n);
fill_n(d.begin(),n,0.);
int k[h.size()];double w1,w2;
fill_n(d.begin(),5,0.);
int j=0;
for (int i=0; i<(n-2); i++)
  if (sign1(delta[i])*sign1(delta[i+1])>0)
     { k[j] =i+1 ;
	 j++;}

 //find(sign1(delta(1:n-2))*sign1(delta(2:n-1))>0)+1;
 for (int i=0;i<j;i++)
    {w1 = 2*h[k[i]]+h[k[i]-1];
    w2 = h[k[i]]+2*h[k[i]-1];
    d[k[i]] = (w1+w2)/(w1/delta[k[i]-1] + w2/delta[k[i]]);}
//% Slopes at endpoints
d[0]= pchipend(h[0],h[1],delta[0],delta[1]);
d[n-1] = pchipend(h[n-2],h[n-3],delta[n-2],delta[n-3]);
//cout<<"kk";
return d;
}



 vector<double> pchiptx(vector<double>x, vector<double> y, vector<double> u )
{
/*%PCHIPTX Textbook piecewise cubic Hermite interpolation.
% v = pchiptx(x,y,u) finds the shape-preserving piecewise
% P(x), with P(x(j)) = y(j), and returns v(k) = P(u(k)).
%
% See PCHIP, SPLINETX.
% First derivatives*/
int nu=u.size();
vector<double> v (nu);
int n = x.size();
vector<double> k(nu);vector<double> h(n-1);vector<double> delta(n-1);
vector<double> b(n-1);vector<double> c(n-1);vector<double> s(nu);vector<double> d(n);
for (int i=0;i<n-1;i++)
{h[i]=x[i+1]-x[i];
delta[i]=(y[i+1]-y[i])/h[i];}

d=pchipslopes(h,delta);
//% Piecewise polynomial coefficients

for (int i=0;i<n-1;i++)
{c[i] = (3*delta[i] - 2*d[i] - d[i+1])/h[i];
b[i] = (d[i] - 2*delta[i] + d[i+1])/pow(h[i],2);}

//% Find subinterval indices k so that x(k) <= u < x(k+1)

fill_n(k.begin(),nu,1);
for (int j=0;  j<nu;j++)
    for(int m=n-1;m>0;m--)
      if (u[j]<x[m])
         k[j] = m;

//% Evaluate interpolant
for (int i=0;i<nu;i++)
{s[i] = u[i] - x[k[i]-1];
v[i] = y[k[i]-1] + s[i]*(d[k[i]-1]+ s[i]*(c[k[i]-1] + s[i]*b[k[i]-1]));}
return v;
}
double pchiptx(const vector<double> x,const  vector<double> y,const  double u )
{
/*%PCHIPTX Textbook piecewise cubic Hermite interpolation.
% v = pchiptx(x,y,u) finds the shape-preserving piecewise
% P(x), with P(x(j)) = y(j), and returns v(k) = P(u(k)).
%
% See PCHIP, SPLINETX.
% First derivatives*/
int nu=1;
double v ;
int n = x.size();
int k=0;vector<double> h(n-1);vector<double> delta(n-1);
vector<double> b(n-1);vector<double> c(n-1);double s;vector<double> d(n);
for (int i=0;i<n-1;i++)
{h[i]=x[i+1]-x[i];
delta[i]=(y[i+1]-y[i])/h[i];}

d=pchipslopes(h,delta);
//% Piecewise polynomial coefficients

for (int i=0;i<n-1;i++)
{c[i] = (3*delta[i] - 2*d[i] - d[i+1])/h[i];
b[i] = (d[i] - 2*delta[i] + d[i+1])/pow(h[i],2);}

//% Find subinterval indices k so that x(k) <= u < x(k+1)

//fill_n(k.begin(),nu,1);
//for (int j=0;  j<nu;j++)
    for(int m=n-1;m>0;m--)
      if (u<=x[m])
         k = m;

//% Evaluate interpolant
//for (int i=0;i<nu;i++)
s = u - x[k-1];
v = y[k-1] + s*(d[k-1]+ s*(c[k-1] + s*b[k-1]));
return v;
}

double pchiptx(const vector<vector<double>> xy,const  double u )
{
/*%PCHIPTX Textbook piecewise cubic Hermite interpolation.
% v = pchiptx(x,y,u) finds the shape-preserving piecewise
% P(x), with P(x(j)) = y(j), and returns v(k) = P(u(k)).
%
% See PCHIP, SPLINETX.
% First derivatives*/
int nu=1;
double v ;
int n = xy.size();
int k=0;vector<double> h(n-1);vector<double> delta(n-1);
vector<double> b(n-1);vector<double> c(n-1);double s;vector<double> d(n);
for (int i=0;i<n-1;i++)
{h[i]=xy[i+1][0]-xy[i][0];
delta[i]=(xy[i+1][1]-xy[i][1])/h[i];}

d=pchipslopes(h,delta);
//% Piecewise polynomial coefficients

for (int i=0;i<n-1;i++)
{c[i] = (3*delta[i] - 2*d[i] - d[i+1])/h[i];
b[i] = (d[i] - 2*delta[i] + d[i+1])/pow(h[i],2);}

//% Find subinterval indices k so that x(k) <= u < x(k+1)

//fill_n(k.begin(),nu,1);
//for (int j=0;  j<nu;j++)
    for(int m=n-1;m>0;m--)
      if (u<=xy[m][0])
         k = m;

//% Evaluate interpolant
//for (int i=0;i<nu;i++)
s = u - xy[k-1][0];
v = xy[k-1][1] + s*(d[k-1]+ s*(c[k-1] + s*b[k-1]));
return v;
}
 double limitedPchip(const vector<vector<double>> xy,const  double u){
   double highAng=xy[xy.size()-1][0],lowAng=xy[0][0];
   double dAng=(highAng-lowAng)/20;
   if(u<(lowAng+dAng)){
   double t1=lowAng-dAng,t2=lowAng+dAng;
   double y2=pchiptx(xy,t2),y1=xy[0][1];
     double k=std::max(0.,std::min(1.,(u-t1)/(t2-t1)));
     double s=k*k*(3-2*k)*(y2-y1)+y1;
	return s;}  
   else if(u>(highAng-dAng)){
   double t1=highAng-dAng,t2=highAng+dAng;
   double y1=pchiptx(xy,t1),y2=xy[xy.size()-1][1];
     double k=std::max(0.,std::min(1.,(u-t1)/(t2-t1)));
     double s=k*k*(3-2*k)*(y2-y1)+y1;
	return s;} 
   else return pchiptx(xy,u); 
   //if (u>xy[xy.size()-1][0]) return xy[xy.size()-1][1];
   //if (u<xy[0][0]) return xy[0][1];
}
