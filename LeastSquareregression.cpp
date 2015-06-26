//    This file is part of 3DFOREST  www.3dforest.eu
//
//    3DFOREST is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    3DFOREST is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with 3DFOREST.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////

#include "LeastSquareregression.h"
LeastSquaredRegression::LeastSquaredRegression ()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_cloud = cloud;
}
LeastSquaredRegression::LeastSquaredRegression (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_cloud = cloud;
}
LeastSquaredRegression::~LeastSquaredRegression ()
{
  m_cloud.reset();
}
void LeastSquaredRegression::setCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_cloud = cloud;
}
void  LeastSquaredRegression::compute()
{
    algebraicCircle();
    geometricCirlce();
}
stred LeastSquaredRegression::getCircle()
{
  return m_circle;
}

void LeastSquaredRegression::algebraicCircle()
{
  float meanX = 0;
  float meanY = 0;
  float meanZ = 0;
  float n = (float) m_cloud->points.size();
  float Mz,Mxy,Mxx,Myy,Mxz,Myz,Mzz,Cov_xy,Var_z;
  float A0,A1,A2,A22;
  float Dy,xnew,x,ynew,y;
  float DET,Xcenter,Ycenter;
  Mxx=Myy=Mxy=Mxz=Myz=0.;
    //for each point in dbh_cloud calculate mean coordinate
  for(int m=0; m < n;m++)
  {
    pcl::PointXYZI ith = m_cloud->points.at(m);
    meanX += ith.x;
    meanY += ith.y;
    meanZ += ith.z;
  }
  meanX /= n;
  meanY /= n;
  meanZ /= n;

  for(int j=0; j < n; j++)
  {
    pcl::PointXYZI ith = m_cloud->points.at(j);
    float Xi = ith.x - meanX;   //  centered x-coordinates
    float Yi = ith.y - meanY;   //  centered y-coordinates
    float Zi = Xi*Xi + Yi*Yi;
    Mxx += Xi*Xi;
    Myy += Yi*Yi;
    Mxy += Xi*Yi;
    Mxz += Xi*Zi;
    Myz += Yi*Zi;
  }
  Mxx /= n;
  Myy /= n;
  Mxy /= n;
  Mxz /= n;
  Myz /= n;
//    computing the coefficients of the characteristic polynomial
  Mz = Mxx + Myy;
  Cov_xy = Mxx*Myy - Mxy*Mxy;
  Var_z = Mzz - Mz*Mz;
  A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
  A1 = Var_z*Mz + 4*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
  A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
  A22 = A2 + A2;
//    finding the root of the characteristic polynomial
//    using Newton's method starting at x=0
//     (it is guaranteed to converge to the right root)
  x=0;
  y=A0;
	for (int iter=0; iter<10; iter++)  // usually, 4-6 iterations are enough
  {
    Dy = A1 + x*(A22 + 16.*x*x);
    xnew = x - y/Dy;
    if ((xnew == x)||(!std::isfinite(xnew)))
      break;
    ynew = A0 + xnew*(A1 + xnew*(A2 + 4*xnew*xnew));
    if (abs(ynew)>=abs(y))
      break;
    x = xnew;
    y = ynew;
  }
//    computing paramters of the fitting circle
  DET = x*x - x*Mz + Cov_xy;
  Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/(DET*2);
  Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/(DET*2);
  float xx = Xcenter + meanX; //xcoord of center
  float yy = Ycenter + meanY; //ycoord of center
  float Mr=0.; //optimal r computation
  float dx,dy;

  for(int k=0; k < n; k++)
  {
    pcl::PointXYZI ith = m_cloud->points.at(k);

    dx = ith.x - xx;
    dy = ith.y - yy;
    Mr += sqrt(dx*dx + dy*dy);
  }
  //float r = Mr/n;
  float r= ((Xcenter*Xcenter) + (Ycenter*Ycenter) + Mz - x - x)*1000;
  float rr = ceil(r)/5.0;
  float ii=0;
  stred T={xx,yy,meanZ,ii,rr};
  m_circle = T;
}
void LeastSquaredRegression::geometricCirlce()
{
  int code,i,iter,inner,IterMAX=99;
  float n = (float) m_cloud->points.size();
  float factorUp=10.,factorDown=0.04,lambda,ParLimit=1.e+6;
  float dx,dy,ri,u,v;
  float Mu,Mv,Muu,Mvv,Muv,Mr,UUl,VVl,Nl,F1,F2,F3,dX,dY,dR;
  float epsilon=3.e-8;
  float G11,G22,G33,G12,G13,G23,D1,D2,D3;
  float meanX=0;
  float meanY=0;
  float meanZ=0;
  //float summ=0.,ddxx,ddyy;
  stredLSR Old,New;
   for(int m=0; m < n;m++)
  {
    pcl::PointXYZI ith = m_cloud->points.at(m);
    meanX += ith.x;
    meanY += ith.y;
    meanZ += ith.z;
  }
  meanX /= n;
  meanY /= n;
  meanZ /= n;
//       starting with the given initial circle (initial guess)
  New = {m_circle.a,m_circle.b,m_circle.r,0,0,0,0};
//       compute the root-mean-square error
  New.s = sigma(New);

//       initializing lambda, iteration counters, and the exit code

    lambda = 0.0001;
    iter = inner = code = 0;

NextIteration:

    Old = New;
    if (++iter > IterMAX)
    {code = 1;  goto enough;}

//       computing moments

    Mu=Mv=Muu=Mvv=Muv=Mr=0.;

    for (i=0; i<n; i++)
    {
      pcl::PointXYZI ith = m_cloud->points.at(i);

        dx = ith.x - Old.a;
        dy = ith.y - Old.b;
        ri = sqrt(dx*dx + dy*dy);
        u = dx/ri;
        v = dy/ri;
        Mu += u;
        Mv += v;
        Muu += u*u;
        Mvv += v*v;
        Muv += u*v;
        Mr += ri;
    }
    Mu /= n;
    Mv /= n;
    Muu /=n;
    Mvv /=n;
    Muv /=n;
    Mr /= n;

//       computing matrices

    F1 = Old.a + Old.r*Mu - meanX;
    F2 = Old.b + Old.r*Mv - meanY;
    F3 = Old.r - Mr;

    Old.g = New.g = sqrt(F1*F1 + F2*F2 + F3*F3);

try_again:

    UUl = Muu + lambda;
    VVl = Mvv + lambda;
    Nl = 1 + lambda;

//         Cholesly decomposition

    G11 = sqrt(UUl);
    G12 = Muv/G11;
    G13 = Mu/G11;
    G22 = sqrt(VVl - G12*G12);
    G23 = (Mv - G12*G13)/G22;
    G33 = sqrt(Nl - G13*G13 - G23*G23);

    D1 = F1/G11;
    D2 = (F2 - G12*D1)/G22;
    D3 = (F3 - G13*D1 - G23*D2)/G33;

    dR = D3/G33;
    dY = (D2 - G23*dR)/G22;
    dX = (D1 - G12*dY - G13*dR)/G11;

    if ((abs(dR)+abs(dX)+abs(dY))/(1+Old.r) < epsilon) goto enough;

//       updating the parameters

    New.a = Old.a - dX;
    New.b = Old.b - dY;

    if (abs(New.a)>ParLimit || abs(New.b)>ParLimit) {code = 3; goto enough;}

    New.r = Old.r - dR;

    if (New.r <= 0.)
    {
        lambda *= factorUp;
        if (++inner > IterMAX) {code = 2;  goto enough;}
        goto try_again;
    }

//       compute the root-mean-square error

    New.s = sigma(New);

//       check if improvement is gained

    if (New.s < Old.s)    //   yes, improvement
    {
        lambda *= factorDown;
        goto NextIteration;
    }
    else                       //   no improvement
    {
      if (++inner > IterMAX)
        {code = 2;  goto enough;}
      lambda *= factorUp;
      goto try_again;
    }
    //       exit
enough:

    Old.i = iter;    // total number of outer iterations (updating the parameters)
    Old.j = inner;   // total number of inner iterations (adjusting lambda)
    stredLSR circlef = Old;
    float r= circlef.r*1000;
    float rr = ceil(r)/10.0;
    stred c = {circlef.a,circlef.b,meanZ,1,rr};
    m_circle = c;
}
float LeastSquaredRegression::sigma (stredLSR circle)
{
  float sum=0.,dx,dy;
  float n = (float) m_cloud->points.size();
  for (int i=0; i<n; i++)
  {
    pcl::PointXYZI ith = m_cloud->points.at(i);
    dx = ith.x - circle.a;
    dy = ith.y - circle.b;
    sum += ((sqrt(dx*dx+dy*dy) - circle.r)*(sqrt(dx*dx+dy*dy) - circle.r));
  }
  return sqrt(sum/n);
}
