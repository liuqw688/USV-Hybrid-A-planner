#pragma once
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include "hybrid_a_star_planner/usv_dynamics.hpp"
namespace hybrid_a_star_planner::planning {
// Three independently sized steering phases include two-arc/straight cases.
// Commands are only seeds; accepted coordinates are integrated vessel states.
struct ShootingControl {std::array<double,3> duration{},turn{};double speed=1.2,radius=3,settle=2.4,turn_slew=1.0;};
struct ShootingTrajectory {ShootingControl control;std::vector<dynamics::State> states;double error=1e9;};
inline dynamics::State shootingRollout(const dynamics::State& start,const ShootingControl& p,
 const dynamics::Parameters& model,std::vector<dynamics::State>* states=nullptr) {
 auto s=start;double command=std::clamp(start.r/std::max(.01,std::min(model.max_yaw_rate,std::hypot(start.u,start.v)/p.radius)),-1.,1.);if(states){states->clear();states->push_back(s);}
 for(int phase=0;phase<4;++phase) {
  double duration=phase<3?p.duration[phase]:p.settle;
  const int n=std::max(1,int(std::ceil(duration/.2)));const double dt=duration/n;
  if(dt<=0)continue;
  for(int i=0;i<n;++i) {
   // Same resultant-speed/radius cap as the production motion primitives.
   const double target=phase<3?p.turn[phase]:0;
   command=std::clamp(target,command-p.turn_slew*dt,command+p.turn_slew*dt);
   const double rate=command*std::min(model.max_yaw_rate,std::hypot(s.u,s.v)/p.radius);
   s=dynamics::stepSetpoint(s,p.speed,rate,model,dt);if(states)states->push_back(s);
  }
 }
 return s;
}
inline std::array<double,3> shootingResidual(const dynamics::State& end,double x,double y,double yaw) {
 return {end.x-x,end.y-y,4*dynamics::normalizeAngle(end.yaw-yaw)};
}
inline double shootingNorm(const std::array<double,3>& e){return e[0]*e[0]+e[1]*e[1]+e[2]*e[2];}
inline bool solveShooting3(double a[3][3],const std::array<double,3>& b,std::array<double,3>& x) {
 double m[3][4];for(int i=0;i<3;++i){for(int k=0;k<3;++k)m[i][k]=a[i][k];m[i][3]=b[i];}
 for(int k=0;k<3;++k) {int pivot=k;for(int i=k+1;i<3;++i)if(std::abs(m[i][k])>std::abs(m[pivot][k]))pivot=i;
  if(std::abs(m[pivot][k])<1e-10) {return false;}
  for(int j=k;j<4;++j)std::swap(m[k][j],m[pivot][j]);
  double d=m[k][k];for(int j=k;j<4;++j)m[k][j]/=d;
  for(int i=0;i<3;++i)if(i!=k){d=m[i][k];for(int j=k;j<4;++j)m[i][j]-=d*m[k][j];}}
 for(int i=0;i<3;++i)x[i]=m[i][3];
 return true;
}
inline bool optimizeShooting(const dynamics::State& start,double gx,double gy,double yaw,
 ShootingControl& p,const dynamics::Parameters& model,const std::function<bool()>& expired) {
 double lambda=.02;
 for(int iteration=0;iteration<28 && !expired();++iteration) {
  auto e=shootingResidual(shootingRollout(start,p,model),gx,gy,yaw);double norm=shootingNorm(e);
  if(std::hypot(e[0],e[1])<.10 && std::abs(e[2])<.12)return true;
  double J[3][6]{};
  for(int col=0;col<6;++col) {
   if(expired())return false;
   auto trial=p;const double h=col<3?.08:.008;
   if(col<3)trial.duration[col]+=h;else trial.turn[col-3]+=h;
   auto other=shootingResidual(shootingRollout(start,trial,model),gx,gy,yaw);
   for(int row=0;row<3;++row)J[row][col]=(other[row]-e[row])/h;
  }
  bool improved=false;
  for(int attempt=0;attempt<5;++attempt) {
   double A[3][3]{};for(int i=0;i<3;++i)for(int j=0;j<3;++j){for(int c=0;c<6;++c)A[i][j]+=J[i][c]*J[j][c];if(i==j)A[i][j]+=lambda;}
   std::array<double,3> correction;if(!solveShooting3(A,e,correction))return false;
   auto trial=p;
   for(int c=0;c<6;++c){double step=0;for(int i=0;i<3;++i)step+=J[i][c]*correction[i];
    if(c<3)trial.duration[c]=std::clamp(p.duration[c]-std::clamp(step,-8.,8.),0.,100.);
    else trial.turn[c-3]=std::clamp(p.turn[c-3]-std::clamp(step,-.35,.35),-1.,1.);}
   if(shootingNorm(shootingResidual(shootingRollout(start,trial,model),gx,gy,yaw))<norm) {p=trial;lambda=std::max(1e-5,lambda*.4);improved=true;break;}
   lambda*=5;
  }
  if(!improved)return false;
 }
 return false;
}
template<class T> inline const T& shootingSegment(const T* type,int i){return type[i];}
template<class T> inline const T& shootingSegment(const std::vector<T>* type,int i){return (*type)[i];}
inline std::vector<ShootingControl> shootingSeeds(const dynamics::State& start,double gx,double gy,double yaw,double speed) {
 std::vector<ShootingControl> seeds;
 // G1 biarc seeds, with unequal tangent distances as well as the symmetric
 // case. Their coordinates are NEVER published: the dynamics solve below
 // adjusts the phase lengths/commands and validates the integrated result.
 for(double tail:{2.4,.4,0.0})for(double ratio:{1.,.5,2.}) {
  const double ax=start.x,ay=start.y,theta=start.yaw+std::atan2(start.v,std::max(.1,start.u));
  const double bx=gx-speed*tail*std::cos(yaw),by=gy-speed*tail*std::sin(yaw);
  const double vx=bx-ax,vy=by-ay,ux=std::cos(theta),uy=std::sin(theta),wx=std::cos(yaw),wy=std::sin(yaw);
  const double A=2*ratio*(1-ux*wx-uy*wy),B=2*(vx*(ux+ratio*wx)+vy*(uy+ratio*wy)),C=-vx*vx-vy*vy;
  const double disc=B*B-4*A*C;
  if(disc<0 || (std::abs(A)<1e-9 && B<=1e-9))continue;
  const double d1=std::abs(A)<1e-9?-C/B:(-B+std::sqrt(disc))/(2*A),d2=ratio*d1;
  if(d1<=1e-6 || d2<=1e-6)continue;
  const double mx=(d2*(ax+d1*ux)+d1*(bx-d2*wx))/(d1+d2),my=(d2*(ay+d1*uy)+d1*(by-d2*wy))/(d1+d2);
  const double x1=mx-ax,y1=my-ay,x2=mx-bx,y2=my-by;
  const double cross1=ux*y1-uy*x1,cross2=-wx*y2+wy*x2;
  if(std::abs(cross1)<1e-8 || std::abs(cross2)<1e-8)continue;
  const double r1=(x1*x1+y1*y1)/(2*cross1),r2=(x2*x2+y2*y2)/(2*cross2);
  const double alpha=2*std::atan2(cross1,ux*x1+uy*y1),beta=-2*std::atan2(cross2,-wx*x2-wy*y2);
  const double minimum=speed<=.91?1.5:3.;
  if(std::min(std::abs(r1),std::abs(r2))<minimum)continue;
  ShootingControl p;p.speed=speed;p.radius=std::min(std::abs(r1),std::abs(r2));p.settle=tail;
  p.duration={std::abs(alpha*r1)/speed,0,std::abs(beta*r2)/speed};p.turn={std::copysign(p.radius/std::abs(r1),alpha),0,std::copysign(p.radius/std::abs(r2),beta)};
  seeds.push_back(p);
 }

 for(double radius:(speed<=.91?std::vector<double>{1.5,2.,3.,5.,8.,12.}:std::vector<double>{3.,5.,8.,12.}))for(double tail:{2.4,4.0,.4,0.0}) {
  if(radius<3 && speed>0.91)continue;
  auto space=std::make_shared<ompl::base::DubinsStateSpace>(radius);
  ompl::base::ScopedState<ompl::base::SE2StateSpace> a(space),b(space);
  a->setXY(start.x,start.y);a->setYaw(start.yaw+std::atan2(start.v,std::max(.1,start.u)));
  b->setXY(gx-speed*tail*std::cos(yaw),gy-speed*tail*std::sin(yaw));b->setYaw(yaw);
  auto word=space->dubins(a.get(),b.get());ShootingControl p;p.speed=speed;p.radius=radius;p.settle=tail;
  for(int i=0;i<3;++i){p.duration[i]=word.length_[i]*radius/speed;p.turn[i]=shootingSegment(word.type_,i)==ompl::base::DubinsStateSpace::DUBINS_LEFT?1:shootingSegment(word.type_,i)==ompl::base::DubinsStateSpace::DUBINS_RIGHT?-1:0;}
  seeds.push_back(p);
 }
 return seeds;
}
}
