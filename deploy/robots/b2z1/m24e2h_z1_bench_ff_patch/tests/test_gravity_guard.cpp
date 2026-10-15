#include "m24e2h_gravity_guard.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
int main(){
  m24e2h::Vec6 g{1,2,3,4,5,6},extra{0.5,0,0,0,-1,0};
  auto out=m24e2h::combineGravityTorque(g,extra);
  assert(out[0]==1.5 && out[4]==4 && out[5]==6);
  bool bad=false;
  try{auto q=g;q[1]=std::numeric_limits<double>::quiet_NaN();m24e2h::combineGravityTorque(q,extra);}catch(...){bad=true;}
  assert(bad);
  bad=false;try{auto q=g;q[0]=51;m24e2h::combineGravityTorque(q,extra);}catch(...){bad=true;}assert(bad);
  bad=false;try{auto q=extra;q[0]=51;m24e2h::combineGravityTorque(g,q);}catch(...){bad=true;}assert(bad);
  std::cout<<"M2.4e2h numerical FF guard PASS (SDK OFF)\n";
}
