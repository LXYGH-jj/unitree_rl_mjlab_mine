#include "m24e2h_gravity_guard.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
using m24e2h::Vec6;
static bool rejected(const Vec6& g,const Vec6& c){
    try{(void)m24e2h::combineGravityTorque(g,c);return false;}
    catch(const std::runtime_error&){return true;}
}
int main(){
    static_assert(m24e2h::kModelTorqueSanityBoundNm==20.0,"torque guard must be exactly 20Nm");
    const Vec6 z{};
    auto g=z,c=z;
    g[0]=20.0; assert(!rejected(g,z));
    g[0]=-20.0;assert(!rejected(g,z));
    g[0]=20.00001;assert(rejected(g,z));
    g[0]=-20.00001;assert(rejected(g,z));
    g=z; c=z;
    g[1]=12.0;c[1]=8.0;assert(!rejected(g,c));
    c[1]=8.00001;assert(rejected(g,c));
    g=z;c=z;c[5]=-21.; assert(rejected(g,c));
    g=z;c=z;g[3]=std::numeric_limits<double>::quiet_NaN();assert(rejected(g,c));
    g=z;c=z;g[2]=std::numeric_limits<double>::infinity();assert(rejected(g,c));
    std::cout<<"M2.4e2i 20 Nm FF threshold boundary tests PASS (no motor IO)\n";
}
