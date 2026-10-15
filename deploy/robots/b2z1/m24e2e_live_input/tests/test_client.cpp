#include "z1_ipc_client.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace m24e2e;
int main(int argc,char** argv){
 if(argc!=2)return 2;
 Session s;s.connectTo(argv[1],Mode::Mock);
 auto f=s.get(1);
 assert(f.mode==Mode::Mock&&std::isfinite(f.q[1]));
 assert(!f.tracking);
 s.target(1,f.q,true);
 auto g=s.get(2);
 assert(g.last_target_seq==1&&g.tracking);
 bool bad=false;try{s.target(1,f.q,true);}catch(const std::runtime_error&){bad=true;} assert(bad);
 // Worker latches after duplicate. Disconnect and new session needed for fresh state.
 s.close();
 Session s2;s2.connectTo(argv[1],Mode::Mock);
 g=s2.get(3); assert(g.last_target_seq==0);
 bad=false;try{s2.connectTo(argv[1],Mode::SdkUnqualified);}catch(const std::exception&){bad=true;} assert(bad);
 std::cout<<"M2.4e2e IPC client checks PASS\n";
}
