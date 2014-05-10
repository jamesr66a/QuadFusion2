#include "../include/PID.h"
#include <iostream>
float constrain(float a, float x, float y);



int main(){
 float tp=7,cp=3;
 float output;
 inFlight=1;
 currentTime=2000000;
 PID[PITCH].P=1;
 PID[PITCH].I=1;
 PID[PITCH].D=1;
 PID[PITCH].lastError=1;
 PID[PITCH].previousPIDTime=1000000;
 PID[PITCH].integratedError=1;
 PID[PITCH].windupGuard=100;
 output= updatePID(tp,cp,&PID[PITCH]);
 std::cout<<output;
 std::cout<<"\n";
}
