#include "state_reconstruction.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace p112 {
namespace { constexpr int motor_index[6]={0,1,3,4,2,5}; constexpr int leg_index[4]={0,1,3,4};
template<std::size_t N> bool finite(const std::array<double,N>& a) noexcept {for(double v:a)if(!std::isfinite(v))return false;return true;}
}
StateReconstructor::StateReconstructor(void* c,GeometryCallback f,const ContactVelocityProjection& p):context_(c),callback_(f),projection_(p) {
    if(!callback_) throw std::invalid_argument("StateReconstructor requires CAD geometry callback");
    for(unsigned r=0;r<2;++r)for(double v:projection_.row[r])if(!std::isfinite(v))throw std::invalid_argument("Invalid contact velocity projection");
}
bool StateReconstructor::reconstruct(const CanonicalSensor& y,const Vec6& attitude,State15& out,CadGeometryPacket* packet) const noexcept {
    if(!finite(y)||!finite(attitude)||std::abs(std::cos(attitude[1]))<1e-7)return false;
    double motor[6]={y[6],y[8],y[14],y[10],y[12],y[16]};
    double velocity[6]={y[7],y[9],y[15],y[11],y[13],y[17]};
    CadGeometryPacket local{};if(callback_(context_,motor,&local)!=0||!finite(local.coordinates)||!finite(local.jacobian))return false;
    double roll=attitude[0],pitch=attitude[1];double pitch_rate=std::cos(roll)*attitude[4]-std::sin(roll)*attitude[5];double yaw_rate=(std::sin(roll)*attitude[4]+std::cos(roll)*attitude[5])/std::cos(pitch);
    State15 state{};state[0]=pitch;state[1]=attitude[2];for(unsigned i=0;i<4;++i)state[2+i]=motor[leg_index[i]];state[6]=motor[2];state[7]=motor[5];
    state[9]=pitch_rate;state[10]=yaw_rate;for(unsigned i=0;i<4;++i)state[11+i]=velocity[leg_index[i]];
    double gain0=projection_.row[0][0],gain1=projection_.row[1][0];double numerator=0.,denominator=gain0*gain0+gain1*gain1;
    if(denominator<1e-14)return false;
    double wheel[2]={velocity[2],velocity[5]};for(unsigned r=0;r<2;++r){double predicted=0.;for(unsigned j=0;j<7;++j)predicted+=projection_.row[r][j]*state[8+j];numerator+=(r==0?gain0:gain1)*(wheel[r]-predicted);}
    state[8]=numerator/denominator;if(!finite(state))return false;out=state;if(packet)*packet=local;return true;
}
}
