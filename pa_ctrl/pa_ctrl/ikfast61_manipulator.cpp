/// autogenerated analytical inverse kinematics code from ikfast program part of OpenRAVE
/// \author Rosen Diankov
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///     http://www.apache.org/licenses/LICENSE-2.0
/// 
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// ikfast version 0x1000004b generated on 2024-02-14 13:07:57.599387
/// Generated using solver transform6d
/// To compile with gcc:
///     gcc -lstdc++ ik.cpp
/// To compile without any main function as a shared object (might need -llapack):
///     gcc -fPIC -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wl,-soname,libik.so -o libik.so ik.cpp
#define IKFAST_HAS_LIBRARY
#include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==0x1000004b);

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI  ((IkReal)6.28318530717959)
#define IKPI  ((IkReal)3.14159265358979)
#define IKPI_2  ((IkReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#ifndef isinf
#define isinf _isinf
#endif
//#ifndef isfinite
//#define isfinite _isfinite
//#endif
#endif // _MSC_VER

// lapack routines
extern "C" {
  void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
  void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
  void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
  void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
  void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
  void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((IkReal)0.03) // 5D IK has some crazy degenerate cases, but can rely on jacobian refinment to make better, just need good starting point
#endif


inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2Simple(float fy, float fx) {
    return atan2f(fy,fx);
}
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2Simple(double fy, double fx) {
    return atan2(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

template <typename T>
struct CheckValue
{
    T value;
    bool valid;
};

template <typename T>
inline CheckValue<T> IKatan2WithCheck(T fy, T fx, T epsilon)
{
    CheckValue<T> ret;
    ret.valid = false;
    ret.value = 0;
    if( !isnan(fy) && !isnan(fx) ) {
        if( IKabs(fy) >= IKFAST_ATAN2_MAGTHRESH || IKabs(fx) > IKFAST_ATAN2_MAGTHRESH ) {
            ret.value = IKatan2Simple(fy,fx);
            ret.valid = true;
        }
    }
    return ret;
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

template <typename T>
inline CheckValue<T> IKPowWithIntegerCheck(T f, int n)
{
    CheckValue<T> ret;
    ret.valid = true;
    if( n == 0 ) {
        ret.value = 1.0;
        return ret;
    }
    else if( n == 1 )
    {
        ret.value = f;
        return ret;
    }
    else if( n < 0 )
    {
        if( f == 0 )
        {
            ret.valid = false;
            ret.value = (T)1.0e30;
            return ret;
        }
        if( n == -1 ) {
            ret.value = T(1.0)/f;
            return ret;
        }
    }

    int num = n > 0 ? n : -n;
    if( num == 2 ) {
        ret.value = f*f;
    }
    else if( num == 3 ) {
        ret.value = f*f*f;
    }
    else {
        ret.value = 1.0;
        while(num>0) {
            if( num & 1 ) {
                ret.value *= f;
            }
            num >>= 1;
            f *= f;
        }
    }
    
    if( n < 0 ) {
        ret.value = T(1.0)/ret.value;
    }
    return ret;
}

template <typename T> struct ComplexLess
{
    bool operator()(const complex<T>& lhs, const complex<T>& rhs) const
    {
        if (real(lhs) < real(rhs)) {
            return true;
        }
        if (real(lhs) > real(rhs)) {
            return false;
        }
        return imag(lhs) < imag(rhs);
    }
};

/// solves the forward kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot) {
IkReal x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36,x37,x38,x39,x40,x41,x42,x43,x44,x45;
x0=IKsin(j[2]);
x1=IKsin(j[3]);
x2=IKcos(j[2]);
x3=IKcos(j[3]);
x4=IKcos(j[5]);
x5=IKcos(j[6]);
x6=IKsin(j[4]);
x7=IKcos(j[4]);
x8=IKsin(j[6]);
x9=IKcos(j[1]);
x10=IKsin(j[1]);
x11=IKsin(j[5]);
x12=((1.0)*x7);
x13=((0.02086)*x4);
x14=((0.02086)*x11);
x15=((1.0)*x4);
x16=(x2*x3);
x17=(x1*x2);
x18=(x4*x7);
x19=(x6*x8);
x20=(x0*x3);
x21=(x0*x1);
x22=(x11*x6);
x23=(x5*x6);
x24=(x11*x7);
x25=((1.0)*x17);
x26=(x12*x5);
x27=((0.3017)*x17);
x28=((0.3017)*x20);
x29=((((-1.0)*x25))+x20);
x30=((((-1.0)*x20))+x25);
x31=((((-1.0)*x21))+(((-1.0)*x16)));
x32=((((0.02086)*x5*x7))+(((0.3017)*x22)));
x33=((((0.15)*x7))+((x13*x19)));
x34=((((1.0)*x16))+(((1.0)*x21)));
x35=((((0.3017)*x21))+(((0.3017)*x16)));
x36=(x29*x7);
x37=((0.02086)*x31);
x38=(x11*x31);
x39=(x18*x29);
x40=(((x4*((x16+x21))))+((x24*x29)));
x41=(x39+x38);
x42=((((-1.0)*x12*x31*x4))+(((-1.0)*x11*x30)));
x43=(((x41*x8))+((x23*x29)));
x44=(((x19*x30))+((x41*x5)));
x45=(((x8*(((((-1.0)*x14*x31))+(((-1.0)*x13*x36))))))+(((0.54)*x21))+(((0.54)*x16))+((x24*(((((-1.0)*x27))+x28))))+((x35*x4))+((x6*(((((-0.15)*x17))+(((0.15)*x20))))))+(((0.71)*x0))+((x23*(((((-0.02086)*x20))+(((0.02086)*x17)))))));
eerot[0]=(((x10*(((((-1.0)*x15*x23))+(((-1.0)*x12*x8))))))+((x44*x9)));
eerot[1]=(((x43*x9))+((x10*(((((-1.0)*x15*x19))+x26)))));
eerot[2]=((((-1.0)*x10*x22))+((x40*x9)));
eetrans[0]=(((x10*(((((-1.0)*x32))+x33))))+((x45*x9)));
eerot[3]=(((x9*((((x7*x8))+((x23*x4))))))+((x10*x44)));
eerot[4]=(((x9*((((x19*x4))+(((-1.0)*x26))))))+((x10*x43)));
eerot[5]=(((x22*x9))+((x10*x40)));
eetrans[1]=(((x10*x45))+((x9*(((((-1.0)*x33))+x32)))));
eerot[6]=(((x19*x31))+((x42*x5)));
eerot[7]=(((x23*x34))+((x42*x8)));
eerot[8]=(((x30*x4))+((x24*x34)));
eetrans[2]=((1.0434)+(((0.54)*x17))+(((-0.54)*x20))+((x23*(((((-0.02086)*x21))+(((-0.02086)*x16))))))+((x6*(((((0.15)*x21))+(((0.15)*x16))))))+((x8*((((x14*x30))+((x13*x31*x7))))))+((x24*x35))+((x4*(((((-1.0)*x28))+x27))))+(((0.71)*x2))+j[0]);
}

IKFAST_API int GetNumFreeParameters() { return 3; }
IKFAST_API const int* GetFreeIndices() { static const int freeindices[] = {0, 1, 5}; return freeindices; }
IKFAST_API int GetNumJoints() { return 7; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

IKFAST_API int GetIkType() { return 0x67000001; }

class IKSolver {
public:
IkReal j2,cj2,sj2,htj2,j2mul,j3,cj3,sj3,htj3,j3mul,j4,cj4,sj4,htj4,j4mul,j6,cj6,sj6,htj6,j6mul,j0,cj0,sj0,htj0,j1,cj1,sj1,htj1,j5,cj5,sj5,htj5,new_r00,r00,rxp0_0,new_r01,r01,rxp0_1,new_r02,r02,rxp0_2,new_r10,r10,rxp1_0,new_r11,r11,rxp1_1,new_r12,r12,rxp1_2,new_r20,r20,rxp2_0,new_r21,r21,rxp2_1,new_r22,r22,rxp2_2,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij2[2], _nj2,_ij3[2], _nj3,_ij4[2], _nj4,_ij6[2], _nj6,_ij0[2], _nj0,_ij1[2], _nj1,_ij5[2], _nj5;

IkReal j100, cj100, sj100;
unsigned char _ij100[2], _nj100;
bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
j2=numeric_limits<IkReal>::quiet_NaN(); _ij2[0] = -1; _ij2[1] = -1; _nj2 = -1; j3=numeric_limits<IkReal>::quiet_NaN(); _ij3[0] = -1; _ij3[1] = -1; _nj3 = -1; j4=numeric_limits<IkReal>::quiet_NaN(); _ij4[0] = -1; _ij4[1] = -1; _nj4 = -1; j6=numeric_limits<IkReal>::quiet_NaN(); _ij6[0] = -1; _ij6[1] = -1; _nj6 = -1;  _ij0[0] = -1; _ij0[1] = -1; _nj0 = 0;  _ij1[0] = -1; _ij1[1] = -1; _nj1 = 0;  _ij5[0] = -1; _ij5[1] = -1; _nj5 = 0; 
for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {
    solutions.Clear();
j0=pfree[0]; cj0=cos(pfree[0]); sj0=sin(pfree[0]), htj0=tan(pfree[0]*0.5);
j1=pfree[1]; cj1=cos(pfree[1]); sj1=sin(pfree[1]), htj1=tan(pfree[1]*0.5);
j5=pfree[2]; cj5=cos(pfree[2]); sj5=sin(pfree[2]), htj5=tan(pfree[2]*0.5);
r00 = eerot[0*3+0];
r01 = eerot[0*3+1];
r02 = eerot[0*3+2];
r10 = eerot[1*3+0];
r11 = eerot[1*3+1];
r12 = eerot[1*3+2];
r20 = eerot[2*3+0];
r21 = eerot[2*3+1];
r22 = eerot[2*3+2];
px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];

new_r00=(((r00*(IKcos(j1))))+((r10*(IKsin(j1)))));
new_r01=((((-1.0)*r11*(IKsin(j1))))+(((-1.0)*r01*(IKcos(j1)))));
new_r02=((((-1.0)*r02*(IKcos(j1))))+(((-1.0)*r12*(IKsin(j1)))));
IkReal x46=IKsin(j1);
IkReal x47=IKcos(j1);
new_px=(((py*x46))+((px*x47))+(((0.02086)*r01*x47))+(((-0.3017)*r02*x47))+(((-0.3017)*r12*x46))+(((0.02086)*r11*x46)));
new_r10=((-1.0)*r20);
new_r11=r21;
new_r12=r22;
new_py=((1.0434)+(((-0.02086)*r21))+(((-1.0)*pz))+(((0.3017)*r22))+j0);
new_r20=(((r10*(IKcos(j1))))+(((-1.0)*r00*(IKsin(j1)))));
new_r21=((((-1.0)*r11*(IKcos(j1))))+((r01*(IKsin(j1)))));
new_r22=((((-1.0)*r12*(IKcos(j1))))+((r02*(IKsin(j1)))));
IkReal x48=IKcos(j1);
IkReal x49=IKsin(j1);
new_pz=((((-1.0)*px*x49))+((py*x48))+(((0.3017)*r02*x49))+(((-0.02086)*r01*x49))+(((-0.3017)*r12*x48))+(((0.02086)*r11*x48)));
r00 = new_r00; r01 = new_r01; r02 = new_r02; r10 = new_r10; r11 = new_r11; r12 = new_r12; r20 = new_r20; r21 = new_r21; r22 = new_r22; px = new_px; py = new_py; pz = new_pz;
IkReal x50=((1.0)*px);
IkReal x51=((1.0)*pz);
IkReal x52=((1.0)*py);
pp=((px*px)+(py*py)+(pz*pz));
npx=(((px*r00))+((py*r10))+((pz*r20)));
npy=(((px*r01))+((py*r11))+((pz*r21)));
npz=(((px*r02))+((py*r12))+((pz*r22)));
rxp0_0=((((-1.0)*r20*x52))+((pz*r10)));
rxp0_1=(((px*r20))+(((-1.0)*r00*x51)));
rxp0_2=((((-1.0)*r10*x50))+((py*r00)));
rxp1_0=((((-1.0)*r21*x52))+((pz*r11)));
rxp1_1=(((px*r21))+(((-1.0)*r01*x51)));
rxp1_2=((((-1.0)*r11*x50))+((py*r01)));
rxp2_0=(((pz*r12))+(((-1.0)*r22*x52)));
rxp2_1=(((px*r22))+(((-1.0)*r02*x51)));
rxp2_2=((((-1.0)*r12*x50))+((py*r02)));
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
cj4array[0]=((-6.66666666666667)*pz);
if( cj4array[0] >= -1-IKFAST_SINCOS_THRESH && cj4array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j4valid[0] = j4valid[1] = true;
    j4array[0] = IKacos(cj4array[0]);
    sj4array[0] = IKsin(j4array[0]);
    cj4array[1] = cj4array[0];
    j4array[1] = -j4array[0];
    sj4array[1] = -sj4array[0];
}
else if( isnan(cj4array[0]) )
{
    // probably any value will work
    j4valid[0] = true;
    cj4array[0] = 1; sj4array[0] = 0; j4array[0] = 0;
}
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j2eval[2];
j2eval[0]=(pp+(((-1.0)*(pz*pz))));
j2eval[1]=((px*px)+(py*py));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j2, j3]

} else
{
{
IkReal j2array[2], cj2array[2], sj2array[2];
bool j2valid[2]={false};
_nj2 = 2;
CheckValue<IkReal> x55 = IKatan2WithCheck(IkReal(((-1.42)*py)),IkReal(((1.42)*px)),IKFAST_ATAN2_MAGTHRESH);
if(!x55.valid){
continue;
}
IkReal x53=((1.0)*(x55.value));
if((((((2.0164)*(py*py)))+(((2.0164)*(px*px))))) < -0.00001)
continue;
CheckValue<IkReal> x56=IKPowWithIntegerCheck(IKabs(IKsqrt(((((2.0164)*(py*py)))+(((2.0164)*(px*px)))))),-1);
if(!x56.valid){
continue;
}
if( (((-1.0)*(x56.value)*(((-0.19)+(((-1.0)*pp)))))) < -1-IKFAST_SINCOS_THRESH || (((-1.0)*(x56.value)*(((-0.19)+(((-1.0)*pp)))))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x54=((-1.0)*(IKasin(((-1.0)*(x56.value)*(((-0.19)+(((-1.0)*pp))))))));
j2array[0]=((((-1.0)*x54))+(((-1.0)*x53)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
j2array[1]=((3.14159265358979)+(((1.0)*x54))+(((-1.0)*x53)));
sj2array[1]=IKsin(j2array[1]);
cj2array[1]=IKcos(j2array[1]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
if( j2array[1] > IKPI )
{
    j2array[1]-=IK2PI;
}
else if( j2array[1] < -IKPI )
{    j2array[1]+=IK2PI;
}
j2valid[1] = true;
for(int ij2 = 0; ij2 < 2; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 2; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];

{
IkReal j3eval[3];
IkReal x57=cj4*cj4;
IkReal x58=((500.0)*sj4);
IkReal x59=(cj2*py);
IkReal x60=(px*sj2);
IkReal x61=(py*sj2);
IkReal x62=(cj2*px);
j3eval[0]=((13.96)+(((-1.0)*x57)));
j3eval[1]=IKsign(((1047.0)+(((-75.0)*x57))));
j3eval[2]=((IKabs(((((1800.0)*x61))+(((1800.0)*x62))+((x58*x60))+(((-1.0)*x58*x59))+(((-355.0)*sj4)))))+(IKabs(((-1278.0)+(((1800.0)*x60))+(((-1.0)*x58*x62))+(((-1.0)*x58*x61))+(((-1800.0)*x59))))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  || IKabs(j3eval[2]) < 0.0000010000000000  )
{
{
IkReal j3eval[1];
j3eval[0]=((-2.556)+(((-3.6)*cj2*py))+(((3.6)*px*sj2))+((py*sj2*sj4))+((cj2*px*sj4)));
if( IKabs(j3eval[0]) < 0.0000010000000000  )
{
{
IkReal j3eval[1];
IkReal x63=((1.40845070422535)*sj4);
j3eval[0]=(sj4+(((-1.0)*px*sj2*x63))+(((5.07042253521127)*cj2*px))+((cj2*py*x63))+(((5.07042253521127)*py*sj2)));
if( IKabs(j3eval[0]) < 0.0000010000000000  )
{
continue; // no branches [j3]

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
IkReal x64=cj2*cj2;
IkReal x65=py*py;
IkReal x66=px*px;
IkReal x67=(cj2*px);
IkReal x68=(py*sj2);
IkReal x69=(px*py);
IkReal x70=(cj2*py*sj4);
IkReal x71=((1000.0)*cj2*sj2);
IkReal x72=((400.0)*x64);
IkReal x73=(px*sj2*sj4);
CheckValue<IkReal> x74=IKPowWithIntegerCheck(((((106.5)*sj4))+(((-150.0)*x73))+(((150.0)*x70))+(((540.0)*x68))+(((540.0)*x67))),-1);
if(!x74.valid){
continue;
}
CheckValue<IkReal> x75=IKPowWithIntegerCheck(((((-216.0)*x67))+(((-216.0)*x68))+(((-60.0)*x70))+(((60.0)*x73))+(((-42.6)*sj4))),-1);
if(!x75.valid){
continue;
}
if( IKabs(((x74.value)*(((((1000.0)*x69))+(((-81.0)*sj4))+(((-710.0)*x68))+(((-710.0)*x67))+((x66*x71))+(((-2000.0)*x64*x69))+(((-1.0)*x65*x71)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((x75.value)*(((9.0)+(((-1.0)*x66*x72))+(((-800.0)*x67*x68))+(((-400.0)*x65))+(((-9.0)*(cj4*cj4)))+((x65*x72)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x74.value)*(((((1000.0)*x69))+(((-81.0)*sj4))+(((-710.0)*x68))+(((-710.0)*x67))+((x66*x71))+(((-2000.0)*x64*x69))+(((-1.0)*x65*x71))))))+IKsqr(((x75.value)*(((9.0)+(((-1.0)*x66*x72))+(((-800.0)*x67*x68))+(((-400.0)*x65))+(((-9.0)*(cj4*cj4)))+((x65*x72))))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j3array[0]=IKatan2(((x74.value)*(((((1000.0)*x69))+(((-81.0)*sj4))+(((-710.0)*x68))+(((-710.0)*x67))+((x66*x71))+(((-2000.0)*x64*x69))+(((-1.0)*x65*x71))))), ((x75.value)*(((9.0)+(((-1.0)*x66*x72))+(((-800.0)*x67*x68))+(((-400.0)*x65))+(((-9.0)*(cj4*cj4)))+((x65*x72))))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[4];
IkReal x76=IKsin(j3);
IkReal x77=IKcos(j3);
IkReal x78=(cj2*px);
IkReal x79=((0.15)*sj4);
IkReal x80=(cj2*py);
IkReal x81=(py*sj2);
IkReal x82=((1.0)*px*sj2);
IkReal x83=(sj2*x76);
evalcond[0]=((((0.54)*x77))+(((-1.0)*x81))+(((-1.0)*x78))+(((-1.0)*x76*x79)));
evalcond[1]=((0.71)+(((0.54)*x76))+x80+(((-1.0)*x82))+((x77*x79)));
evalcond[2]=((-0.54)+((x77*x81))+(((-0.71)*x76))+(((-1.0)*x76*x80))+((px*x83))+((x77*x78)));
evalcond[3]=(((x77*x80))+(((0.71)*x77))+(((-1.0)*x77*x82))+((x76*x81))+x79+((x76*x78)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
IkReal x104=py*py;
IkReal x105=cj2*cj2;
IkReal x106=px*px;
IkReal x107=(px*sj2);
IkReal x108=(py*sj2);
IkReal x109=((375.0)*sj4);
IkReal x110=(cj2*px);
IkReal x111=((150.0)*sj4);
IkReal x112=(cj2*py);
IkReal x113=(px*py);
IkReal x114=((1000.0)*cj2*sj2);
IkReal x115=((2500.0)*x105);
CheckValue<IkReal> x116=IKPowWithIntegerCheck(((-958.5)+(((-1350.0)*x112))+(((1350.0)*x107))+((x109*x110))+((x108*x109))),-1);
if(!x116.valid){
continue;
}
CheckValue<IkReal> x117=IKPowWithIntegerCheck(((-383.4)+((x110*x111))+(((-540.0)*x112))+((x108*x111))+(((540.0)*x107))),-1);
if(!x117.valid){
continue;
}
if( IKabs(((x116.value)*(((729.0)+(((-5000.0)*x107*x112))+(((-1.0)*x106*x115))+((x104*x115))+(((-2500.0)*x104)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((x117.value)*((((x106*x114))+(((1000.0)*x113))+(((-2000.0)*x105*x113))+(((-710.0)*x110))+(((-710.0)*x108))+(((81.0)*sj4))+(((-1.0)*x104*x114)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x116.value)*(((729.0)+(((-5000.0)*x107*x112))+(((-1.0)*x106*x115))+((x104*x115))+(((-2500.0)*x104))))))+IKsqr(((x117.value)*((((x106*x114))+(((1000.0)*x113))+(((-2000.0)*x105*x113))+(((-710.0)*x110))+(((-710.0)*x108))+(((81.0)*sj4))+(((-1.0)*x104*x114))))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j3array[0]=IKatan2(((x116.value)*(((729.0)+(((-5000.0)*x107*x112))+(((-1.0)*x106*x115))+((x104*x115))+(((-2500.0)*x104))))), ((x117.value)*((((x106*x114))+(((1000.0)*x113))+(((-2000.0)*x105*x113))+(((-710.0)*x110))+(((-710.0)*x108))+(((81.0)*sj4))+(((-1.0)*x104*x114))))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[4];
IkReal x118=IKsin(j3);
IkReal x119=IKcos(j3);
IkReal x120=(cj2*px);
IkReal x121=((0.15)*sj4);
IkReal x122=(cj2*py);
IkReal x123=(py*sj2);
IkReal x124=((1.0)*px*sj2);
IkReal x125=(sj2*x118);
evalcond[0]=((((-1.0)*x118*x121))+(((-1.0)*x123))+(((-1.0)*x120))+(((0.54)*x119)));
evalcond[1]=((0.71)+(((-1.0)*x124))+x122+((x119*x121))+(((0.54)*x118)));
evalcond[2]=((-0.54)+(((-1.0)*x118*x122))+((px*x125))+(((-0.71)*x118))+((x119*x120))+((x119*x123)));
evalcond[3]=((((-1.0)*x119*x124))+(((0.71)*x119))+x121+((x119*x122))+((x118*x120))+((x118*x123)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
IkReal x126=((500.0)*sj4);
IkReal x127=(cj2*py);
IkReal x128=(px*sj2);
IkReal x129=(py*sj2);
IkReal x130=(cj2*px);
CheckValue<IkReal> x131 = IKatan2WithCheck(IkReal(((-1278.0)+(((-1.0)*x126*x129))+(((1800.0)*x128))+(((-1.0)*x126*x130))+(((-1800.0)*x127)))),IkReal(((((1800.0)*x130))+(((-1.0)*x126*x127))+(((1800.0)*x129))+((x126*x128))+(((-355.0)*sj4)))),IKFAST_ATAN2_MAGTHRESH);
if(!x131.valid){
continue;
}
CheckValue<IkReal> x132=IKPowWithIntegerCheck(IKsign(((1047.0)+(((-75.0)*(cj4*cj4))))),-1);
if(!x132.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(x131.value)+(((1.5707963267949)*(x132.value))));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[4];
IkReal x133=IKsin(j3);
IkReal x134=IKcos(j3);
IkReal x135=(cj2*px);
IkReal x136=((0.15)*sj4);
IkReal x137=(cj2*py);
IkReal x138=(py*sj2);
IkReal x139=((1.0)*px*sj2);
IkReal x140=(sj2*x133);
evalcond[0]=((((0.54)*x134))+(((-1.0)*x133*x136))+(((-1.0)*x138))+(((-1.0)*x135)));
evalcond[1]=((0.71)+(((0.54)*x133))+x137+(((-1.0)*x139))+((x134*x136)));
evalcond[2]=((-0.54)+((x134*x135))+((x134*x138))+(((-1.0)*x133*x137))+((px*x140))+(((-0.71)*x133)));
evalcond[3]=((((-1.0)*x134*x139))+x136+((x134*x137))+((x133*x135))+((x133*x138))+(((0.71)*x134)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}
}
}
}
return solutions.GetNumSolutions()>0;
}
inline void rotationfunction0(IkSolutionListBase<IkReal>& solutions) {
for(int rotationiter = 0; rotationiter < 1; ++rotationiter) {
IkReal x84=(sj3*sj4);
IkReal x85=((1.0)*cj3);
IkReal x86=(cj5*sj4);
IkReal x87=(cj4*cj5);
IkReal x88=(cj4*sj5);
IkReal x89=((1.0)*sj2);
IkReal x90=((-1.0)*sj2);
IkReal x91=((1.0)*sj4*sj5);
IkReal x92=(((sj3*sj5))+(((-1.0)*x85*x87)));
IkReal x93=((((-1.0)*cj5*x85))+((sj3*x88)));
IkReal x94=(((cj3*x88))+((cj5*sj3)));
IkReal x95=(((sj2*x84))+((cj2*cj3*sj4)));
IkReal x96=(((cj2*x84))+(((-1.0)*sj2*sj4*x85)));
IkReal x97=(cj2*x93);
IkReal x98=((((-1.0)*sj3*x87))+(((-1.0)*sj5*x85)));
IkReal x99=(cj2*x98);
IkReal x100=(((cj2*x94))+((sj2*x93)));
IkReal x101=(x97+(((-1.0)*x89*x94)));
IkReal x102=(((cj2*x92))+((sj2*x98)));
IkReal x103=(x99+(((-1.0)*x89*x92)));
new_r00=(((r10*x102))+((r20*x86))+((r00*x103)));
new_r01=(((r21*x86))+((r01*x103))+((r11*x102)));
new_r02=(((r02*((((x90*x92))+x99))))+((r12*x102))+((r22*x86)));
new_r10=(((cj4*r20))+((r10*x95))+((r00*x96)));
new_r11=(((cj4*r21))+((r11*x95))+((r01*x96)));
new_r12=(((r12*x95))+((cj4*r22))+((r02*x96)));
new_r20=(((r10*x100))+((r00*x101))+(((-1.0)*r20*x91)));
new_r21=(((r01*x101))+((r11*x100))+(((-1.0)*r21*x91)));
new_r22=((((-1.0)*r22*x91))+((r02*((((x90*x94))+x97))))+((r12*x100)));
{
IkReal j6array[1], cj6array[1], sj6array[1];
bool j6valid[1]={false};
_nj6 = 1;
if( IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r01))+IKsqr(new_r00)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j6array[0]=IKatan2(((-1.0)*new_r01), new_r00);
sj6array[0]=IKsin(j6array[0]);
cj6array[0]=IKcos(j6array[0]);
if( j6array[0] > IKPI )
{
    j6array[0]-=IK2PI;
}
else if( j6array[0] < -IKPI )
{    j6array[0]+=IK2PI;
}
j6valid[0] = true;
for(int ij6 = 0; ij6 < 1; ++ij6)
{
if( !j6valid[ij6] )
{
    continue;
}
_ij6[0] = ij6; _ij6[1] = -1;
for(int iij6 = ij6+1; iij6 < 1; ++iij6)
{
if( j6valid[iij6] && IKabs(cj6array[ij6]-cj6array[iij6]) < IKFAST_SOLUTION_THRESH && IKabs(sj6array[ij6]-sj6array[iij6]) < IKFAST_SOLUTION_THRESH )
{
    j6valid[iij6]=false; _ij6[1] = iij6; break; 
}
}
j6 = j6array[ij6]; cj6 = cj6array[ij6]; sj6 = sj6array[ij6];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 17;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}};


/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API bool ComputeIk2(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions, void* pOpenRAVEManip) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API const char* GetKinematicsHash() { return "<robot:GenericRobot - fanuc_crx10ial (8f0693bfc076f85205a9d57bdbc5a8f2)>"; }

IKFAST_API const char* GetIkFastVersion() { return "0x1000004b"; }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>
#ifdef IKFAST_NAMESPACE
using namespace IKFAST_NAMESPACE;
#endif
int main(int argc, char** argv)
{
    if( argc != 12+GetNumFreeParameters()+1 ) {
        printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
               "Returns the ik solutions given the transformation of the end effector specified by\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
               "There are %d free parameters that have to be specified.\n\n",GetNumFreeParameters());
        return 1;
    }

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    IkReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return -1;
    }

    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }
    return 0;
}

#endif