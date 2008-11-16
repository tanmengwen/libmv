#ifndef _TESTMV_H
#define _TESTMV_H

#include <iostream>
#include <json.h>

#include <testsoon.hpp>
#include <mv.h>

using namespace mv;
using namespace ublas;
using std::cout;
using std::endl;

using namespace json;

#define NotNull(x)        Equals((x)!=NULL,1)
#define Null(x)           Equals((x)==NULL,1)
//#define True(x)           Equals((x),1)
#define Nearby(x, y, eps) Equals(fabs((x)-(y)) < (eps) ,1)
//#define Close(x,y)        Nearby((x),(y), 1e-6)
//#define VeryClose(x,y)    Nearby((x),(y), 1e-10)
#define Small(x)          Close((x),0.0)
#define VerySmall(x)      VeryClose((x),0.0)
#define Zero(x)           Equals((x),0.0)

#define MATDATA(xx) mat xx(xx##_size1,xx##_size2); fill(xx, xx##_raw);
#define VECDATA(xx) vec xx(xx##_size); fill(xx, xx##_raw);

void AllVerySmall(mat &m);
void AllVerySmall(vec &H);
void AllSmall(vec &H);
void AllSmall(mat &m);
void AssertProjectiveEquals(vec &a, vec &b);

#endif // _TESTMV_H
