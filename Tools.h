//
//  Tools.h
//  Test
//
//  Created by Xiaoyu Chen on 11/21/14.
//  Copyright (c) 2014 Xiaoyu Chen. All rights reserved.
//
#ifndef Test_Tools_h
#define Test_Tools_h

#include "RelativeMultiFlock.h"

void cart2pol(polarCoordinates* pc, float* partX, float* partY);
void pol2cart(cartCoordinates* cc, float* theta, float* rho);
void normalizeAngle(float* x);

#endif
