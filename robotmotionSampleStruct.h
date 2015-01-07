//
//  robotmotionSampleStruct.h
//  Test
//
//  Created by Xiaoyu Chen on 11/21/14.
//  Copyright (c) 2014 Xiaoyu Chen. All rights reserved.
//

#ifndef Test_robotmotionSampleStruct_h
#define Test_robotmotionSampleStruct_h

#include "RelativeMultiFlock.h"

#define tvnoise 0.1
#define rvnoise 0.17

void robotmotionSampleStruct(particleCloud* XBar, particleCloud* X, float tv, float rv);

#endif
