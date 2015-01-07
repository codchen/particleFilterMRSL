//
//  newFilterpdfRelative.h
//  Test
//
//  Created by Xiaoyu Chen on 11/21/14.
//  Copyright (c) 2014 Xiaoyu Chen. All rights reserved.
//

#ifndef Test_newFilterpdfRelative_h
#define Test_newFilterpdfRelative_h
#include "RelativeMultiFlock.h"

#define bearingMu -0.18
#define bearingSig 0.23
#define orientationMu -0.189
#define orientationSig 0.23
#define rangeMu 0.07
#define rangeSig 0.27
#define binSize 0.02

float TempWeightofbin(float error, float bin_size, float mu, float sig);

void newFilterpdfRelative(float* tempWeight, particleCloud* XBar, float bearing, float orientation, float range);

#endif
