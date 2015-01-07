//
//  relativeUnifreinject.h
//  Test
//
//  Created by Xiaoyu Chen on 11/21/14.
//  Copyright (c) 2014 Xiaoyu Chen. All rights reserved.
//

#ifndef Test_relativeUnifreinject_h
#define Test_relativeUnifreinject_h

#include "RelativeMultiFlock.h"

#define rbearingMu -0.18
#define rbearingSig 0.23
#define rorientationMu -0.18
#define rorientationSig 0.24
#define rrangeMu 0.069
#define rrangeSig 0.271

void relativeUnifreinject(particleCloud* XBar, float bearing, float orientation, float range);

#endif
