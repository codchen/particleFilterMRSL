//
//  randNormal.h
//  Test
//
//  Created by Xiaoyu Chen on 11/21/14.
//  Copyright (c) 2014 Xiaoyu Chen. All rights reserved.
//

#ifndef Test_randNormal_h
#define Test_randNormal_h

#define MULTIPLIER 10000

float myerf(float x);

float cdf(float x, float mu, float sigma);

float gaussrand();

void randNormal(float* theArray, int size);

void randSample(int* output, float* weight, int size);

#endif
