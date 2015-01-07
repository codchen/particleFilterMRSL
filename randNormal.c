//
//  randNormal.c
//  Test
//
//  Created by Xiaoyu Chen on 11/21/14.
//  Copyright (c) 2014 Xiaoyu Chen. All rights reserved.
//

#include "randNormal.h"
#include <math.h>
#include <stdlib.h>
float myerf(float x)
{
    float y = 1.0 / ( 1.0 + 0.3275911 * x);
//    printf("%d\n", (((((
//                        + 1.061405429  * y
//                        - 1.453152027) * y
//                       + 1.421413741) * y
//                      - 0.284496736) * y
//                     + 0.254829592) * y)*exp(-x * x) * 0.5 == 0);
//    printf("%d\n", exp(-x * x) == 0);
    float result = (((((
                    + 1.061405429  * y
                    - 1.453152027) * y
                   + 1.421413741) * y
                  - 0.284496736) * y
                 + 0.254829592) * y) * MULTIPLIER * exp (-x * x);
    
//    float ex = exp(-x * x);
//    float ys = (((((
//                    + 1.061405429  * y
//                    - 1.453152027) * y
//                   + 1.421413741) * y
//                  - 0.284496736) * y
//                 + 0.254829592) * y);
    
//    float if1 = result == 1;
    
    return result;
}

float cdf(float x, float mu, float sigma)
{
    float abc =myerf((x - mu) / (sigma * sqrt(2.)));
//    printf("%d\n", 0.5 * (1 + abc) == 0.5);
    return 5 * abc;
}


float gaussrand()
{
    static double U, V;
    static int phase = 0;
    float Z;
    
    if(phase == 0) {
        U = (rand() + 1.) / (RAND_MAX + 2.);
        V = rand() / (RAND_MAX + 1.);
        Z = sqrt(-2 * log(U)) * sin(2 * M_PI * V);
    } else
        Z = sqrt(-2 * log(U)) * cos(2 * M_PI * V);
    
    phase = 1 - phase;
    
    return Z;
}

void randNormal(float* theArray, int size){
    for(int i = 0; i < size; i++){
        *(theArray + i) = gaussrand();
    }
}

void randSample(int* output, float* weight, int size){
    float sum = 0;
    for (int i = 0; i < size; i++){
        sum += *(weight + i);
    }
    for (int i = 0; i < size; i++){
        float dart = sum * (float)rand() / (float)RAND_MAX;
        float accumulated = 0;
        int result = -1;
        for (int j = 0; j < size; j++){
            if (dart < accumulated + *(weight + j)){
                result = j;
                break;
            }
            accumulated += *(weight + j);
        }
        output[i] = result;
    }
}
