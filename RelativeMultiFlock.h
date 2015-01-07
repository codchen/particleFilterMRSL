/*
 * RelativeMultiFlock.h
 *
 *  Created on: Oct 31, 2014
 *      Author: chenxiaoyu
 */


#ifndef RELATIVEMULTIFLOCK_H_
#define RELATIVEMULTIFLOCK_H_

#define NUM_ROBOT 4

#define FLT_EPSILON 2.2204460492503131e-16

#define MAX_TIME 1000

#define M 100

#define deltat 0.3

#define dead 0.1

typedef struct{
    float x;
    float y;
    float heading;
}Guess;

typedef struct{
    int RCCTimeStamp;
    int neighbor;
    float nBearing;
    float nOrientation;
    float nRange;
    float nTV;
    float nRV;
    float x;
    float y;
    float heading;
    int ATTimeStamp;
}inputdata;
//
//typedef struct{
//	int t;
//	int numOfNeighbors;
//	int hostID;
//	double deltat;
//    double rv[MAX_TIME];
//    double tv[MAX_TIME];
//    double range[MAX_TIME];
//    double orientation[MAX_TIME];
//    double bearing[MAX_TIME];
//    int Id[MAX_TIME];
//    double p[][][];
//    double pHost[][];
//}inputData;

typedef struct{
    float partX[M];
    float partY[M];
    float partHeading[M];
    float weight[M];
    float nextWeight[M];
    int counter;
}particleCloud;

typedef struct{
    particleCloud particleClouds[NUM_ROBOT];
}particleClouds;

typedef struct{
    float theta[M];
    float rho[M];
}polarCoordinates;

typedef struct{
    float x[M];
    float y[M];
}cartCoordinates;

typedef struct{
    float partX[M];
    float partY[M];
    float partHeading[M];
}reinject;

void particleFilter(particleCloud* original, float bearing, float orientation, float range, float tv, float rv);

void simulationInit(particleCloud* E);

void updateGuess(Guess* guess, particleCloud* X);

#endif /* RELATIVEMULTIFLOCK_H_ */
