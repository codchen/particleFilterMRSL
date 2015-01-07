/*
 * newFilterpdfRelative.c
 *
 *  Created on: Oct 31, 2014
 *      Author: chenxiaoyu
 */
#include "newFilterpdfRelative.h"
#include "randNormal.h"
#include "Tools.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

float TempWeightofbin(float error, float bin_size, float mu, float sig){
    float a = cdf(error + bin_size / 2, mu, sig);
    float b = cdf(error - bin_size / 2, mu, sig);
    return a - b;
}

void newFilterpdfRelative(float* tempWeight, particleCloud* XBar, float bearing, float orientation, float range){
    polarCoordinates pc;
    polarCoordinates* toPc = &pc;
    cart2pol(toPc, &(XBar -> partX[0]), &(XBar -> partY[0]));
	float rawOrientation[M];
	for (int i = 0; i < M; i++){
		rawOrientation[i] = M_PI + pc.theta[i] - XBar -> partHeading[i];
	}
	normalizeAngle(rawOrientation);

	for (int i = 0; i < M; i++){
		if (pc.rho[i] > 1){
			tempWeight[i] = tempWeight[i] * (-10.0/7*(pc.rho[i]-1) + 1);
		}

		if (pc.rho[i] < dead * 0.6){
			tempWeight[i] = 0;
		}

		float rangeError = pc.rho[i] - range;
		float temp = fmod(abs(rangeError), binSize);
		if (temp > binSize / 2){
			rangeError = rangeError + binSize - temp;
		}
		else{
			rangeError = rangeError - temp;
		}
        
		tempWeight[i] = tempWeight[i] * TempWeightofbin(rangeError, binSize, rangeMu, rangeSig);

		float bearingError = pc.theta[i] - bearing;
		temp = fmod(abs(bearingError), binSize);
		if (temp > binSize / 2){
				bearingError = bearingError + binSize - temp;
			}
			else{
				bearingError = bearingError - temp;
			}
		tempWeight[i] *= TempWeightofbin(bearingError, binSize, bearingMu, bearingSig);
        
		float orientationError = rawOrientation[i] - orientation;
		temp = fmod(abs(orientationError), binSize);;
		if (temp > binSize / 2){
            orientationError = orientationError + binSize - temp;
        }
        else{
            orientationError = orientationError - temp;
        }
        
		float abc = TempWeightofbin(orientationError, binSize, orientationMu, orientationSig);
        
        //printf("%f, %f, %f, %f", orientationError, binSize, orientationMu, orientationSig);
        //printf("%d", abc > 0);
        
        tempWeight[i] *= abc;
        
		if (tempWeight[i] == 0){
			tempWeight[i] = FLT_EPSILON;
		}
	}
}

