#include "relativeUnifreinject.h"
#include "RelativeMultiFlock.h"
#include <stdio.h>
#include <stdlib.h>
#include "robotmotionSampleStruct.h"
#include "newFilterpdfRelative.h"
#include "Tools.h"
#include "randNormal.h"
#include <math.h>

void particleFilter(particleCloud* original, float bearing, float orientation, float range, float tv, float rv){
    float resample_thresh = 0.1;
    float lowWeightConstant = MULTIPLIER * pow(2, -10.0);
    particleCloud XBar;
    particleCloud* toXBar = &XBar;

    robotmotionSampleStruct(toXBar, original, tv, rv);

    float tempWeight[M];
    for (int i = 0; i < M; i++){
        tempWeight[i] = 1;
    }

	newFilterpdfRelative(tempWeight, toXBar, bearing, orientation, range);
    
	float currentLowestSum = 0;
	for (int i = 0; i < M; i++){
		original -> weight[i] = original -> nextWeight[i] * tempWeight[i];
		if (original -> weight[i] <= lowWeightConstant){
			currentLowestSum += original -> weight[i];
		}
	}

	float reinjectRange = 0.3;
	float reinjectBearing = M_PI / 8;

    polarCoordinates polars;
    polarCoordinates* toPolars = &polars;
    cart2pol(toPolars, toXBar -> partX, toXBar -> partY);
    
	float numBearingError = 0;
	float numRangeError = 0;

	float realThe[M];
	float realRho[M];
	for (int i = 0; i < M; i++){
		realThe[i] = polars.theta[i] - bearing;
		realRho[i] = polars.rho[i] - range;
	}

	normalizeAngle(realThe);
	normalizeAngle(realRho);

	for (int i = 0; i < M; i++){
		if (abs(realThe[i]) > reinjectBearing){
			numBearingError += abs(realThe[i]);
		}
		if (abs(realRho[i]) > reinjectRange){
			numRangeError += abs(realRho[i]);
		}
	}

	if (numBearingError == M || numRangeError == M){
	    original -> counter += 1;
	}
	else{
	     original -> counter -= 1;
	     if (original -> counter < 0){
	         original -> counter = 0;
	     }
	}
	if (original -> counter == 5){
		relativeUnifreinject(toXBar, bearing, orientation, range);
		for (int i = 0; i < M; i++){
			original -> weight[i] = 1;
		}
		original -> counter = 0;
	}
    
    int re_samp[M];
    
	if (currentLowestSum > M * resample_thresh){
		for (int i = 0; i < M; i++){
			if (original -> weight[i] <= 0) original -> weight[i] = FLT_EPSILON;
		}
		//TODO resample !!
        randSample(re_samp, &(original -> weight[0]), M);

		for (int i = 0; i < M; i++){
			original -> weight[i] = 1;
		}
	}
	else{
		//TODO re_sample default value
        for (int i = 0; i < M; i++){
            re_samp[i] = i;
        }
	}
    
    float sum = 0;
    
    for (int i = 0; i < M; i++){
        original -> partX[i] = XBar.partX[re_samp[i]];
        original -> partY[i] = XBar.partY[re_samp[i]];
        original -> partHeading[i] = XBar.partHeading[re_samp[i]];
        original -> weight[i] = original -> weight[re_samp[i]];
        sum += original -> weight[i];
    }
    
    for (int i = 0; i < M; i++){
        original -> weight[i] /= sum;
        original -> nextWeight[i] = original -> weight[i];
    }
	//TODO set new attributes to original

}

void updateGuess(Guess* guess, particleCloud* X){
    float temp[M];
    for (int i = 0; i < M; i++){
//        printf("%f", X -> partX[i]);
//        printf("%f", X -> weight[i]);
        guess -> x += X -> partX[i] * X -> weight[i];
        guess -> y += X -> partY[i] * X -> weight[i];
        temp[i] = X -> weight[i] * X -> partHeading[i];
    }
    normalizeAngle(temp);
    for (int i = 0; i < M; i++){
        guess -> heading += temp[i];
    }
}

void simulationInit(particleCloud* E){
	for (int i = 0; i < NUM_ROBOT; i++){
		particleCloud* current = E + i;
		for (int j = 0; j < M; j++){
            current -> partX[j] = 0;
            current -> partY[j] = 0;
            current -> partHeading[j] = 0;
			current -> weight[j] = 1;
			current -> nextWeight[j] = 1;
		}
        current -> counter = 0;
	}
}
