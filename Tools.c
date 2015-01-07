/*
 * Tools.c
 *
 *  Created on: Oct 31, 2014
 *      Author: chenxiaoyu
 */
#include "Tools.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

void cart2pol(polarCoordinates* pc, float* partX, float* partY){
	for (int i = 0; i < M; i++){
		pc -> theta[i] = atan2(partY[i], partX[i]);
		pc -> rho[i] = sqrt(pow(partX[i], 2) + pow(partY[i], 2));
	}
}

void pol2cart(cartCoordinates* cc, float* theta, float* rho){
	for (int i = 0; i < M; i++){
		cc -> x[i] = rho[i] * cos(theta[i]);
		cc -> y[i] = rho[i] * sin(theta[i]);
    }
}

void normalizeAngle(float* x){
	for (int i = 0; i < M; i++){
		if (x[i] > M_PI){
			x[i] -= 2*M_PI;
		}
		else if (x[i] < -M_PI){
			x[i] += 2*M_PI;
		}
	}
	for (int i = 0; i < M; i++){
		if (x[i] > M_PI){
			x[i] -= 2*M_PI;
		}
		else if (x[i] < -M_PI){
			x[i] += 2*M_PI;
		}
	}
}
