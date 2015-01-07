/*
 * robotmotionSampleStruct.c

 *
 *  Created on: Oct 31, 2014
 *      Author: chenxiaoyu
 */
#include "robotmotionSampleStruct.h"
#include "randNormal.h"
#include <stdio.h>
#include <math.h>

void robotmotionSampleStruct(particleCloud* XBar, particleCloud* X, float tv, float rv){
	if (tv !=0 || rv != 0){
		float ntv[M];
		float nrv[M];
		float r[M];
		//TODO implement randNormal
		randNormal(ntv, M);
		randNormal(nrv, M);
		float deltaThe[M];
		for (int i = 0; i < M; i++){
			ntv[i] = ntv[i] * tvnoise + tv;
			nrv[i] = nrv[i] * rvnoise + rv;
			r[i] = ntv[i] / nrv[i];
			deltaThe[i] = nrv[i] * deltat;
			XBar -> partX[i] = X -> partX[i] - r[i] * sin(X -> partHeading[i]) + r[i] * sin(X -> partHeading[i] + deltaThe[i]);
			XBar -> partY[i] = X -> partY[i] + r[i] * cos(X -> partHeading[i]) - r[i] * cos(X -> partHeading[i] + deltaThe[i]);
			XBar -> partHeading[i] = X -> partHeading[i] + deltaThe[i];
		}
	}
}

